import tempfile
import os
from pathlib import Path
import time
import unittest

import numpy as np
from xr_arm_input_mux import BridgeError, Source
from xr_inprocess_bridge import XrInProcessBridge
from xr_ipc_protocol import transact, send_sample, IpcError
from xr_runtime_overlay import patch_source


class InProcessTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = str(Path(self.tmp.name) / 'sim.sock')
        self.bridge = XrInProcessBridge(gravity_fn=lambda q: np.asarray(q) + 0.3,
                                        socket_path=self.path)
        self.q = np.zeros(14)

    def tearDown(self):
        self.bridge.stop()
        self.tmp.cleanup()

    def frame(self, q=None, ready=True, vr_q=None):
        q = self.q if q is None else q
        v = self.q if vr_q is None else vr_q
        return self.bridge.frame(q, v, self.q + 0.9, vr_ready=ready)

    def test_original_vr_target_and_torque_pass_through(self):
        selected = self.frame()
        self.assertEqual(selected.source, Source.VR)
        np.testing.assert_allclose(selected.tau_ff, self.q + 0.9)

    def test_trajectory_hold_manual_vr_and_original_torque(self):
        self.bridge.start()
        self.frame()
        ack = send_sample(self.q, 'trajectory', 0, self.path)
        self.assertFalse(ack['hardware_connected'])
        self.assertFalse(ack['executed'])
        self.assertEqual(ack['tau_ff'], [0.3] * 14)
        selected = self.frame()
        self.assertEqual(selected.source, Source.TRAJECTORY)
        np.testing.assert_allclose(selected.tau_ff, 0.3)
        q2 = self.q.copy()
        q2[3] = 0.04
        send_sample(q2, 'trajectory', 1, self.path)
        self.frame(q=q2)
        send_sample(q2, 'hold', 2, self.path)
        selected = self.frame(q=q2, vr_q=q2)
        self.assertEqual(selected.source, Source.HOLD)
        np.testing.assert_allclose(selected.q, q2)
        self.assertEqual(transact({'kind': 'resume_vr'}, self.path)['source'], 'vr')
        self.assertEqual(self.frame(q=q2, vr_q=q2).source, Source.VR)

    def test_reject_if_xr_loop_not_running(self):
        self.bridge.start()
        with self.assertRaisesRegex(IpcError, 'XR loop is not running'):
            send_sample(self.q, 'trajectory', 0, self.path)

    def test_no_resume_with_stale_tracking(self):
        self.bridge.start()
        self.frame(ready=False)
        self.assertEqual(self.bridge.mux.source, Source.HOLD)
        with self.assertRaisesRegex(IpcError, 'fresh tracking'):
            transact({'kind': 'resume_vr'}, self.path)

    def test_start_rejects_discontinuous_target(self):
        self.bridge.start()
        self.frame()
        far = self.q.copy(); far[3] = 0.4
        with self.assertRaisesRegex(IpcError, 'First trajectory'):
            send_sample(far, 'trajectory', 0, self.path)
        self.assertEqual(self.bridge.mux.source, Source.VR)

    def test_fail_closed_on_stale_command(self):
        self.bridge.frame(self.q, self.q, self.q, vr_ready=True, now=20.0)
        self.bridge.mux.accept_sample(self.q, 'trajectory', 0, 20.0)
        # Keep real feedback current, but let the ROS stream expire.
        selected = self.bridge.frame(self.q, self.q, self.q, vr_ready=True, now=20.36)
        self.assertEqual(selected.source, Source.HOLD)

    def test_single_socket_owner(self):
        self.bridge.start()
        other = XrInProcessBridge(gravity_fn=lambda q: self.q, socket_path=self.path)
        with self.assertRaisesRegex(BridgeError, 'socket exists'):
            other.start()


class OverlayTests(unittest.TestCase):
    def test_pinned_xr_main_loop_patch_and_guards(self):
        xr_path = Path(os.environ.get('G1_XR_SOURCE_ROOT', '/workspace/src/xr_teleoperate')) / 'teleop' / 'teleop_hand_and_arm.py'
        if not xr_path.is_file():
            self.skipTest(f'XR source not found: {xr_path}')
        source = xr_path.read_text()
        new = patch_source(source)
        compile(new, 'teleop_hand_and_arm.py', 'exec')
        self.assertIn('g1_bridge.frame(', new)
        self.assertIn('if args.g1_bridge_sim and (not args.sim or not args.motion', new)
        self.assertEqual(new.count('arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)'), 1)
        self.assertIn('g1_bridge.stop()', new)
        self.assertEqual(source.count('g1_bridge.frame'), 0)

    def test_patcher_fails_on_changed_source(self):
        with self.assertRaisesRegex(RuntimeError, 'source anchors|anchor'):
            patch_source('other version')


if __name__ == '__main__':
    unittest.main()
