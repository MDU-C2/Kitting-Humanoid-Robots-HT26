"""SDK-free fault injection for VR/MoveIt handover in isolated XR simulation.

No physical G1 interface, ROS actuator topic, or DDS publisher is constructed.
These assertions describe command selection, NOT physical hold/stop safety.
"""
import tempfile
from pathlib import Path
import unittest

import numpy as np

from xr_arm_input_mux import BridgeError, Source, XrArmInputMux
from xr_ipc_protocol import IpcError, send_sample, transact
from xr_vr_passthrough_bridge import VrFirstBridge


class DeterministicHandoverTests(unittest.TestCase):
    def setUp(self):
        self.mux = XrArmInputMux(lambda q: np.full(14, 0.7),
                                 timeout_s=0.35, feedback_timeout_s=0.25)
        self.zero = np.zeros(14)
        self.vr = np.full(14, 0.8)
        self.tau = np.full(14, 0.2)

    def test_uninterrupted_vr_preserves_upstream_ik_and_torque(self):
        self.mux.update_measured(self.zero, 5.0)
        out = self.mux.step(self.vr, self.tau, 5.01)
        self.assertEqual(out.source, Source.VR)
        np.testing.assert_array_equal(out.q, self.vr)
        np.testing.assert_array_equal(out.tau_ff, self.tau)

    def test_moveit_takeover_excludes_vr_and_uses_xr_gravity(self):
        self.mux.update_measured(self.zero, 5.0)
        target = np.full(14, 0.02)
        self.mux.accept_sample(target, 'trajectory', 0, 5.01)
        out = self.mux.step(self.vr, self.tau, 5.02)
        self.assertEqual(out.source, Source.TRAJECTORY)
        np.testing.assert_allclose(out.q, target)
        np.testing.assert_allclose(out.tau_ff, 0.7)

    def test_cancel_goes_to_measured_hold_not_last_command(self):
        self.mux.update_measured(self.zero, 10.0)
        self.mux.accept_sample(np.full(14, 0.06), 'trajectory', 0, 10.01)
        measured = np.full(14, 0.025)
        self.mux.update_measured(measured, 10.02)
        self.mux.accept_sample(measured, 'hold', 1, 10.03)
        out = self.mux.step(self.vr, self.tau, 10.04)
        self.assertEqual(out.source, Source.HOLD)
        np.testing.assert_allclose(out.q, measured)
        np.testing.assert_allclose(out.tau_ff, 0.7)

    def test_command_timeout_holds_and_never_auto_resumes_vr(self):
        self.mux.update_measured(self.zero, 1.0)
        self.mux.accept_sample(self.zero, 'trajectory', 0, 1.01)
        self.mux.update_measured(self.zero, 1.36)
        self.assertEqual(self.mux.step(self.vr, self.tau, 1.37).source, Source.HOLD)
        self.mux.update_measured(self.zero, 1.4)
        self.assertEqual(self.mux.step(self.vr, self.tau, 1.41).source, Source.HOLD)
        with self.assertRaisesRegex(BridgeError, 'too far'):
            self.mux.resume_vr(self.vr, 1.41)
        self.assertEqual(self.mux.source, Source.HOLD)
        self.mux.resume_vr(self.zero, 1.41)
        self.assertEqual(self.mux.step(self.vr, self.tau, 1.42).source, Source.VR)

    def test_stale_feedback_fails_closed_without_claiming_safe_hold(self):
        self.mux.update_measured(self.zero, 7.0)
        self.mux.accept_sample(self.zero, 'trajectory', 0, 7.01)
        with self.assertRaisesRegex(BridgeError, 'stale'):
            self.mux.step(self.vr, self.tau, 7.4)
        self.assertNotEqual(self.mux.source, Source.VR)

    def test_sequence_replay_and_discontinuous_handover_rejected(self):
        self.mux.update_measured(self.zero, 2.0)
        with self.assertRaisesRegex(BridgeError, 'differs'):
            self.mux.accept_sample(self.vr, 'trajectory', 0, 2.01)
        self.assertEqual(self.mux.source, Source.VR)
        self.mux.accept_sample(self.zero, 'trajectory', 0, 2.02)
        with self.assertRaisesRegex(BridgeError, 'Out-of-order'):
            self.mux.accept_sample(self.zero, 'trajectory', 0, 2.03)
        self.assertEqual(self.mux.source, Source.TRAJECTORY)


class IpcOwnershipTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.sock = str(Path(self.tmp.name) / 'offline.sock')
        self.bridge = VrFirstBridge(gravity_fn=lambda q: np.full(14, 0.7),
                                    socket_path=self.sock)
        self.addCleanup(self.bridge.stop)
        self.zero = np.zeros(14)
        self.bridge.start()

    def frame(self, vr_q=None, ready=True):
        q = self.zero if vr_q is None else vr_q
        return self.bridge.frame(self.zero, q, np.full(14, 0.4), vr_ready=ready)

    def test_ipc_takeover_hold_no_automatic_resume_and_manual_resume(self):
        original = self.frame()
        self.assertEqual(original.source, Source.VR)
        self.assertFalse(self.bridge._trajectory_ever_accepted)
        ack = send_sample(np.full(14, 0.02), 'trajectory', 0, self.sock)
        self.assertFalse(ack['hardware_connected'])
        self.assertFalse(ack['executed'])
        self.assertEqual(self.frame(vr_q=np.ones(14)).source, Source.TRAJECTORY)
        send_sample(self.zero, 'hold', 1, self.sock)
        self.assertEqual(self.frame(vr_q=np.ones(14)).source, Source.HOLD)
        with self.assertRaisesRegex(IpcError, 'too far'):
            transact({'kind': 'resume_vr'}, self.sock)
        self.assertEqual(self.frame(vr_q=self.zero).source, Source.HOLD)
        self.assertEqual(transact({'kind': 'resume_vr'}, self.sock)['source'], 'vr')
        self.assertEqual(self.frame().source, Source.VR)

    def test_tracking_unready_blocks_vr_resume_after_takeover(self):
        self.frame()
        send_sample(self.zero, 'trajectory', 0, self.sock)
        self.frame()
        send_sample(self.zero, 'hold', 1, self.sock)
        self.frame(ready=False)
        with self.assertRaisesRegex(IpcError, 'fresh tracking'):
            transact({'kind': 'resume_vr'}, self.sock)
        self.assertEqual(self.bridge.mux.source, Source.HOLD)

    def test_rejected_ipc_takeover_does_not_change_original_vr(self):
        self.frame()
        with self.assertRaisesRegex(IpcError, 'differs'):
            send_sample(np.ones(14), 'trajectory', 0, self.sock)
        self.assertFalse(self.bridge._trajectory_ever_accepted)
        self.assertEqual(self.frame().source, Source.VR)


if __name__ == '__main__':
    unittest.main()
