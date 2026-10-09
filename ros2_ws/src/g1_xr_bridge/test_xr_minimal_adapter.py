"""SDK-free tests: original XR behavior is preserved until explicit takeover."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest

import numpy as np

import g1_pipeline
from xr_arm_input_mux import BridgeError, Source
from xr_minimal_sim_overlay import XR_ROOT, patch_main
from xr_vr_passthrough_bridge import VrFirstBridge
from xr_ipc_protocol import PROTOCOL, transact, send_sample


class OriginalXrPreservation(unittest.TestCase):
    def test_original_source_unchanged_and_selects_only_at_command(self):
        f = XR_ROOT / 'teleop' / 'teleop_hand_and_arm.py'
        before = f.read_bytes()
        after = patch_main(before.decode())
        compile(after, '<minimal XR overlay>', 'exec')
        self.assertEqual(f.read_bytes(), before)
        self.assertEqual(after.count('arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)'), 1)
        # Exact original constructor, IK call, FTP hands and exit command remain.
        self.assertIn('arm_ctrl = G1_29_ArmController(motion_mode=args.motion, simulation_mode=args.sim)', after)
        self.assertIn('sol_q, sol_tauff  = arm_ik.solve_ik(', after)
        self.assertIn('hand_ctrl = Inspire_Controller_FTP(', after)
        self.assertIn('arm_ctrl.ctrl_dual_arm_go_home()', after)
        self.assertIn('g1_bridge.update_sim_state(arm_ctrl.get_current_motor_q())', after)
        self.assertNotIn('g1_bridge.update_sim_state(', before.decode())
        self.assertNotIn('install_sim_controller_patch', after)
        self.assertNotIn('bridge_motion_guard', after)

    def test_source_drift_refused(self):
        with self.assertRaises(RuntimeError):
            patch_main('not original XR code')

    def test_hardware_invocation_refused_before_sdk_import(self):
        script = Path(__file__).with_name('xr_minimal_sim_overlay.py')
        for flags in ([], ['--motion', '--arm', 'G1_29', '--ee', 'inspire_ftp'],
                      ['--g1-bridge-minimal-sim', '--motion', '--arm', 'G1_29', '--ee', 'inspire_ftp']):
            proc = subprocess.run([sys.executable, str(script), *flags],
                                  capture_output=True, text=True, timeout=5)
            self.assertNotEqual(proc.returncode, 0)
            self.assertIn('HARDWARE BLOCKED', proc.stdout + proc.stderr)

    def test_simulator_requires_isolation(self):
        script = Path(__file__).with_name('xr_minimal_sim_overlay.py')
        env = os.environ.copy()
        env.pop('G1_XR_SIM_ACK', None)
        proc = subprocess.run([sys.executable, str(script), '--g1-bridge-minimal-sim',
                               '--sim', '--motion', '--arm', 'G1_29', '--ee', 'inspire_ftp'],
                              env=env, capture_output=True, text=True, timeout=5)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn('Simulator isolation not acknowledged', proc.stdout + proc.stderr)

    def test_launcher_option_preserves_existing_default(self):
        args = ['--mode', 'xr-sim', '--isolation-ack', 'ISOLATED_SIMULATOR',
                '--sim-interface', 'sim0']
        old = g1_pipeline.build_commands(g1_pipeline.get_args(args))
        new = g1_pipeline.build_commands(g1_pipeline.get_args(args + ['--xr-adapter', 'minimal']))
        self.assertIn('xr_integrated_sim_overlay.py', ' '.join(old[0][1]))
        self.assertIn('xr_minimal_sim_overlay.py', ' '.join(new[0][1]))
        self.assertIn('--g1-bridge-minimal-sim', new[0][1])
        self.assertIn('--sim', new[0][1])
        self.assertIn('--motion', new[0][1])
        with self.assertRaisesRegex(ValueError, 'HARDWARE BLOCKED'):
            g1_pipeline.build_commands(g1_pipeline.get_args(['--mode', 'hardware', '--xr-adapter', 'minimal']))


class VrFirstTests(unittest.TestCase):
    def make(self):
        temp = tempfile.TemporaryDirectory()
        b = VrFirstBridge(gravity_fn=lambda q: np.full(14, 0.7),
                          socket_path=str(Path(temp.name) / 'sim-only.sock'))
        self.addCleanup(temp.cleanup)
        return b

    def test_initial_vr_passes_exact_original_values_even_before_tracking(self):
        b = self.make()
        q = np.arange(14, dtype=np.float64) / 100
        tau = np.arange(14, dtype=np.float64) / 50
        chosen = b.frame(np.zeros(14), q, tau, vr_ready=False)
        self.assertIs(chosen.q, q)
        self.assertIs(chosen.tau_ff, tau)
        self.assertEqual(chosen.source, Source.VR)
        self.assertFalse(b._trajectory_ever_accepted)
        self.assertEqual(b.mux.source, Source.VR)

    def test_rejected_moveit_command_keeps_original_vr_passthrough(self):
        b = self.make()
        q = np.zeros(14)
        b.frame(q, q, q, vr_ready=True)
        with self.assertRaisesRegex(BridgeError, 'differs from measured'):
            b.handle({'protocol': PROTOCOL, 'kind': 'sample', 'source': 'trajectory',
                      'seq': 0, 'q': [1.0] * 14})
        self.assertFalse(b._trajectory_ever_accepted)
        self.assertIs(b.frame(q, q, q, vr_ready=True).q, q)

    def test_explicit_takeover_then_hold_no_automatic_vr_return(self):
        b = self.make()
        zero = np.zeros(14)
        b.frame(zero, zero, np.zeros(14), vr_ready=True)
        sample = b.handle({'protocol': PROTOCOL, 'kind': 'sample',
                           'source': 'trajectory', 'seq': 0, 'q': [0.02] * 14})
        self.assertEqual(sample['kind'], 'sample_ack')
        self.assertFalse(sample['executed'])
        self.assertTrue(b._trajectory_ever_accepted)
        chosen = b.frame(np.zeros(14), np.ones(14), np.ones(14), vr_ready=True)
        self.assertEqual(chosen.source, Source.TRAJECTORY)
        np.testing.assert_allclose(chosen.q, 0.02)
        np.testing.assert_allclose(chosen.tau_ff, 0.7)
        self.assertNotEqual(chosen.q[0], 1.0)
        # With no samples, the existing timeout logic requests HOLD, not VR.
        time.sleep(0.38)
        hold = b.frame(np.zeros(14), np.ones(14), np.ones(14), vr_ready=True)
        self.assertEqual(hold.source, Source.HOLD)
        np.testing.assert_allclose(hold.q, 0.0)
        self.assertFalse(b._trajectory_ever_accepted is False)

    def test_vr_first_cannot_resume_from_cached_target_after_tracking_loss(self):
        b = self.make()
        zero = np.zeros(14)
        b.frame(zero, zero, zero, vr_ready=True)
        b.handle({'protocol': PROTOCOL, 'kind': 'sample',
                  'source': 'trajectory', 'seq': 0, 'q': [0.02] * 14})
        b.frame(zero, zero, zero, vr_ready=True)
        b.mux.enter_hold(time.monotonic())
        b.frame(zero, zero, zero, vr_ready=False)
        with self.assertRaisesRegex(BridgeError, 'fresh tracking'):
            b.handle({'protocol': PROTOCOL, 'kind': 'resume_vr'})
        self.assertIs(b.mux.source, Source.HOLD)
        b.frame(zero, zero, zero, vr_ready=True)
        self.assertEqual(b.handle({'protocol': PROTOCOL, 'kind': 'resume_vr'})['source'], 'vr')

    def test_unix_socket_vr_moveit_hold_resume_without_motor_commands(self):
        """Real IPC transport, original VR passthrough, virtual source switching."""
        b = self.make()
        self.addCleanup(b.stop)
        b.start()
        zero = np.zeros(14)
        original_tau = np.full(14, 0.41)
        original = b.frame(zero, zero, original_tau, vr_ready=True)
        self.assertIs(original.tau_ff, original_tau)
        b.update_sim_state(np.arange(29, dtype=float) / 100)
        status = transact({'kind': 'status'}, b._socket_path)
        self.assertEqual(len(status['sim_joint_positions']), 29)
        self.assertFalse(status['hardware_connected'])
        ack = send_sample(np.full(14, 0.01), 'trajectory', 0, b._socket_path)
        self.assertFalse(ack['executed'])
        self.assertEqual(ack['selected_source'], 'trajectory')
        chosen = b.frame(zero, np.ones(14), original_tau, vr_ready=True)
        self.assertIs(chosen.source, Source.TRAJECTORY)
        np.testing.assert_allclose(chosen.q, 0.01)
        np.testing.assert_allclose(chosen.tau_ff, 0.7)
        hold_ack = send_sample(zero, 'hold', 1, b._socket_path)
        self.assertEqual(hold_ack['selected_source'], 'hold')
        chosen = b.frame(zero, zero, original_tau, vr_ready=True)
        self.assertIs(chosen.source, Source.HOLD)
        self.assertEqual(transact({'kind': 'resume_vr'}, b._socket_path)['source'], 'vr')
        chosen = b.frame(zero, zero, original_tau, vr_ready=True)
        self.assertIs(chosen.source, Source.VR)
        np.testing.assert_allclose(chosen.tau_ff, original_tau)

    def test_vr_resume_requires_target_near_measured(self):
        b = self.make()
        zero = np.zeros(14)
        b.frame(zero, zero, zero, vr_ready=True)
        b.handle({'protocol': PROTOCOL, 'kind': 'sample', 'source': 'trajectory',
                  'seq': 0, 'q': [0.02]*14})
        b.frame(zero, zero, zero, vr_ready=True)
        b.handle({'protocol': PROTOCOL, 'kind': 'sample', 'source': 'hold',
                  'seq': 1, 'q': [0.0]*14})
        b.frame(zero, np.ones(14), np.ones(14), vr_ready=True)
        with self.assertRaisesRegex(BridgeError, 'too far from measured'):
            b.handle({'protocol': PROTOCOL, 'kind': 'resume_vr'})
        b.frame(zero, zero, zero, vr_ready=True)
        result = b.handle({'protocol': PROTOCOL, 'kind': 'resume_vr'})
        self.assertEqual(result['source'], 'vr')
        selected = b.frame(zero, zero, np.full(14, 0.23), vr_ready=True)
        self.assertEqual(selected.source, Source.VR)
        np.testing.assert_allclose(selected.tau_ff, 0.23)


if __name__ == '__main__':
    unittest.main()
