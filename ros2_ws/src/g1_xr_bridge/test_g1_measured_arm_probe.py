"""SDK-free regression tests: real measured arm input into read-only selector."""
import math
from pathlib import Path
import unittest

import numpy as np

import g1_pipeline
from g1_measured_arm_probe import run_probe
from g1_state_contract import JOINT_NAMES_29
from xr_arm_input_mux import Source
from xr_hardware_feedback import FeedbackError
from xr_measured_arm_probe import ReadOnlyMeasuredArmProbe


def measured_state(*, age=0.005, q=None):
    return {'kind': 'state', 'mode': 'observe', 'hardware_connected': True,
            'read_only': True, 'source': 'lowstate', 'fresh': True,
            'age_s': age, 'joint_names': list(JOINT_NAMES_29),
            'positions': ([0.0] * 29 if q is None else list(q)),
            'velocities': [0.0] * 29, 'mode_machine': 6}


class MeasuredSelectorTests(unittest.TestCase):
    def setUp(self):
        self.probe = ReadOnlyMeasuredArmProbe()
        self.full_q = np.arange(29, dtype=float) * 0.01
        self.arm = self.full_q[15:29].copy()
        self.snapshot = measured_state(q=self.full_q)

    def test_real_ordered_lowstate_drives_same_mux_and_no_motor_output(self):
        status = self.probe.preview_start(self.snapshot, self.arm, now=100.)
        self.assertEqual(status.source, Source.TRAJECTORY.value)
        self.assertFalse(status.physically_executed)
        self.assertEqual(status.max_tracking_error_rad, 0.)
        np.testing.assert_allclose(status.arm_positions, self.arm)
        np.testing.assert_allclose(self.probe._mux._measured, self.arm)
        self.assertAlmostEqual(self.probe._mux._measured_at, 99.995)
        self.assertEqual(self.probe._mux.source, Source.TRAJECTORY)

    def test_hardware_snapshot_required_no_synthetic_or_wrong_order(self):
        for field, value in (('mode', 'offline'), ('source', 'synthetic'),
                             ('read_only', False), ('fresh', False),
                             ('joint_names', list(reversed(JOINT_NAMES_29)))):
            with self.subTest(field=field):
                bad = dict(self.snapshot)
                bad[field] = value
                with self.assertRaises(FeedbackError):
                    self.probe.preview_start(bad, self.arm, now=100.)
                self.assertEqual(self.probe._mux.source, Source.VR)

    def test_start_rejected_if_not_at_measured_pose(self):
        far = self.arm.copy()
        far[3] += 0.12
        with self.assertRaisesRegex(FeedbackError, 'start differs'):
            self.probe.preview_start(self.snapshot, far, now=100.)
        self.assertEqual(self.probe._mux.source, Source.VR)

    def test_path_error_check_before_sample_and_hold_never_returns_vr(self):
        self.probe.preview_start(self.snapshot, self.arm, now=100.)
        far = self.arm.copy()
        far[3] += 0.18
        with self.assertRaisesRegex(FeedbackError, 'tracking error'):
            self.probe.preview_path(self.snapshot, far, now=100.05)
        self.assertEqual(self.probe._mux._last_seq, 0)
        state = self.probe.preview_path(self.snapshot, self.arm, now=100.1)
        self.assertEqual(state.source, Source.TRAJECTORY.value)
        self.assertEqual(self.probe._mux._last_seq, 1)
        stopped = self.probe.preview_hold(self.snapshot, now=100.15)
        self.assertEqual(stopped.source, Source.HOLD.value)
        self.assertFalse(stopped.physically_executed)
        self.assertEqual(self.probe._mux.source, Source.HOLD)

    def test_not_refreshing_poll_clock_or_allowing_old_receipt(self):
        self.probe.preview_start(self.snapshot, self.arm, now=100.)
        old = measured_state(age=0.20, q=self.full_q)
        # Snapshot itself says 'fresh', but its estimated receive time
        # is *older* than last receipt and must be rejected.
        with self.assertRaisesRegex(FeedbackError, 'backwards'):
            self.probe.preview_path(old, self.arm, now=100.01)
        self.assertAlmostEqual(self.probe._mux._measured_at, 99.995)
        with self.assertRaisesRegex(FeedbackError, 'Stale'):
            self.probe.preview_path(measured_state(age=0.3, q=self.full_q),
                                    self.arm, now=101.)

    def test_probe_no_torque_function_is_available(self):
        self.probe.preview_start(self.snapshot, self.arm, now=100.)
        with self.assertRaisesRegex(RuntimeError, 'never calculate actuator torques'):
            self.probe._mux.step(self.arm, np.zeros(14), 100.)

    def test_missing_baseline_rejected(self):
        with self.assertRaisesRegex(FeedbackError, 'No preview'):
            self.probe.preview_path(self.snapshot, self.arm, now=100.)


class ReadOnlyProbeStreamTests(unittest.TestCase):
    def test_progress_with_fresh_actual_feedback(self):
        clock_val = [100.]
        def clock(): return clock_val[0]
        def sleep(delay): clock_val[0] += delay
        summary = run_probe(lambda: measured_state(q=[0.0]*29),
                            clock=clock, sleep=sleep, duration_s=0.4, interval_s=0.1)
        self.assertEqual(summary['validated_arm_joints'], 14)
        self.assertEqual(summary['hypothetical_selected_source'], 'trajectory')
        self.assertTrue(summary['read_only'])
        self.assertFalse(summary['execution_enabled'])
        self.assertFalse(summary['physical_execution'])
        self.assertGreater(summary['receipt_progress_s'], 0.2)

    def test_frozen_feedback_stream_rejected(self):
        clock_val = [100.]
        def clock(): return clock_val[0]
        def sleep(delay): clock_val[0] += delay
        def stale():
            return measured_state(q=[0.0]*29, age=max(0., clock_val[0]-99.995))
        with self.assertRaisesRegex(FeedbackError, 'Stale|progress'):
            run_probe(stale, clock=clock, sleep=sleep, duration_s=0.5, interval_s=0.1)

    def test_launcher_opt_in_and_hardware_still_blocked(self):
        args = g1_pipeline.get_args(['--mode', 'hardware-preflight',
                                     '--interface', 'enx50a03000b0dc',
                                     '--arm-selector-probe'])
        plan = g1_pipeline.build_commands(args)
        rendered = '\n'.join(' '.join(cmd) for _, cmd, _ in plan)
        self.assertIn('g1_measured_arm_probe.py --watch', rendered)
        self.assertIn('g1_readonly_observer.py', rendered)
        self.assertIn('enable_execution:=false', rendered)
        self.assertNotIn('ros2_trajectory_inprocess_sim.py', rendered)
        self.assertNotIn('xr_minimal_sim_overlay.py', rendered)
        self.assertNotIn('teleop_hand_and_arm.py', rendered)
        self.assertNotIn('--motion', rendered)
        self.assertNotIn('--sim', rendered)
        for bad in (['--mode', 'offline', '--arm-selector-probe'],
                    ['--mode', 'hardware', '--arm-selector-probe'],
                    ['--mode', 'xr-sim', '--arm-selector-probe']):
            with self.subTest(args=bad), self.assertRaises(ValueError):
                g1_pipeline.build_commands(g1_pipeline.get_args(bad))
        without = g1_pipeline.build_commands(g1_pipeline.get_args([
            '--mode', 'hardware-preflight', '--interface', 'enx50a03000b0dc']))
        self.assertNotIn('g1_measured_arm_probe', str(without))

    def test_no_actuator_api_or_ros_action_in_new_files(self):
        root = Path(__file__).parent
        for name in ('xr_measured_arm_probe.py', 'g1_measured_arm_probe.py'):
            src = (root / name).read_text()
            for forbidden in ('ChannelPublisher(', 'ChannelFactoryInitialize(',
                              'ActionServer(', 'ctrl_dual_arm(', 'send_sample(',
                              'rt/arm_sdk', 'rt/lowcmd'):
                self.assertNotIn(forbidden, src, msg=name)


if __name__ == '__main__':
    unittest.main()
