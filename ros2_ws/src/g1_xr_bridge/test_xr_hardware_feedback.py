"""Unit tests for future physical feedback guard and READ-ONLY pipeline mode."""
import subprocess
import sys
import tempfile
import time
from pathlib import Path
import unittest

import numpy as np

import g1_pipeline
from g1_hardware_preflight import verify_stream
from g1_state_contract import JOINT_NAMES_29
from xr_hardware_feedback import (FeedbackError, TrajectoryTrackingValidator,
                                  validate_lowstate_snapshot)


def fake_state(*, age=0.002):
    return {
        'kind': 'state', 'mode': 'observe', 'hardware_connected': True,
        'read_only': True, 'source': 'lowstate', 'fresh': True,
        'age_s': age, 'joint_names': list(JOINT_NAMES_29),
        'positions': [i * 0.01 for i in range(29)],
        'velocities': [0.0] * 29, 'mode_machine': 2,
    }


class StateValidationTests(unittest.TestCase):
    def test_valid_g1_arm_indices_and_copy(self):
        state = fake_state()
        feedback = validate_lowstate_snapshot(state, now=200.)
        np.testing.assert_allclose(feedback.q, np.arange(15, 29) * 0.01)
        np.testing.assert_allclose(feedback.dq, 0)
        self.assertEqual(feedback.mode_machine, 2)
        self.assertAlmostEqual(feedback.estimated_receipt_at, 199.998)
        state['positions'][15] = 999
        self.assertAlmostEqual(feedback.q[0], 0.15)

    def test_rejects_synthetic_wrong_order_and_unvalidated_inputs(self):
        for field, value in (
            ('read_only', False), ('mode', 'offline'), ('source', 'synthetic'),
            ('fresh', False), ('hardware_connected', False),
            ('joint_names', list(reversed(JOINT_NAMES_29))),
            ('mode_machine', None), ('mode_machine', True),
            ('age_s', 0.4), ('age_s', -1.0), ('age_s', float('nan')),
            ('positions', [0.0] * 29),
        ):
            case = fake_state()
            case[field] = value
            # A valid 29-zero state should pass validation (G1 can be zero).
            if field == 'positions':
                case['positions'][0] = float('nan')
            with self.subTest(field=field, value=str(value)[:25]):
                with self.assertRaises(FeedbackError):
                    validate_lowstate_snapshot(case, now=200.)

    def test_rejects_invalid_velocities_and_short_feedback(self):
        for velocities in ([0.0] * 28, [0.0] * 28 + [float('inf')],
                           [True] + [0.0] * 28):
            state = fake_state()
            state['velocities'] = velocities
            with self.assertRaises(FeedbackError):
                validate_lowstate_snapshot(state, now=200.)


class TrajectoryFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.validator = TrajectoryTrackingValidator()
        self.state = fake_state()
        self.baseline = np.arange(15, 29) * 0.01

    def test_nominal_measured_start_path_and_goal(self):
        self.validator.validate_start(self.state, self.baseline, now=100.)
        feedback, error = self.validator.validate_path(self.state, self.baseline, now=100.)
        self.assertEqual(feedback.q.size, 14)
        self.assertAlmostEqual(error, 0.)
        self.validator.validate_goal(self.state, self.baseline, now=100.)

    def test_rejects_start_discontinuity_tracking_error_and_unreached_goal(self):
        goal = self.baseline.copy()
        goal[3] += 0.2
        with self.assertRaisesRegex(FeedbackError, 'start differs'):
            self.validator.validate_start(self.state, goal, now=100.)
        with self.assertRaisesRegex(FeedbackError, 'tracking error'):
            self.validator.validate_path(self.state, goal, now=100.)
        with self.assertRaisesRegex(FeedbackError, 'goal not reached'):
            self.validator.validate_goal(self.state, goal, now=100.)

    def test_rejects_moving_at_goal_and_stale_during_execution(self):
        self.state['velocities'][15] = 0.11
        with self.assertRaisesRegex(FeedbackError, 'still moving'):
            self.validator.validate_goal(self.state, self.baseline, now=100.)
        self.state['age_s'] = 0.251
        with self.assertRaisesRegex(FeedbackError, 'Stale'):
            self.validator.validate_path(self.state, self.baseline, now=101.)


class ObserverPreflightTests(unittest.TestCase):
    def test_receipt_progress_with_fake_clock(self):
        timer = [100.0]
        def tick():
            return timer[0]
        def snooze(dt):
            timer[0] += dt
        result = verify_stream(lambda: fake_state(age=.002), duration_s=.35,
                               interval_s=.1, clock=tick, sleep=snooze)
        self.assertGreaterEqual(result['samples'], 4)
        self.assertTrue(result['read_only'])
        self.assertFalse(result['execution_enabled'])

    def test_stale_or_frozen_observer_fails(self):
        timer = [100.0]
        def tick():
            return timer[0]
        def snooze(dt):
            timer[0] += dt
        def frozen():
            return fake_state(age=timer[0] - 99.995)
        with self.assertRaises(FeedbackError):
            verify_stream(frozen, duration_s=.4, interval_s=.1,
                          clock=tick, sleep=snooze)

    def test_new_hardware_preflight_launch_has_no_actuator_or_action(self):
        args = g1_pipeline.get_args(['--mode', 'hardware-preflight',
                                     '--interface', 'enx50a03000b0dc'])
        commands = g1_pipeline.build_commands(args)
        rendered = '\n'.join(' '.join(cmd) for _, cmd, _ in commands)
        self.assertIn('g1_readonly_observer.py', rendered)
        self.assertIn('g1_hardware_preflight.py', rendered)
        self.assertIn('g1_joint_state_relay.py --mode observe', rendered)
        self.assertIn('enable_execution:=false', rendered)
        self.assertNotIn('ros2_trajectory_inprocess_sim.py', rendered)
        self.assertNotIn('xr_minimal_sim_overlay.py', rendered)
        self.assertNotIn('teleop_hand_and_arm.py', rendered)
        self.assertNotIn('--motion', rendered)
        with self.assertRaisesRegex(ValueError, 'HARDWARE BLOCKED'):
            g1_pipeline.build_commands(g1_pipeline.get_args(['--mode', 'hardware']))
        with self.assertRaisesRegex(ValueError, '--interface'):
            g1_pipeline.build_commands(g1_pipeline.get_args(['--mode', 'hardware-preflight']))

    def test_no_connection_refuses_without_sdk(self):
        script = Path(__file__).with_name('g1_hardware_preflight.py')
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'absent.sock'
            result = subprocess.run([sys.executable, str(script), '--socket', str(path),
                                     '--duration', '0.2'], capture_output=True,
                                    text=True, timeout=8)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('PREFLIGHT FAILED', result.stderr)


if __name__ == '__main__':
    unittest.main()
