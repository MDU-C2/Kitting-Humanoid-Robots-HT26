#!/usr/bin/env python3
"""Read-only/offline tests; does not import ROS, DDS or motor controllers."""
import math
import ast
import os
import threading
import time
from unittest.mock import patch
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

def load_arm_modules(bridge):
    sys.path.insert(0, str(bridge))
    from xr_arm_trajectory import ARM_JOINT_NAMES
    from xr_moveit_goal_conversion import convert_goal
    return ARM_JOINT_NAMES, convert_goal


def fake_goal(names, positions, times):
    points = []
    for q, t in zip(positions, times):
        sec = int(t)
        points.append(SimpleNamespace(
            time_from_start=SimpleNamespace(sec=sec, nanosec=int(round((t-sec)*1e9))),
            positions=list(q), velocities=[]))
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0)),
                           joint_names=list(names), points=points)


class CorrectionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import argparse
        p = argparse.ArgumentParser()
        p.add_argument('--repo', default=None)
        p.add_argument('--workspace', default=None)
        p.add_argument('--urdf', default=None)
        cls.cmd_args, _ = p.parse_known_args()
        root = Path(cls.cmd_args.repo or cls.cmd_args.workspace) if (cls.cmd_args.repo or cls.cmd_args.workspace) else (HERE / '..' / 'g1_correction_testrepo')
        src = root / 'ros2_ws/src' if (root / 'ros2_ws/src/g1_xr_bridge').is_dir() else root / 'src'
        bridge = src / 'g1_xr_bridge'
        cls.bridge = bridge
        cls.names, conversion = load_arm_modules(bridge)
        from g1_wrist_pose import WristFK
        from g1_wrist_correction import WristCorrection, CorrectionError, limits_from_urdf
        cls.CorrectionError = CorrectionError
        cls.convert_goal = staticmethod(conversion)
        if cls.cmd_args.urdf:
            urdf = Path(cls.cmd_args.urdf)
        else:
            urdf = src / 'g1_description/urdf/g1_29dof_with_inspire_hand_ftp.urdf'
        if not urdf.is_file():
            raise RuntimeError(f'Missing FTP URDF for offline test: {urdf} (use --urdf)')
        cls.fk = WristFK(urdf, cls.names)
        cls.bounds = limits_from_urdf(urdf, cls.names)
        cls.corrector = WristCorrection(cls.fk, cls.bounds)

    @staticmethod
    def pose():
        q = np.zeros(14)
        q[0] = -0.65
        q[1] = 0.15
        q[3] = 0.65
        q[7] = -0.55
        q[8] = -0.15
        q[10] = 0.60
        return q

    def test_single_point_is_convertible_but_physical_validator_must_check_noop(self):
        q = self.pose()
        msg = fake_goal(self.names[:7], [q[:7]], [0])
        traj = self.convert_goal(msg, q)
        self.assertEqual(len(traj.points), 1)
        self.assertEqual(traj.duration, 0.0)
        np.testing.assert_allclose(traj.sample(0.0), q)
        # A single-point no-op may have a nonzero time_from_start; physical
        # baseline validation still decides whether it actually is a no-op.
        future_one = fake_goal(self.names[:7], [q[:7]], [0.2])
        self.assertAlmostEqual(self.convert_goal(future_one, q).duration, 0.2)

    def test_tiny_two_point_trajectory_is_convertible(self):
        q = self.pose()
        q2 = q.copy(); q2[0] -= 0.001
        msg = fake_goal(self.names[:7], [q[:7], q2[:7]], [0, 0.03])
        traj = self.convert_goal(msg, q)
        self.assertAlmostEqual(traj.duration, .03)
        self.assertAlmostEqual(traj.sample(.03)[0], q2[0])

    def test_no_unvalidated_jump_or_malformed_time(self):
        q = self.pose()
        bad = fake_goal(self.names[:7], [q[:7], q[:7]], [0, 0])
        with self.assertRaises(ValueError):
            self.convert_goal(bad, q)


    def test_action_short_goal_validation_from_source(self):
        # Extract the exact pure validation function from installed action code;
        # no rclpy, DDS, or physical hardware imports.
        action_src = (self.bridge / 'ros2_headless_moveit_action.py').read_text()
        tree = ast.parse(action_src)
        func = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'prepare_trajectory')
        space = {'np': np, 'math': math, 'TrajectoryError': ValueError, 'convert_goal': self.convert_goal}
        from xr_arm_trajectory import ARM_JOINT_NAMES
        space['ARM_JOINT_NAMES'] = ARM_JOINT_NAMES
        exec(compile(ast.Module(body=[func], type_ignores=[]), '<prepare>', 'exec'), space)
        prepare = space['prepare_trajectory']
        q = self.pose()
        one = fake_goal(self.names[:7], [q[:7]], [0])
        caps = np.full(14, 0.5)
        self.assertEqual(len(prepare(one, q, velocity_limits=caps).points), 1)
        nonzero = q.copy(); nonzero[0] += 0.01
        with self.assertRaisesRegex(ValueError, 'Single-point goal differs'):
            prepare(fake_goal(self.names[:7], [nonzero[:7]], [0]), q, velocity_limits=caps)
        tiny = fake_goal(self.names[:7], [q[:7], nonzero[:7]], [0, 0.05])
        self.assertAlmostEqual(prepare(tiny, q, velocity_limits=caps).duration, 0.05)
        fast_q = q.copy(); fast_q[0] += 0.1
        too_fast = fake_goal(self.names[:7], [q[:7], fast_q[:7]], [0, 0.05])
        with self.assertRaisesRegex(ValueError, 'exceeds configured MoveIt joint velocity limits'):
            prepare(too_fast, q, velocity_limits=caps)

    def test_headless_retained_hold_requires_opt_in_and_remains_bounded(self):
        # Test the exact headless IPC acceptance logic, not a second fake motor.
        import xr_headless_moveit as headless
        c = headless.Control.__new__(headless.Control)
        c.np = np
        c.lock = threading.RLock()
        c.active = True
        c.started = time.monotonic() - 5.0
        c.source = 'trajectory'
        c.last_q = self.pose()
        c.last_seq = 5
        c.last_sample_at = time.monotonic()
        measured = self.pose().copy(); measured[0] += 0.02
        c.measured = lambda: (measured, np.zeros(14))
        c.gravity = lambda q: np.zeros(14)
        request = dict(protocol=headless.PROTOCOL, kind='sample', source='hold', seq=6,
                       q=c.last_q.tolist(), retain_final_target=True)
        with patch.dict(os.environ, {'G1_WRIST_CORRECTION_MODE': 'preview'}, clear=False):
            with self.assertRaisesRegex(ValueError, 'Retained HOLD requires'):
                c.handle(request)
        with patch.dict(os.environ, {
                'G1_WRIST_CORRECTION_MODE': 'execute',
                'G1_WRIST_CORRECTION_OPERATOR_ACK': 'G1_WRIST_CORRECTION_SUPERVISED'}, clear=False):
            result = c.handle(request)
        self.assertEqual(result['selected_source'], 'hold')
        np.testing.assert_allclose(c.last_q, self.pose())  # retained command, not measurement
        # Default HOLD must still behave as original XR (capture measured).
        request['seq'] = 7
        request['retain_final_target'] = False
        c.handle(request)
        np.testing.assert_allclose(c.last_q, measured)

    def test_fk_identical_pose_zero_error(self):
        q = self.pose()
        ok, score, errors = self.corrector.metrics(q, q, ('left_wrist_yaw_link', 'right_wrist_yaw_link'))
        self.assertTrue(ok)
        self.assertLess(score, 1e-10)
        out = self.corrector.propose(q, q, q, ('left_wrist_yaw_link',))
        np.testing.assert_allclose(out, q)

    def test_left_correction_reduces_pose_error_without_changing_right_arm(self):
        q = self.pose(); measured = q.copy(); measured[0] += 0.023
        old_ok, old_score, _ = self.corrector.metrics(q, measured, ('left_wrist_yaw_link',))
        self.assertFalse(old_ok)
        proposed = self.corrector.propose(q, measured, q, ('left_wrist_yaw_link',))
        self.assertLessEqual(np.max(np.abs(proposed - q)), 0.008 + 1e-9)
        np.testing.assert_allclose(proposed[7:], q[7:])
        new_ok, new_score, _ = self.corrector.metrics(q, measured + proposed-q, ('left_wrist_yaw_link',))
        self.assertLess(new_score, old_score)

    def test_simulated_constant_undershoot_can_converge(self):
        # Ideal response mock only: real G1 motor response is NOT validated.
        q = self.pose(); bias = np.zeros(14); bias[0] = +0.023
        command = q.copy(); measured = command+bias
        score_initial = self.corrector.metrics(q, measured, ('left_wrist_yaw_link',))[1]
        for _ in range(5):
            done, _, _ = self.corrector.metrics(q, measured, ('left_wrist_yaw_link',))
            if done:
                break
            command = self.corrector.propose(q, measured, command, ('left_wrist_yaw_link',))
            measured = command + bias
        done, score_final, _ = self.corrector.metrics(q, measured, ('left_wrist_yaw_link',))
        self.assertTrue(done)
        self.assertLess(score_final, score_initial)

    def test_joint_limit_blocks_correction(self):
        q = self.pose(); q[0] = self.bounds[0, 0] + 0.001
        a = q.copy(); a[0] += 0.025
        with self.assertRaises(self.CorrectionError):
            self.corrector.propose(q, a, q, ('left_wrist_yaw_link',))

    def test_cumulative_offset_blocks_correction(self):
        q = self.pose(); a = q.copy(); a[0] += 0.023
        cmd = q.copy(); cmd[0] -= 0.034
        with self.assertRaises(self.CorrectionError):
            self.corrector.propose(q, a, cmd, ('left_wrist_yaw_link',))

    def test_invalid_input_fails_closed(self):
        q = self.pose(); a = q.copy(); a[0] = math.nan
        with self.assertRaises(self.CorrectionError):
            self.corrector.propose(q, a, q, ('left_wrist_yaw_link',))
        with self.assertRaises(self.CorrectionError):
            self.corrector.metrics(q, q, ('unknown_wrist',))


if __name__ == '__main__':
    unittest.main(argv=[sys.argv[0]], verbosity=2)
