"""No ROS, no SDK, no DDS: regression checks for pipeline supervisor & safety."""
import ast
from pathlib import Path
import time
import unittest
from types import SimpleNamespace

import numpy as np

import g1_pipeline
from g1_readonly_observer import StateCache
from g1_state_contract import ARM_INDICES, JOINT_NAMES_29
from xr_arm_trajectory import ARM_JOINT_NAMES, Trajectory, TrajectoryError, Waypoint
from xr_motion_guard import XrMotionGuard


class LauncherTests(unittest.TestCase):
    def args(self, *tokens):
        return g1_pipeline.get_args(tokens)

    def test_hardware_always_blocked_and_sim_needs_explicit_isolation(self):
        with self.assertRaisesRegex(ValueError, 'HARDWARE BLOCKED'):
            g1_pipeline.build_commands(self.args('--mode', 'hardware'))
        with self.assertRaisesRegex(ValueError, 'ISOLATED_SIMULATOR'):
            g1_pipeline.build_commands(self.args('--mode', 'xr-sim'))

    def test_offline_no_dds_or_ros2_control(self):
        commands = g1_pipeline.build_commands(self.args())
        flat = ' '.join(' '.join(cmd) for _, cmd, _ in commands)
        self.assertIn('xr_inprocess_offline_harness.py', flat)
        self.assertIn('ros2_trajectory_inprocess_sim.py', flat)
        self.assertIn('xr_pipeline_moveit.launch.py', flat)
        self.assertNotIn('xr_integrated_sim_overlay.py', flat)
        self.assertNotIn('g1_control.launch', flat)
        self.assertNotIn('ChannelFactoryInitialize', flat)

    def test_observe_no_command_server_and_moveit_execution_disabled(self):
        with self.assertRaisesRegex(ValueError, 'interface'):
            g1_pipeline.build_commands(self.args('--mode', 'observe'))
        cmds = g1_pipeline.build_commands(self.args('--mode', 'observe', '--interface', 'eth0'))
        flat = ' '.join(' '.join(cmd) for _, cmd, _ in cmds)
        self.assertIn('g1_readonly_observer.py', flat)
        self.assertIn('enable_execution:=false', flat)
        self.assertNotIn('ros2_trajectory_inprocess_sim.py', flat)
        self.assertNotIn('xr_integrated_sim_overlay.py', flat)

    def test_isolated_sim_keeps_original_xr_motion_and_single_action(self):
        cmds = g1_pipeline.build_commands(self.args('--mode', 'xr-sim',
            '--isolation-ack', 'ISOLATED_SIMULATOR', '--sim-interface', 'eth0'))
        flat = ' '.join(' '.join(cmd) for _, cmd, _ in cmds)
        self.assertIn('--motion', flat)
        self.assertIn('--sim', flat)
        self.assertIn('xr_integrated_sim_overlay.py', flat)
        self.assertIn('ros2_trajectory_inprocess_sim.py', flat)
        self.assertIn('enable_execution:=true', flat)


class SimWatchdogTests(unittest.TestCase):
    def test_ramp_heartbeat_expiry_and_disarm(self):
        guard = XrMotionGuard(ramp_up_s=0.2, ramp_down_s=0.1)
        t = 1000.0
        self.assertEqual(guard.step(t, t), 0.0)
        guard.heartbeat(t)
        # First re-synchronised frame starts at zero weight.
        w = guard.step(t, t + 0.004)
        self.assertGreater(w, 0)
        self.assertLess(w, 0.2)
        for i in range(1, 60):
            now = t + 0.004 + i * 0.004
            guard.heartbeat(now)
            w = guard.step(now, now)
        self.assertGreater(w, 0.9)
        expired = guard.step(t + 0.23, t + 0.60)
        self.assertLess(expired, w)
        guard.disarm()
        self.assertLess(guard.step(t + 0.60, t + 0.62), expired)

    def test_no_blend_without_feedback_even_with_vr_heartbeat(self):
        guard = XrMotionGuard()
        guard.heartbeat(now=50)
        self.assertEqual(guard.step(None, now=50.01), 0)
        self.assertEqual(guard.step(49.0, now=50.02), 0)


class GoalExpansionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Extract the PURE validation function from ROS module; cannot import
        # rclpy in this offline test environment. No function monkeypatching.
        p = Path(__file__).with_name('ros2_trajectory_xr_ipc_dry_run.py')
        tree = ast.parse(p.read_text())
        func = next(x for x in tree.body if isinstance(x, ast.FunctionDef) and x.name == 'convert_goal')
        ns = {'Trajectory': Trajectory, 'Waypoint': Waypoint, 'ARM_JOINT_NAMES': ARM_JOINT_NAMES,
              'TrajectoryError': TrajectoryError, 'np': np}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[func], type_ignores=[])),
                     '<offline ROS action validation>', 'exec'), ns)
        cls.convert = staticmethod(ns['convert_goal'])

    def goal(self, joints, first, last):
        stamp = SimpleNamespace(sec=0, nanosec=0)
        def point(seconds, q):
            return SimpleNamespace(time_from_start=SimpleNamespace(sec=seconds, nanosec=0),
                                   positions=q, velocities=[])
        return SimpleNamespace(header=SimpleNamespace(stamp=stamp), joint_names=joints,
                               points=[point(0, first), point(1, last)])

    def test_left_arm_goal_preserves_right_arm_initial_feedback(self):
        measured = np.arange(14, dtype=float) * 0.01
        goal = self.goal(list(ARM_JOINT_NAMES[:7]), measured[:7].tolist(),
                         (measured[:7] + 0.02).tolist())
        traj = self.convert(goal, measured)
        np.testing.assert_allclose(traj.sample(1)[:7], measured[:7] + 0.02)
        np.testing.assert_allclose(traj.sample(1)[7:], measured[7:])

    def test_right_arm_goal_preserves_left_arm_feedback(self):
        measured = np.arange(14, dtype=float) * 0.01
        names = list(reversed(ARM_JOINT_NAMES[7:]))
        baseline = [float(measured[ARM_JOINT_NAMES.index(j)]) for j in names]
        goal = self.goal(names, baseline, [v + 0.01 for v in baseline])
        traj = self.convert(goal, measured)
        np.testing.assert_allclose(traj.sample(1)[:7], measured[:7])
        np.testing.assert_allclose(traj.sample(1)[7:], measured[7:] + 0.01)

    def test_reject_partial_arm_without_measured_baseline(self):
        goal = self.goal(list(ARM_JOINT_NAMES[:7]), [0]*7, [0.1]*7)
        with self.assertRaises(TrajectoryError):
            self.convert(goal)


class StateContractTests(unittest.TestCase):
    def test_29_slots_and_14_arm_indices_match_xr(self):
        self.assertEqual(len(JOINT_NAMES_29), 29)
        self.assertEqual(tuple(JOINT_NAMES_29[i] for i in ARM_INDICES), ARM_JOINT_NAMES)

    def test_observer_never_returns_unvalidated_state(self):
        cache = StateCache()
        class Motor:
            q = 0.1
            dq = 0.0
        msg = SimpleNamespace(motor_state=[Motor() for _ in range(35)], mode_machine=1)
        self.assertFalse(cache.snapshot()['fresh'])
        cache.update(msg)
        self.assertTrue(cache.snapshot()['fresh'])
        self.assertEqual(len(cache.snapshot()['positions']), 29)
        msg.motor_state[16].q = float('nan')
        cache.update(msg)
        self.assertTrue(np.isfinite(cache.snapshot()['positions']).all())


if __name__ == '__main__':
    unittest.main()
