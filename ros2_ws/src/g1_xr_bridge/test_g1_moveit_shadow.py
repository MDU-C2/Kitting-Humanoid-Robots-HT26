"""SDK-free tests for real-position-aligned, never-executing MoveIt shadow."""
from pathlib import Path
from types import SimpleNamespace as Obj
import unittest

import numpy as np

import g1_pipeline
from g1_moveit_shadow_core import inspect_display_plan
from g1_state_contract import JOINT_NAMES_29
from xr_arm_trajectory import ARM_JOINT_NAMES, TrajectoryError
from xr_hardware_feedback import FeedbackError


def snapshot(*, q=None, age=0.005):
    body = list(np.arange(29, dtype=float) * 0.01) if q is None else list(q)
    return dict(kind='state', mode='observe', hardware_connected=True,
                read_only=True, source='lowstate', fresh=True,
                age_s=age, joint_names=list(JOINT_NAMES_29),
                positions=body, velocities=[0.0]*29, mode_machine=6)


def plan(*, names=ARM_JOINT_NAMES, delta=0.03, t0=0, t1=2.0,
         velocities=False, extra=False):
    measured = np.arange(29, dtype=float)[15:29] * 0.01
    start = [float(measured[ARM_JOINT_NAMES.index(name)] if name in ARM_JOINT_NAMES else 0.0) for name in names]
    end = start.copy()
    end[0] += delta
    def point(sec, q):
        seconds = int(sec)
        nano = round((sec - seconds) * 1e9)
        return Obj(time_from_start=Obj(sec=seconds, nanosec=nano),
                   positions=list(q), velocities=([0.] * len(q) if velocities else []))
    jt = Obj(joint_names=list(names), points=[point(t0, start), point(t1, end)])
    return Obj(trajectory=[Obj(joint_trajectory=jt,
                              multi_dof_joint_trajectory=Obj(joint_names=['base'] if extra else [],
                                                            points=[]))])


class MoveItShadowCoreTests(unittest.TestCase):
    def test_real_baseline_full_plan_and_virtual_hold(self):
        result = inspect_display_plan(plan(), snapshot(), now=100.)
        self.assertEqual(result.joint_count, 14)
        self.assertEqual(result.waypoint_count, 2)
        self.assertEqual(result.virtual_samples, 101)
        self.assertEqual(result.final_virtual_source, 'hold')
        self.assertAlmostEqual(result.max_virtual_displacement_rad, 0.03)
        self.assertTrue(result.read_only)
        self.assertFalse(result.execution_enabled)
        self.assertFalse(result.physical_execution)

    def test_single_arm_plan_keeps_measured_other_arm(self):
        for names in (ARM_JOINT_NAMES[:7], ARM_JOINT_NAMES[7:]):
            with self.subTest(names=names[0]):
                result = inspect_display_plan(plan(names=names, velocities=True),
                                              snapshot(), now=100.)
                self.assertEqual(result.joint_count, 7)
                self.assertAlmostEqual(result.max_virtual_displacement_rad, 0.03)

    def test_joint_name_order_reversed(self):
        result = inspect_display_plan(plan(names=tuple(reversed(ARM_JOINT_NAMES))),
                                      snapshot(), now=100.)
        self.assertEqual(result.joint_count, 14)

    def test_start_discontinuity_rejected_against_real_feedback(self):
        with self.assertRaisesRegex(FeedbackError, 'start differs'):
            inspect_display_plan(plan(delta=0.2), snapshot(q=[0.0]*29), now=100.)

    def test_stale_synthetic_wrong_order_rejected(self):
        for updates in (dict(age_s=0.3), dict(source='synthetic'),
                        dict(joint_names=list(reversed(JOINT_NAMES_29))),
                        dict(read_only=False), dict(fresh=False)):
            with self.subTest(updates=updates), self.assertRaises(FeedbackError):
                inspect_display_plan(plan(), dict(snapshot(), **updates), now=100.)

    def test_reject_extraneous_hand_unknown_duplicate_joints(self):
        for names in (ARM_JOINT_NAMES + ('left_thumb_1_joint',),
                      ARM_JOINT_NAMES[:-1], ARM_JOINT_NAMES[:-1] + (ARM_JOINT_NAMES[0],),
                      ARM_JOINT_NAMES[:7] + ('other_joint',)):
            with self.subTest(names=names), self.assertRaises(TrajectoryError):
                inspect_display_plan(plan(names=names), snapshot(), now=100.)

    def test_reject_invalid_time_multiple_paths_multidof(self):
        cases = [plan(t0=0.2), plan(t1=61.), plan(t1=0), plan(extra=True)]
        duplicated = plan()
        duplicated.trajectory.append(duplicated.trajectory[0])
        cases.append(duplicated)
        for message in cases:
            with self.subTest(message=message), self.assertRaises(TrajectoryError):
                inspect_display_plan(message, snapshot(), now=100.)

    def test_reject_nonfinite_or_mismatched_waypoints(self):
        for invalid in (float('nan'), float('inf')):
            msg = plan()
            msg.trajectory[0].joint_trajectory.points[1].positions[0] = invalid
            with self.assertRaises(TrajectoryError):
                inspect_display_plan(msg, snapshot(), now=100.)
        msg = plan()
        msg.trajectory[0].joint_trajectory.points[1].positions.pop()
        with self.assertRaises(TrajectoryError):
            inspect_display_plan(msg, snapshot(), now=100.)


class ShadowLauncherTests(unittest.TestCase):
    def test_opt_in_planned_path_only_and_execution_stays_disabled(self):
        args = g1_pipeline.get_args(['--mode', 'hardware-preflight',
                                     '--interface', 'enx50a03000b0dc',
                                     '--moveit-plan-shadow'])
        plan_commands = g1_pipeline.build_commands(args)
        rendered = '\n'.join(' '.join(cmd) for _, cmd, _ in plan_commands)
        self.assertIn('g1_moveit_shadow_monitor.py', rendered)
        self.assertIn('g1_readonly_observer.py', rendered)
        self.assertIn('enable_execution:=false', rendered)
        self.assertNotIn('ros2_trajectory_inprocess_sim.py', rendered)
        self.assertNotIn('xr_minimal_sim_overlay.py', rendered)
        self.assertNotIn('--motion', rendered)
        self.assertNotIn('teleop_hand_and_arm.py', rendered)

    def test_hardware_motion_still_blocked_and_shadow_requires_moveit(self):
        for mode in ('hardware', 'offline', 'observe', 'xr-sim'):
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                g1_pipeline.build_commands(g1_pipeline.get_args([
                    '--mode', mode, '--moveit-plan-shadow']))
        with self.assertRaises(ValueError):
            g1_pipeline.build_commands(g1_pipeline.get_args([
                '--mode', 'hardware-preflight', '--interface', 'eth0',
                '--moveit-plan-shadow', '--no-moveit']))

    def test_new_modules_have_no_sdk_motor_or_ros_action_authority(self):
        root = Path(__file__).parent
        for filename in ('g1_moveit_shadow_core.py', 'g1_moveit_shadow_monitor.py'):
            contents = (root / filename).read_text()
            for forbidden in ('ChannelPublisher(', 'ActionServer(', 'ctrl_dual_arm(',
                              'send_sample(', 'rt/arm_sdk', 'rt/lowcmd',
                              'G1_29_ArmController(', 'xr_minimal_sim_overlay'):
                self.assertNotIn(forbidden, contents, filename)


if __name__ == '__main__':
    unittest.main()
