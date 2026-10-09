import unittest
import numpy as np
from xr_arm_trajectory import (
    ARM_JOINT_NAMES, ArmCommandRouter, Mode, Trajectory, TrajectoryError, Waypoint
)

class TrajectoryTests(unittest.TestCase):
    def setUp(self):
        self.q0 = np.zeros(14)
        self.q1 = np.ones(14) * 0.2
        self.gravity = lambda q: q * 2.0  # FAKE TORQUE, offline arbitration tests only

    def test_reorders_by_joint_name(self):
        order = list(reversed(ARM_JOINT_NAMES))
        q = np.arange(14) * 0.01
        traj = Trajectory(order, [Waypoint(0, q[::-1])])
        np.testing.assert_allclose(traj.sample(0), q)

    def test_interpolation(self):
        traj = Trajectory(ARM_JOINT_NAMES, [Waypoint(0, self.q0), Waypoint(2, self.q1)])
        np.testing.assert_allclose(traj.sample(1), np.ones(14) * 0.1)

    def test_hermite_velocity(self):
        z = np.zeros(14)
        v = np.ones(14) * 0.2
        traj = Trajectory(ARM_JOINT_NAMES, [Waypoint(0, z, v), Waypoint(2, self.q1, z)])
        self.assertEqual(traj.sample(0).shape, (14,))
        np.testing.assert_allclose(traj.sample(2), self.q1)

    def test_switch_trajectory_then_hold(self):
        router = ArmCommandRouter(self.gravity)
        router.submit_vr(self.q0, self.q0)
        traj = Trajectory(ARM_JOINT_NAMES, [Waypoint(0, self.q0), Waypoint(2, self.q1)])
        router.start_trajectory(traj, measured_q=self.q0, now=10.0)
        cmd = router.tick(11.0)
        self.assertEqual(cmd.source, Mode.TRAJECTORY)
        np.testing.assert_allclose(cmd.q, 0.1)
        np.testing.assert_allclose(cmd.tau_ff, 0.2)
        cmd = router.tick(12.1)
        self.assertEqual(cmd.source, Mode.HOLD)
        np.testing.assert_allclose(router.tick(20).q, self.q1)
        self.assertEqual(router.mode, Mode.HOLD)

    def test_reject_bad_start(self):
        router = ArmCommandRouter(self.gravity)
        traj = Trajectory(ARM_JOINT_NAMES, [Waypoint(0, self.q1)])
        with self.assertRaises(TrajectoryError):
            router.start_trajectory(traj, measured_q=self.q0, now=0.0)

    def test_no_unintended_vr_resume(self):
        router = ArmCommandRouter(self.gravity)
        router.submit_vr(self.q0, self.q0)
        router.cancel_to_hold(self.q1)
        with self.assertRaises(TrajectoryError):
            router.resume_vr(self.q1)
        self.assertEqual(router.mode, Mode.HOLD)
        router.submit_vr(self.q1, self.q0)
        router.resume_vr(self.q1)
        self.assertEqual(router.mode, Mode.VR)

    def test_reject_invalid_names_values(self):
        with self.assertRaises(TrajectoryError):
            Trajectory(ARM_JOINT_NAMES[:-1], [Waypoint(0, self.q0)])
        bad = self.q0.copy(); bad[0] = np.nan
        with self.assertRaises(TrajectoryError):
            Trajectory(ARM_JOINT_NAMES, [Waypoint(0, bad)])

if __name__ == '__main__':
    unittest.main(verbosity=2)
