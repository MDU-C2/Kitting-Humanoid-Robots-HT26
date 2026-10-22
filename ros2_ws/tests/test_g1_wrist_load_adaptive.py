#!/usr/bin/env python3
"""Offline numerical tests. No ROS/DDS or physical actuation."""
import argparse
import sys
from pathlib import Path
import unittest
import numpy as np

HERE=Path(__file__).resolve().parent
bridge_arg = next((a.split('=',1)[1] for a in sys.argv if a.startswith('--bridge=')), str(HERE.parent / 'src/g1_xr_bridge'))
urdf_arg = next((a.split('=',1)[1] for a in sys.argv if a.startswith('--urdf=')), str(HERE.parent / 'src/g1_description/urdf/g1_29dof_with_inspire_hand_ftp.urdf'))
sys.path.insert(0, bridge_arg)
from g1_wrist_correction import WristCorrection, JointResponseEstimator, limits_from_urdf, CorrectionError
from g1_wrist_pose import WristFK
from xr_arm_trajectory import ARM_JOINT_NAMES


class LoadAdaptiveTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not Path(urdf_arg).is_file():
            raise RuntimeError('FTP URDF missing; supply --urdf=/path/to/g1_29dof_with_inspire_hand_ftp.urdf')
        cls.fk = WristFK(urdf_arg, ARM_JOINT_NAMES)
        cls.corrector = WristCorrection(cls.fk, limits_from_urdf(urdf_arg, ARM_JOINT_NAMES))

    @staticmethod
    def pose():
        q = np.zeros(14)
        q[0],q[1],q[3],q[7],q[8],q[10] = -0.65, 0.15, 0.65, -0.75, -0.15, 0.60
        return q

    def test_no_change_at_goal(self):
        goal=self.pose()
        out=self.corrector.propose(goal,goal,goal,('right_wrist_yaw_link',),np.ones(14))
        np.testing.assert_allclose(out,goal)

    def test_gain_observation_clamped_and_filtered(self):
        r=JointResponseEstimator()
        cmd=np.zeros(14); measured=np.zeros(14)
        cmd[7]=-0.004; measured[7]=-0.00008
        cmd[10]=-0.003; measured[10]=-0.0027
        idx=r.observe(cmd,measured)
        self.assertEqual(set(idx),{7,10})
        self.assertTrue(.15 <= r.gains[7] < .4)
        self.assertTrue(.8 < r.gains[10] < 1.0)
        self.assertEqual(r.samples[7],1)

    def test_ignore_very_small_commands(self):
        r=JointResponseEstimator()
        c=np.zeros(14); a=np.zeros(14)
        c[7]=0.0005; a[7]=0.001
        r.observe(c,a)
        self.assertEqual(r.samples[7],0)
        self.assertEqual(r.gains[7],1)

    def test_wrong_way_motion_rejected(self):
        r=JointResponseEstimator()
        c=np.zeros(14); a=np.zeros(14)
        c[7]=-0.004; a[7]=+0.001
        with self.assertRaisesRegex(ValueError,'opposite'):
            r.observe(c,a)

    def test_weighting_changes_joint_commands_under_load(self):
        q=self.pose()
        measured=q.copy(); measured[7]+=0.023
        ideal=self.corrector.propose(q,measured,q,('right_wrist_yaw_link',))
        gains=np.ones(14); gains[7]=0.15; gains[10]=0.9
        adaptive=self.corrector.propose(q,measured,q,('right_wrist_yaw_link',),gains)
        self.assertGreater(np.linalg.norm(adaptive - ideal),1e-5)
        self.assertLessEqual(np.max(np.abs(adaptive-q)),0.00800001)
        self.assertLess(self.corrector.metrics(q,measured+gains*(adaptive-q),('right_wrist_yaw_link',))[1],
                        self.corrector.metrics(q,measured,('right_wrist_yaw_link',))[1])

    def test_wrong_gain_or_nonfinite_fails(self):
        q=self.pose(); m=q.copy(); m[7]+=0.023
        bad=np.ones(14); bad[7]=5
        with self.assertRaises(CorrectionError):
            self.corrector.propose(q,m,q,('right_wrist_yaw_link',),bad)
        bad[7]=np.nan
        with self.assertRaises(CorrectionError):
            self.corrector.propose(q,m,q,('right_wrist_yaw_link',),bad)

    def test_synthetic_loaded_shoulder_response_improves_final_pose(self):
        goal=self.pose(); bias=np.zeros(14); bias[7]=0.022; bias[10]=0.005
        gains=np.ones(14); gains[7]=0.08; gains[10]=0.88
        command=goal.copy(); measured=command + bias
        estimator=JointResponseEstimator()
        initial_score=self.corrector.metrics(goal,measured,('right_wrist_yaw_link',))[1]
        scores=[initial_score]
        for _ in range(5):
            if self.corrector.metrics(goal,measured,('right_wrist_yaw_link',))[0]:
                break
            try:
                proposed=self.corrector.propose(goal,measured,command,('right_wrist_yaw_link',),estimator.gains)
            except CorrectionError:
                break
            next_measured=measured + gains*(proposed-command)
            estimator.observe(proposed-command, next_measured-measured)
            measured=next_measured
            command=proposed
            scores.append(self.corrector.metrics(goal,measured,('right_wrist_yaw_link',))[1])
        self.assertLess(scores[-1], initial_score)
        self.assertLessEqual(np.max(np.abs(command-goal)),0.035+1e-9)

if __name__=='__main__':
    args=[a for a in sys.argv if not (a.startswith('--bridge=') or a.startswith('--urdf='))]
    unittest.main(argv=args,verbosity=2)
