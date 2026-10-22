#!/usr/bin/env python3
"""Offline / hardware-free tests; run from ros2_ws/tests or installer payload dir."""
import math
import os
from pathlib import Path
import sys
import tempfile
import types
import unittest

# Installed under ros2_ws/tests; bridge modules are in ros2_ws/src/g1_xr_bridge.
HERE = Path(__file__).resolve().parent
BR = Path(os.environ['G1_HAND_TEST_BR']) if 'G1_HAND_TEST_BR' in os.environ else (HERE if (HERE/'g1_ftp_hand_math.py').exists() else HERE.parent/'src/g1_xr_bridge')
sys.path.insert(0,str(BR))
from g1_ftp_hand_math import MOTORS
from g1_ftp_hand_trajectory_math import (
  joint_names, q_to_motor, motor_to_q, validate_points, validate_start_and_range,
  interpolate, URDF_LIMITS,
)
from g1_inspire_ftp_bridge import Cache


class Point:
    def __init__(self,t,normalized):
        self.time_from_start = types.SimpleNamespace(sec=int(t),nanosec=int((t-int(t))*1e9))
        self.positions=motor_to_q(normalized)
        self.velocities=[]
        self.accelerations=[]
        self.effort=[]


class Defaults:
    @staticmethod
    def get_inspire_hand_ctrl():
        return types.SimpleNamespace(angle_set=[],mode=None)


class Pub:
    def __init__(self):self.messages=[]
    def Write(self,m):self.messages.append((list(m.angle_set),m.mode));return True


class Tests(unittest.TestCase):
    def test_joint_names_math(self):
        self.assertEqual(joint_names('right')[3],'right_index_1_joint')
        m=[.5,.75,.30,.80,.10,.25]
        self.assertTrue(all(abs(a-b)<1e-9 for a,b in zip(q_to_motor(motor_to_q(m)),m)))
        for values in ([1.0]*5,[float('nan')]*6,[-.2]*6):
            with self.assertRaises(ValueError): q_to_motor(values)

    def test_valid_trajectory_and_interpolation(self):
        m=[.5]*6
        p=[Point(0,m),Point(1,[.45]*6)]
        points=validate_points(list(joint_names('left')),p,'left')
        validate_start_and_range(m,points)
        self.assertAlmostEqual(interpolate(points,.5,m)[3],.475)
        self.assertAlmostEqual(interpolate(points,3,m)[3],.45)

    def test_reorder_joints(self):
        orig=list(joint_names('right'))
        m=[.50,.55,.60,.65,.70,.75]
        p=Point(1,m)
        p.positions.reverse()
        points=validate_points(orig[::-1],[p],'right')
        self.assertEqual([round(x,6) for x in points[0][1]],[round(x,6) for x in m])

    def test_invalid_plans_refused(self):
        names=list(joint_names('left'))
        measured=[.5]*6
        with self.assertRaises(ValueError):validate_points(names+[names[0]],[Point(1,measured)],'left')
        with self.assertRaises(ValueError):validate_points(names,[Point(.4,measured),Point(.4,measured)],'left')
        with self.assertRaises(ValueError):validate_start_and_range(measured, validate_points(names,[Point(.1,[.1]*6)],'left'))
        validate_start_and_range(measured,[(1.,[.1]*6)])
        with self.assertRaises(ValueError):validate_start_and_range(measured,[(0.,[.48]*6)])
        with self.assertRaises(ValueError):validate_start_and_range(measured,[(0.,[.4]*6)])

    def test_native_urdf_limits(self):
        # G1_FTP_NATIVE_URDF_MOTION_LIMITS_V1: no artificial 2% step / 20% travel cap.
        self.assertEqual(URDF_LIMITS['left_index_1_joint'],(0.0,1.4381,1.0))
        m=[.75]*6
        samples=validate_points(joint_names('right'),[Point(0,m),Point(3,[.1]*6)],'right')
        validate_start_and_range(m,samples)

    def new_cache(self):
        c=Cache()
        c.motion_enabled=True
        c.default_module=Defaults()
        c.publisher={'left':Pub(),'right':Pub()}
        c.arm_is_active=lambda:True
        class State: angle_act=[500]*6
        c.update('left',State());c.update('right',State())
        return c

    def test_trajectory_exclusive_hand_ownership(self):
        c=self.new_cache()
        lease=c.trajectory_claim('left')['token']
        self.assertEqual(len(lease),32)
        with self.assertRaises(RuntimeError): c.step('left','index','close',.02)
        with self.assertRaises(RuntimeError): c.trajectory_claim('left')
        with self.assertRaises(RuntimeError): c.trajectory_target('left','invalid',[.49]*6)
        with self.assertRaises(RuntimeError): c.trajectory_target('left',lease,[0.]*6)
        t,q=c.last_trajectory_cmd['left']
        c.last_trajectory_cmd['left']=(t-.1,q)
        c.trajectory_target('left',lease,[.45]*6)
        self.assertEqual(c.publisher['left'].messages[0],([450]*6,1))
        self.assertEqual(c.publisher['right'].messages,[])
        with self.assertRaises(RuntimeError):c.trajectory_release('left','wrong')
        c.trajectory_release('left',lease)
        c.step('left','index','close',.02)
        with self.assertRaises(RuntimeError):c.trajectory_claim('left')

    def test_motion_and_feedback_guards(self):
        c=self.new_cache()
        c.motion_enabled=False
        with self.assertRaises(RuntimeError):c.trajectory_claim('right')
        c.motion_enabled=True
        c.arm_is_active=lambda:False
        with self.assertRaises(RuntimeError):c.trajectory_claim('right')
        c.arm_is_active=lambda:True
        token=c.trajectory_claim('right')['token']
        c.arm_is_active=lambda:False
        with self.assertRaises(RuntimeError):c.trajectory_target('right',token,[.5]*6)
        c.arm_is_active=lambda:True
        c.at['right']-=10
        with self.assertRaises(RuntimeError):c.trajectory_target('right',token,[.5]*6)
        self.assertFalse(c.publisher['right'].messages)

    def test_expired_lease_rejected(self):
        c=self.new_cache()
        token=c.trajectory_claim('left')['token']
        c.trajectory_lease['left']['expires']-=10
        with self.assertRaises(RuntimeError):c.trajectory_target('left',token,[.5]*6)

    def test_installer_and_robot_config(self):
        if not (HERE/'install_g1_ftp_moveit_hands.py').exists():
            self.skipTest('Installer is in external ZIP, not installed tests')
        import install_g1_ftp_moveit_hands as installer
        original=(BR/'g1_inspire_ftp_bridge.py').read_text()
        self.assertIn('G1_FTP_NATIVE_URDF_MOTION_LIMITS_V1',original)
        import ast
        ast.parse(original)


if __name__=='__main__':unittest.main(verbosity=2)
