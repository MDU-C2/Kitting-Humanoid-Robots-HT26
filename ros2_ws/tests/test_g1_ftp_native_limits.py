#!/usr/bin/env python3
"""Offline tests of URDF governed Inspire FTP hand motion, no robot involved."""
import os
from pathlib import Path
import sys
import types
import unittest

THIS = Path(__file__).resolve().parent
BR = Path(os.environ.get('G1_HAND_TEST_BR', str(THIS.parent / 'src/g1_xr_bridge')))
sys.path.insert(0, str(BR))

from g1_ftp_hand_trajectory_math import (
    URDF_LIMITS, joint_names, q_to_motor, motor_to_q, validate_points,
    validate_start_and_range, interpolate,
)
from g1_inspire_ftp_bridge import Cache

class Point:
    def __init__(self, seconds, norm, vel=None):
        self.time_from_start = types.SimpleNamespace(sec=int(seconds), nanosec=int((seconds%1)*1e9))
        self.positions = motor_to_q(norm)
        self.velocities = [] if vel is None else vel
        self.accelerations = []
        self.effort = []

class Defaults:
    @staticmethod
    def get_inspire_hand_ctrl():
        return types.SimpleNamespace(angle_set=[],mode=0)

class Pub:
    def __init__(self):self.messages=[]
    def Write(self,msg):
        self.messages.append((list(msg.angle_set),msg.mode))
        return True

class TestNativeLimits(unittest.TestCase):
    def test_urdf_velocity_and_position_loaded_for_both_sides(self):
        self.assertEqual(len(URDF_LIMITS), 12)
        self.assertEqual(URDF_LIMITS['left_index_1_joint'], (0.0, 1.4381, 1.0))
        self.assertAlmostEqual(URDF_LIMITS['right_thumb_1_joint'][2],1.0)

    def test_no_arbitrary_goal_travel_cap(self):
        initial = [.75]*6
        destination = [.10]*6   # 65% travel, above prior 20% arbitrary cap
        points = validate_points(joint_names('right'), [Point(0,initial),Point(3,destination)],'right')
        validate_start_and_range(initial,points)
        for a,b in zip(interpolate(points,3,initial),destination):self.assertAlmostEqual(a,b)

    def test_speeds_validate_urdf_not_old_normalized_cap(self):
        m = [.8]*6
        end = [.5]*6
        # 0.3 normalized / 1.0 s: formerly rejected by extra 0.2 norm/s.
        # Index finger moves ~0.4314rad/s, below its URDF 1rad/s.
        points=validate_points(joint_names('left'),[Point(0,m),Point(1,end)],'left')
        validate_start_and_range(m,points)
        with self.assertRaisesRegex(ValueError,'URDF joint velocity'):
            validate_points(joint_names('left'),[Point(0,m),Point(.2,end)],'left')

    def test_waypoint_velocity_is_urdf_checked(self):
        norm=[.6]*6
        with self.assertRaisesRegex(ValueError,'waypoint velocity'):
            validate_points(joint_names('left'),[Point(1,norm,[1.5]*6)],'left')

    def test_start_state_needs_sensible_urdf_velocity(self):
        with self.assertRaisesRegex(ValueError,'URDF joint velocity'):
            validate_start_and_range([.8]*6, [(0.1,[.2]*6)])
        validate_start_and_range([.8]*6, [(2.0,[.2]*6)])
        with self.assertRaisesRegex(ValueError,'First t=0'):
            validate_start_and_range([.8]*6,[(0.,[.2]*6)])

    def fresh_bridge(self):
        c=Cache()
        c.motion_enabled=True
        c.default_module=Defaults()
        c.publisher={'left':Pub(),'right':Pub()}
        c.arm_is_active=lambda:True
        class Reading: angle_act=[500]*6
        c.update('left',Reading()); c.update('right',Reading())
        return c

    def test_bridge_no_artificial_002_clamp(self):
        c=self.fresh_bridge()
        lease=c.trajectory_claim('right')['token']
        # In tests, simulate elapsed time sufficient for URDF 1rad/s travel.
        start_time, previous=c.last_trajectory_cmd['right']
        c.last_trajectory_cmd['right']=(start_time-1,previous)
        c.trajectory_target('right',lease,[.3]*6)
        self.assertEqual(c.publisher['right'].messages, [([300]*6,1)])
        self.assertEqual(c.publisher['left'].messages,[])
        c.trajectory_release('right',lease)

    def test_bridge_rejects_excess_urdf_velocity(self):
        c=self.fresh_bridge()
        lease=c.trajectory_claim('left')['token']
        with self.assertRaisesRegex(RuntimeError,'exceed URDF velocity'):
            c.trajectory_target('left',lease,[0.]*6)
        self.assertEqual(c.publisher['left'].messages,[])

    def test_ownership_stale_feedback_and_hold_still_protected(self):
        c=self.fresh_bridge()
        lease=c.trajectory_claim('left')['token']
        with self.assertRaises(RuntimeError):c.step('left','index','close',.02)
        with self.assertRaises(RuntimeError):c.trajectory_target('left','invalid',[.5]*6)
        c.at['left']-=4
        with self.assertRaises(RuntimeError):c.trajectory_target('left',lease,[.5]*6)
        c.at['left']+=4
        c.arm_is_active=lambda:False
        with self.assertRaises(RuntimeError):c.trajectory_target('left',lease,[.5]*6)

if __name__=='__main__': unittest.main(verbosity=2)
