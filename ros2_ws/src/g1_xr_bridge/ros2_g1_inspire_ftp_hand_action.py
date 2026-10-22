#!/usr/bin/env python3
"""Opt-in, URDF-limited FollowJointTrajectory actions for the Inspire FTP hands.

Uses the existing single-owner FTP bridge over a Unix socket; NO SDK/DDS publisher
and NO modifications to the upstream XR or VR motor controller. Hand motion is
serialised per hand by bridge leases and forbidden during arm trajectories.

NOT hardware calibrated: joint-angle mapping and collision geometry remain
provisional. Require physical supervision and an independent tested stop.
"""
import argparse
import json
import math
import socket
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from g1_ftp_hand_math import SOCKET
from g1_ftp_hand_trajectory_math import (
    GOAL_TOL_NORM, interpolate, joint_names, motor_to_q,
    validate_points, validate_start_and_range,
)

ACK = 'G1_INSPIRE_FTP_TRAJECTORY_SUPERVISED'
CYCLE_S = 0.10
EXTRA_SETTLE_S = 2.0


def ipc(payload):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
        s.settimeout(0.65)
        s.connect(SOCKET)
        s.sendall((json.dumps(payload, allow_nan=False) + '\n').encode())
        raw = s.makefile('rb').readline(16384)
    if not raw:
        raise RuntimeError('Inspire FTP bridge disconnected')
    r = json.loads(raw)
    if r.get('ok') is not True:
        raise RuntimeError('Inspire FTP bridge refused: ' + str(r.get('error')))
    return r


def measured(side):
    reply = ipc({'kind':'status'})
    hand = reply['hands'][side]
    if not hand['fresh'] or hand['age_s'] is None or hand['age_s'] > 0.20:
        raise RuntimeError(f'{side} hand feedback stale')
    if list(joint_names(side)) != hand.get('joint_names'):
        raise RuntimeError('Hand URDF joint feedback order mismatch')
    motor = hand.get('normalized')
    motor_to_q(motor)   # Reject malformed sensor data
    return motor


class HandAction(Node):
    def __init__(self):
        super().__init__('g1_inspire_ftp_trajectory_actions')
        cb = ReentrantCallbackGroup()
        self.actions = []
        for side in ('left','right'):
            self.actions.append(ActionServer(
                self, FollowJointTrajectory,
                f'g1_{side}_ftp_hand/follow_joint_trajectory',
                execute_callback=lambda gh, side=side: self.execute_hand(gh, side),
                goal_callback=lambda request, side=side: self.accept_goal(request, side),
                cancel_callback=lambda gh: CancelResponse.ACCEPT,
                callback_group=cb,
            ))
        self.get_logger().warn(
            'SUPERVISED FTP HAND TRAJECTORIES ENABLED; G1_FTP_NATIVE_URDF_MOTION_LIMITS_V1. '
            'URDF joint positions and velocities govern commanded trajectories; '
            'NO force feedback or physical collision validation. XR HOLD required; '
            'VR hand publisher must not be running.')

    def accept_goal(self, request, side):
        try:
            if request.multi_dof_trajectory.points or request.multi_dof_trajectory.joint_names:
                raise ValueError('Multi-DOF hand trajectories unsupported')
            if request.trajectory.header.stamp.sec or request.trajectory.header.stamp.nanosec:
                raise ValueError('Scheduled future hand trajectory start timestamps unsupported')
            if request.path_tolerance or request.component_path_tolerance or request.component_goal_tolerance:
                raise ValueError('Unsupported hand tolerance fields; cannot silently ignore')
            validate_points(list(request.trajectory.joint_names),
                            list(request.trajectory.points), side)
        except (ValueError, AttributeError) as exc:
            self.get_logger().warn(f'Rejected {side} FTP goal: {exc}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute_hand(self, gh, side):
        result = FollowJointTrajectory.Result()
        lease = None
        try:
            points = validate_points(list(gh.request.trajectory.joint_names),
                                     list(gh.request.trajectory.points), side)
            start_motor = measured(side)
            validate_start_and_range(start_motor, points)
            normalized_goal_tolerances = [GOAL_TOL_NORM]*6
            for t in gh.request.goal_tolerance:
                if t.name not in joint_names(side) or not math.isfinite(t.position) or t.position < 0:
                    raise ValueError('Invalid or unsupported hand goal_tolerance')
                if t.position > 0:
                    idx = joint_names(side).index(t.name)
                    from g1_ftp_hand_math import URDF_MAX_RAD
                    normalized_goal_tolerances[idx] = min(GOAL_TOL_NORM, t.position/URDF_MAX_RAD[idx])
            response = ipc({'kind':'trajectory_claim','side':side,'ack':ACK})
            lease = response['token']
            started = time.monotonic()
            last_activity_at = started
            last_motor = list(start_motor)
            last_log = started
            # Hold must be verified for every physical command by the bridge.
            while rclpy.ok():
                if gh.is_cancel_requested:
                    gh.canceled()
                    result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                    result.error_string = 'Cancelled: hand command stream stopped'
                    return result
                now = time.monotonic()
                t = now - started
                actual = measured(side)
                target = interpolate(points, t, start_motor)
                max_gap = max(abs(a-b) for a,b in zip(target,actual))
                within_goal = all(abs(a-b) <= tol for a,b,tol in zip(target,actual,normalized_goal_tolerances))
                # Report the actual measured six hand joint values.
                fb = FollowJointTrajectory.Feedback()
                fb.joint_names = list(joint_names(side))
                fb.desired = JointTrajectoryPoint(positions=motor_to_q(target))
                fb.actual = JointTrajectoryPoint(positions=motor_to_q(actual))
                fb.error = JointTrajectoryPoint(positions=[a-b for a,b in zip(fb.desired.positions,fb.actual.positions)])
                fb.desired.time_from_start.sec = int(t)
                fb.desired.time_from_start.nanosec = int((t-int(t))*1e9)
                gh.publish_feedback(fb)
                if t >= points[-1][0] and within_goal:
                    # Goal success is based on ACTUAL feedback, not a successful DDS Write.
                    gh.succeed()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = 'Measured hand feedback within normalized tolerance'
                    return result
                if t > points[-1][0] + EXTRA_SETTLE_S:
                    raise RuntimeError(f'Hand goal not reached (max normalized error {max_gap:.4f})')
                if max(abs(a-b) for a,b in zip(last_motor,actual)) >= 0.004:
                    last_activity_at = now
                    last_motor = list(actual)
                if t > 0.7 and max_gap > 0.025 and now-last_activity_at > 1.5:
                    raise RuntimeError('Finger stalled or blocked; no more commands')
                # The time-parameterized MoveIt trajectory supplies the command.
                # No extra arbitrary lead/step clamp or motor-travel cap.
                next_motor = target
                ipc({'kind':'trajectory_target', 'side':side,'token':lease,
                     'normalized':next_motor,'ack':ACK})
                if now - last_log > 1:
                    self.get_logger().info(f'{side} hand trajectory t={t:.1f}s max_error={max_gap:.3f}')
                    last_log = now
                time.sleep(CYCLE_S)
            raise RuntimeError('ROS execution stopped before hand goal completed')
        except (OSError, RuntimeError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
            self.get_logger().error(f'{side} hand trajectory aborted: {exc}')
            gh.abort()
            result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
            result.error_string = str(exc)[:220]
            return result
        finally:
            # Releasing ownership publishes NO new target. The hand's downstream
            # driver may continue retaining its last setpoint.
            if lease is not None:
                try:
                    ipc({'kind':'trajectory_release','side':side,'token':lease})
                except Exception as exc:
                    self.get_logger().error(f'Hand lease release unconfirmed: {exc}')
        return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--operator-ack', required=True)
    args, ros_args = ap.parse_known_args()
    if args.operator_ack != ACK:
        ap.error('Supervised hand trajectory authorization required')
    rclpy.init(args=ros_args)
    n = HandAction()
    exe = MultiThreadedExecutor(num_threads=4)
    exe.add_node(n)
    try:
        exe.spin()
    except KeyboardInterrupt:
        pass
    finally:
        exe.shutdown()
        for server in n.actions:
            server.destroy()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
