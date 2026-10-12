#!/usr/bin/env python3
"""ROS Humble FollowJointTrajectory -> physical headless XR Unix socket.

Use ONLY for supervised, collision-checked G1 MoveIt trajectories. This endpoint
is separate from offline/simulation endpoints; no synthetic feedback is used.
"""
import math
import time
from threading import Lock

import numpy as np
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from xr_arm_trajectory import ARM_JOINT_NAMES, TrajectoryError
from ros2_trajectory_xr_ipc_dry_run import convert_goal
from xr_ipc_protocol import transact

ACTION = '/g1_xr_bridge/follow_joint_trajectory'
SOCKET = '/tmp/g1_xr_headless_moveit.sock'


def request(msg):
    result = transact(msg, SOCKET, timeout=0.6)
    if not result.get('hardware_connected') is True or result.get('mode') == 'sim':
        raise RuntimeError('XR controller is not the physical headless motion-mode server')
    return result


def state():
    result = request({'kind': 'status'})
    if (result.get('kind') != 'status_ack' or result.get('feedback_fresh') is not True or
            result.get('xr_loop_fresh') is not True):
        raise RuntimeError('Stale/absent physical XR feedback')
    q = np.asarray(result.get('measured_arm_positions'), dtype=float)
    dq = np.asarray(result.get('measured_arm_velocities'), dtype=float)
    if q.shape != (14,) or dq.shape != (14,) or not np.isfinite(q).all() or not np.isfinite(dq).all():
        raise RuntimeError('Invalid physical 14-arm feedback')
    return result, q, dq


def send(q, source, seq):
    result = request({'kind': 'sample', 'source': source, 'seq': int(seq),
                      'q': np.asarray(q, dtype=float).tolist()})
    if (result.get('kind') != 'sample_ack' or result.get('seq') != seq or
            result.get('selected_source') != source):
        raise RuntimeError('XR did not accept selected sample')
    return result


def prepare_trajectory(msg, baseline, max_displacement_rad=0.12, max_speed_rad_s=0.20):
    trajectory = convert_goal(msg, baseline)
    if trajectory.duration < 0.5 or trajectory.duration > 12:
        raise TrajectoryError('Initial hardware trials accept duration 0.5..12 seconds')
    initial_q = trajectory.points[0][1]
    if np.max(np.abs(initial_q - baseline)) > 0.05:
        raise TrajectoryError('First planned point must agree with physical G1 arms within 0.05rad')
    # Deliberately restrict the first physical experiment to tiny motions.
    times = list(np.arange(0, trajectory.duration, 0.02)) + [trajectory.duration]
    samples = [trajectory.sample(float(t)) for t in times]
    deviations = np.abs(np.asarray(samples) - baseline)
    sample_idx, joint_idx = np.unravel_index(np.argmax(deviations), deviations.shape)
    worst = float(deviations[sample_idx, joint_idx])
    if worst > max_displacement_rad:
        raise TrajectoryError(
            f'Joint {ARM_JOINT_NAMES[joint_idx]} moves {worst:.4f}rad '
            f'(limit {max_displacement_rad:.4f}rad) at t={times[sample_idx]:.3f}s')
    if any(np.max(np.abs((b-a)/(t1-t0))) > max_speed_rad_s
           for (t0, a), (t1, b) in zip(zip(times, samples), zip(times[1:], samples[1:]))):
        raise TrajectoryError(f'Initial physical trial speed limited to {max_speed_rad_s}rad/s per joint')
    return trajectory


class HeadlessMoveItAction(Node):
    def __init__(self):
        super().__init__('g1_xr_headless_moveit_action')
        self.lock = Lock()
        self.busy = False
        # Configurable with a conservative default. Never disable the physical guards.
        self.declare_parameter('max_displacement_rad', 0.12)
        self.declare_parameter('max_speed_rad_s', 0.20)
        self.max_displacement_rad = float(self.get_parameter('max_displacement_rad').value)
        self.max_speed_rad_s = float(self.get_parameter('max_speed_rad_s').value)
        if not (0 < self.max_displacement_rad <= 0.30 and 0 < self.max_speed_rad_s <= 0.30):
            raise ValueError('Physical limits must remain within 0..0.30 rad and 0..0.30 rad/s')
        self.get_logger().warn(
            f'Trial limits: displacement={self.max_displacement_rad:.3f}rad, '
            f'speed={self.max_speed_rad_s:.3f}rad/s; physical watchdogs remain enabled')
        self.group = ReentrantCallbackGroup()
        self.action = ActionServer(
            self, FollowJointTrajectory, ACTION,
            execute_callback=self.execute, goal_callback=self.accept,
            cancel_callback=self.cancel, callback_group=self.group)
        self.get_logger().warn('PHYSICAL G1 endpoint: original XR motion-mode motor controller. '
                               'Only use approved collision-free tiny arm motions.')

    def accept(self, request_msg):
        try:
            status, q, dq = state()
            if not status.get('weight_ramp_complete'):
                raise RuntimeError('Physical XR motion-mode weight ramp incomplete')
            if np.max(np.abs(dq)) > 0.12:
                raise RuntimeError('Arms are moving; refuse trajectory ownership')
            prepare_trajectory(request_msg.trajectory, q, self.max_displacement_rad, self.max_speed_rad_s)
            if status.get('source') == 'trajectory':
                raise RuntimeError('XR is already executing a trajectory')
        except Exception as exc:
            self.get_logger().error(f'Physical goal rejected: {exc}')
            return GoalResponse.REJECT
        with self.lock:
            if self.busy:
                return GoalResponse.REJECT
            self.busy = True
        return GoalResponse.ACCEPT

    def cancel(self, goal):
        return CancelResponse.ACCEPT

    def execute(self, goal):
        result = FollowJointTrajectory.Result()
        seq = 0
        try:
            status, baseline, _ = state()
            traj = prepare_trajectory(goal.request.trajectory, baseline, self.max_displacement_rad, self.max_speed_rad_s)
            start = time.monotonic()
            last_log = 0.0
            self.get_logger().warn(f'Trajectory accepted: duration={traj.duration:.3f}s, '
                                   f'max_goal_delta={np.max(np.abs(traj.sample(traj.duration)-baseline)):.4f}rad')
            while rclpy.ok():
                now = time.monotonic()
                desired = traj.sample(min(now-start, traj.duration))
                if goal.is_cancel_requested:
                    _, actual, _ = state()
                    send(actual, 'hold', seq)
                    goal.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = 'Physical action cancelled; XR requested HOLD of measured position'
                    return result
                send(desired, 'trajectory', seq)
                seq += 1
                status_for_log, actual, velocity = state()
                error = np.max(np.abs(actual-desired))
                if now - last_log >= 0.50:
                    last_log = now
                    joint_idx = int(np.argmax(np.abs(actual-desired)))
                    self.get_logger().warn(
                        f'TRACK t={now-start:.2f}s source={status_for_log.get("source", "?")} '
                        f'joint={ARM_JOINT_NAMES[joint_idx]} '
                        f'desired={desired[joint_idx]:+.4f} actual={actual[joint_idx]:+.4f} '
                        f'error={error:.4f}rad; XR selected_q_delta='
                        f'{status_for_log.get("selected_target_delta_rad", "unknown")}')
                if error > 0.15:
                    raise RuntimeError(f'Physical arm tracking exceeded 0.15rad ({error:.4f}rad)')
                feedback = FollowJointTrajectory.Feedback()
                feedback.header.stamp = self.get_clock().now().to_msg()
                feedback.joint_names = list(ARM_JOINT_NAMES)
                feedback.desired = JointTrajectoryPoint()
                feedback.desired.positions = desired.tolist()
                feedback.actual = JointTrajectoryPoint()
                feedback.actual.positions = actual.tolist()
                feedback.actual.velocities = velocity.tolist()
                feedback.error = JointTrajectoryPoint()
                feedback.error.positions = (desired-actual).tolist()
                goal.publish_feedback(feedback)
                if now-start >= traj.duration:
                    break
                time.sleep(0.02)
            # Give the physical arms up to 2 seconds to settle at the goal.
            settling_at = time.monotonic()
            while time.monotonic()-settling_at < 2.0:
                send(traj.sample(traj.duration), 'trajectory', seq)
                seq += 1
                _, actual, dq = state()
                if (np.max(np.abs(actual-traj.sample(traj.duration))) <= 0.01
                        and np.max(np.abs(dq)) <= 0.12):
                    send(actual, 'hold', seq)
                    goal.succeed()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = 'Physical G1 goal confirmed from measured LowState; XR HOLD requested'
                    return result
                time.sleep(0.02)
            raise RuntimeError('Goal not achieved within 0.05rad / 0.12rad/s settling tolerance')
        except Exception as exc:
            self.get_logger().error(f'Physical goal aborted: {exc}')
            try:
                _, actual, _ = state()
                send(actual, 'hold', seq)
            except Exception as hold_exc:
                self.get_logger().error(f'HOLD request failed: {hold_exc}; independent operator stop REQUIRED')
            if goal.is_active:
                goal.abort()
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            result.error_string = str(exc)
            return result
        finally:
            with self.lock:
                self.busy = False


def main():
    rclpy.init()
    node = HeadlessMoveItAction()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.action.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
