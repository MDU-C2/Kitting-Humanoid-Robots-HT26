"""ROS 2 -> XR OFFLINE IPC integration trial.

Goal sampling remains on ROS side; each sample is delivered to a separate
Micromamba XR process, which computes ORIGINAL XR RNEA and acknowledges.
No DDS/SDK/motor control, no measured positions, no MoveIt production endpoint.
"""
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

from xr_arm_trajectory import ARM_JOINT_NAMES, ArmCommandRouter, Trajectory, TrajectoryError, Waypoint
from xr_ipc_protocol import IpcError, ping, send_sample

ACTION_NAME = '/g1_xr_bridge/offline_ipc/follow_joint_trajectory'


def convert_goal(msg, baseline=None):
    """Validate 14-arm goal, or complete one 7-arm goal from fresh XR state.

    A single-arm goal never resets or overwrites the other arm with zeros.
    Baseline is sourced from the XR receiver, NOT from arbitrary user input.
    """
    if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0:
        raise TrajectoryError('Offline test only accepts zero trajectory header timestamp')
    names = tuple(msg.joint_names)
    is_single_arm = (len(names) == 7 and len(set(names)) == 7 and
                     (set(names) == set(ARM_JOINT_NAMES[:7]) or
                      set(names) == set(ARM_JOINT_NAMES[7:])))
    if is_single_arm:
        if baseline is None:
            raise TrajectoryError('Single-arm goal requires fresh XR measured baseline')
        base_q = np.asarray(baseline, dtype=float)
        if base_q.shape != (14,) or not np.isfinite(base_q).all():
            raise TrajectoryError('Invalid measured XR baseline for single-arm goal')
    points = []
    for p in msg.points:
        seconds = float(p.time_from_start.sec) + p.time_from_start.nanosec * 1e-9
        positions = list(p.positions)
        velocities = list(p.velocities) if p.velocities else None
        if is_single_arm:
            if len(positions) != 7 or (velocities is not None and len(velocities) != 7):
                raise TrajectoryError('Single-arm points require 7 positions/velocities')
            expanded_q = base_q.copy()
            expanded_v = np.zeros(14) if velocities is not None else None
            for i, joint in enumerate(names):
                idx = ARM_JOINT_NAMES.index(joint)
                expanded_q[idx] = positions[i]
                if expanded_v is not None:
                    expanded_v[idx] = velocities[i]
            positions, velocities = expanded_q, expanded_v
        points.append(Waypoint(seconds, positions, velocities))
    traj = Trajectory(ARM_JOINT_NAMES if is_single_arm else names, points)
    if len(points) < 2 or points[0].time_from_start != 0.0 or traj.duration > 60.0:
        raise TrajectoryError('Offline test requires 2+ points, t0=0, duration <= 60 seconds')
    return traj


class OfflineIpcServer(Node):
    def __init__(self):
        super().__init__('g1_xr_bridge_offline_ipc')
        self._gate = Lock()
        self._busy = False
        self._group = ReentrantCallbackGroup()
        self._action = ActionServer(
            self, FollowJointTrajectory, ACTION_NAME,
            execute_callback=self.execute, goal_callback=self.accept,
            cancel_callback=self.cancel, callback_group=self._group)
        self.get_logger().warn(
            'OFFLINE IPC ONLY. No measured state or motor commands. XR process must run separately. '
            f'Action: {ACTION_NAME}')

    def get_baseline(self):
        return None  # plain offline IPC cannot assume measured real/sim state

    def accept(self, request):
        try:
            convert_goal(request.trajectory, self.get_baseline())
            ping()  # Refuse to accept if XR process is missing/unresponsive.
        except (IpcError, TrajectoryError, ValueError, TypeError) as exc:
            self.get_logger().warn(f'Offline IPC goal rejected: {exc}')
            return GoalResponse.REJECT
        with self._gate:
            if self._busy:
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        result = FollowJointTrajectory.Result()
        try:
            traj = convert_goal(goal_handle.request.trajectory, self.get_baseline())
            # OFFLINE ONLY: synthetic first point, NOT a measured G1 position.
            synthetic_q = traj.points[0][1]
            router = ArmCommandRouter(lambda q: np.zeros(14))  # only for arbitration, NEVER applied as torque
            started = time.monotonic()
            router.start_trajectory(traj, synthetic_q, started)
            next_log = 0.0
            seq = 0
            while rclpy.ok():
                now = time.monotonic()
                command = router.tick(now)
                if goal_handle.is_cancel_requested:
                    # OFFLINE: sampled q is NOT measured; no physical holding is claimed.
                    router.cancel_to_hold(command.q)
                    command = router.tick(time.monotonic())
                    send_sample(command.q, command.source.value, seq)
                    goal_handle.canceled()
                    result.error_string = 'OFFLINE IPC goal cancelled; no robot was controlled'
                    return result
                ack = send_sample(command.q, command.source.value, seq)
                seq += 1
                feedback = FollowJointTrajectory.Feedback()
                feedback.header.stamp = self.get_clock().now().to_msg()
                feedback.joint_names = list(ARM_JOINT_NAMES)
                feedback.desired = JointTrajectoryPoint()
                feedback.desired.positions = command.q.tolist()
                # No actual/error: offline XR receiver has no physical G1 feedback.
                goal_handle.publish_feedback(feedback)
                elapsed = now - started
                if elapsed >= next_log:
                    self.get_logger().info(
                        f'OFFLINE IPC t={elapsed:.2f}s, source={command.source.value}, '
                        f'left_elbow={command.q[3]:+.4f} rad, XR tau={ack["tau_ff"][3]:+.4f} Nm')
                    next_log += 0.5
                if elapsed >= traj.duration:
                    goal_handle.succeed()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = 'OFFLINE IPC complete; XR RNEA acknowledged; NO hardware executed'
                    return result
                time.sleep(0.02)
            raise RuntimeError('ROS shutdown')
        except Exception as exc:
            self.get_logger().error(f'OFFLINE IPC abort: {exc}')
            if goal_handle.is_active:
                goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = f'OFFLINE IPC failure: {exc}'
            return result
        finally:
            with self._gate:
                self._busy = False


def main():
    rclpy.init()
    node = OfflineIpcServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node._action.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
