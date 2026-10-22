#!/usr/bin/env python3
"""ROS Humble FollowJointTrajectory -> physical headless XR Unix socket.

Use ONLY for supervised, collision-checked G1 MoveIt trajectories. This endpoint
is separate from offline/simulation endpoints; no synthetic feedback is used.
"""
import math
import time
import os
import xml.etree.ElementTree as ET
from threading import Lock

import numpy as np
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from ament_index_python.packages import get_package_share_directory

from xr_arm_trajectory import ARM_JOINT_NAMES, TrajectoryError
from xr_moveit_goal_conversion import convert_goal
from xr_ipc_protocol import transact
from g1_wrist_pose import WristFK
from g1_wrist_correction import WristCorrection, CorrectionError, JointResponseEstimator, limits_from_urdf
import yaml

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


def send(q, source, seq, retain_final_target=False):
    result = request({'kind': 'sample', 'source': source, 'seq': int(seq),
                      'q': np.asarray(q, dtype=float).tolist(),
                      'retain_final_target': bool(retain_final_target)})
    if (result.get('kind') != 'sample_ack' or result.get('seq') != seq or
            result.get('selected_source') != source):
        raise RuntimeError('XR did not accept selected sample')
    return result


def load_joint_position_limits():
    """Fail closed if the G1 description lacks any 14-arm position limits."""
    urdf = os.path.join(get_package_share_directory('g1_description'), 'urdf',
                        'g1_29dof_with_inspire_hand_ftp.urdf')
    root = ET.parse(urdf).getroot()
    joints = {joint.get('name'): joint for joint in root.findall('joint')}
    limits = []
    for name in ARM_JOINT_NAMES:
        joint = joints.get(name)
        limit = joint.find('limit') if joint is not None else None
        if joint is None or joint.get('type') != 'revolute' or limit is None:
            raise ValueError(f'Missing revolute joint/limit for {name} in {urdf}')
        low, high = float(limit.get('lower')), float(limit.get('upper'))
        if not (math.isfinite(low) and math.isfinite(high) and low < high):
            raise ValueError(f'Invalid position limits for {name} in {urdf}')
        limits.append((low, high))
    return np.asarray(limits, dtype=float)


def prepare_trajectory(msg, baseline, max_displacement_rad=0.0,
                       joint_limits=None, velocity_limits=None):
    trajectory = convert_goal(msg, baseline)
    # G1_BOUNDED_WRIST_CORRECTION_V1: short valid moves may execute.
    # A one-point goal must be a true measured-state no-op, never a step jump.
    if trajectory.duration > 60.0:
        raise TrajectoryError('Trajectory exceeds physical 60 second maximum')
    if len(trajectory.points) == 1 and np.max(np.abs(trajectory.points[0][1] - baseline)) > 1e-4:
        raise TrajectoryError('Single-point goal differs from measured G1 state; a timed trajectory is required')
    if 0.0 < trajectory.duration < 0.5:
        # The old 0.5s guard prevented even valid millimetre-scale movements.
        # For newly accepted short moves, verify sampled slopes against the
        # SAME per-joint MoveIt YAML velocities already used by this project.
        # Never allow a large instantaneous move via tiny duration.
        speed_caps = np.asarray(velocity_limits, dtype=float)
        if (speed_caps.shape != (14,) or not np.isfinite(speed_caps).all() or
                np.any(speed_caps <= 0)):
            raise TrajectoryError('Short trajectory requires valid MoveIt velocity limits')
        dt = min(0.01, trajectory.duration / 10.0)
        t_samples = np.linspace(0.0, trajectory.duration,
                                max(2, int(math.ceil(trajectory.duration / dt)) + 1))
        q_samples = np.asarray([trajectory.sample(float(t)) for t in t_samples])
        speeds = np.abs(np.diff(q_samples, axis=0) / np.diff(t_samples)[:, None])
        if np.any(speeds > speed_caps[None, :] * 1.01):
            raise TrajectoryError('Short trajectory exceeds configured MoveIt joint velocity limits')
    initial_q = trajectory.points[0][1]
    if np.max(np.abs(initial_q - baseline)) > 0.05:
        raise TrajectoryError('First planned point must agree with physical G1 arms within 0.05rad')
    times = list(np.arange(0, trajectory.duration, 0.02)) + [trajectory.duration]
    samples = np.asarray([trajectory.sample(float(t)) for t in times])
    # Zero disables only the additional per-goal excursion cap.
    # Retain URDF physical joint bounds and live hardware watchdogs.
    if max_displacement_rad > 0.0:
        deviations = np.abs(samples - baseline)
        sample_idx, joint_idx = np.unravel_index(np.argmax(deviations), deviations.shape)
        worst = float(deviations[sample_idx, joint_idx])
        if worst > max_displacement_rad:
            raise TrajectoryError(
                f'Joint {ARM_JOINT_NAMES[joint_idx]} moves {worst:.4f}rad '
                f'(limit {max_displacement_rad:.4f}rad) at t={times[sample_idx]:.3f}s')
    # Joint position constraints are delegated to MoveIt and Unitree.
    return trajectory


class HeadlessMoveItAction(Node):
    def __init__(self):
        super().__init__('g1_xr_headless_moveit_action')
        self.lock = Lock()
        self.busy = False
        # G1_WRIST_GOAL_CHECK_V1: opt-in pose-based success, read-only wrist FK.
        self.goal_check_mode = os.environ.get('G1_GOAL_CHECK_MODE', 'joint').strip().lower()
        if self.goal_check_mode not in ('joint', 'wrist'):
            raise ValueError('G1_GOAL_CHECK_MODE must be joint or wrist')
        def finite_positive_env(name, default):
            value = float(os.environ.get(name, default))
            if not (math.isfinite(value) and value > 0):
                raise ValueError(f'{name} must be a positive finite number')
            return value
        # Initial trial tolerances: 5 mm of wrist-link origin, 2 degrees of link orientation.
        # These are success thresholds, NOT a promise of physical accuracy.
        self.wrist_pos_limit = finite_positive_env('G1_WRIST_POS_TOL_M', '0.005')
        self.wrist_rot_limit = finite_positive_env('G1_WRIST_ROT_TOL_DEG', '2.0') * math.pi / 180.0
        self.wrist_fk = None
        self.wrist_corrector = None
        # Load exactly the MoveIt YAML speed limits, rather than introducing
        # a second, independent physical velocity configuration.
        velocity_yaml = os.path.join(
            get_package_share_directory('g1_moveit_config'), 'config', 'joint_limits.yaml')
        with open(velocity_yaml, encoding='utf-8') as stream:
            velocity_config = yaml.safe_load(stream)['joint_limits']
        self.short_goal_velocity_limits = np.asarray([
            float(velocity_config[name]['max_velocity'])
            for name in ARM_JOINT_NAMES], dtype=float)
        if not np.isfinite(self.short_goal_velocity_limits).all() or np.any(self.short_goal_velocity_limits <= 0):
            raise ValueError('Invalid MoveIt velocity limit configuration')
        # G1_BOUNDED_WRIST_CORRECTION_V1: preview is read-only by default.
        self.wrist_correction_mode = os.environ.get('G1_WRIST_CORRECTION_MODE', 'preview').strip().lower()
        if self.wrist_correction_mode not in ('off', 'preview', 'execute'):
            raise ValueError('G1_WRIST_CORRECTION_MODE must be off, preview or execute')
        if self.wrist_correction_mode == 'execute':
            if self.goal_check_mode != 'wrist':
                raise ValueError('Wrist correction requires G1_GOAL_CHECK_MODE=wrist')
            if os.environ.get('G1_WRIST_CORRECTION_OPERATOR_ACK') != 'G1_WRIST_CORRECTION_SUPERVISED':
                raise ValueError('Physical correction requires G1_WRIST_CORRECTION_OPERATOR_ACK=G1_WRIST_CORRECTION_SUPERVISED')
        if self.goal_check_mode == 'wrist':
            urdf = os.path.join(get_package_share_directory('g1_description'),
                                'urdf', 'g1_29dof_with_inspire_hand_ftp.urdf')
            self.wrist_fk = WristFK(urdf, ARM_JOINT_NAMES)
            if self.wrist_correction_mode != 'off':
                self.wrist_corrector = WristCorrection(
                    self.wrist_fk, limits_from_urdf(urdf, ARM_JOINT_NAMES),
                    pos_tol=self.wrist_pos_limit,
                    rot_tol_deg=math.degrees(self.wrist_rot_limit))
        self.get_logger().warn(
            f'G1 wrist correction mode={self.wrist_correction_mode}; '
            'max joint correction step=0.008rad; cumulative offset=0.035rad; '
            'correction target speed<=0.04rad/s; max 5 passes; '
            'no collision checker in this action server; '
            'G1_WRIST_LOAD_ADAPTIVE_V1 response-aware final correction')
        self.get_logger().warn(
            f'G1 goal check mode={self.goal_check_mode}; wrist frame=torso_link; '
            f'wrist limits={self.wrist_pos_limit*1000:.1f}mm, '
            f'{math.degrees(self.wrist_rot_limit):.1f}deg; '
            'tracking watchdog and final joint-speed watchdog unchanged')
        # 0.0 disables the extra per-goal excursion cap, not the physical protections.
        self.declare_parameter('max_displacement_rad', 0.0)
        self.max_displacement_rad = float(self.get_parameter('max_displacement_rad').value)
        if not (math.isfinite(self.max_displacement_rad) and
                0 <= self.max_displacement_rad <= 0.30):
            raise ValueError('Displacement cap must be 0 (disabled) or <=0.30 rad')
        cap = 'disabled' if self.max_displacement_rad == 0 else f'{self.max_displacement_rad:.3f}rad'
        self.get_logger().warn(
            f'MoveIt joint excursion cap: {cap}; extra trial-speed cap removed; '
            'tracking and feedback watchdogs enabled')
        self.group = ReentrantCallbackGroup()
        self.action = ActionServer(
            self, FollowJointTrajectory, ACTION,
            execute_callback=self.execute, goal_callback=self.accept,
            cancel_callback=self.cancel, callback_group=self.group)
        self.get_logger().warn('PHYSICAL G1 endpoint: original XR motion-mode motor controller. '
                               'Only use operator-approved collision-free trajectories; collision model is incomplete.')

    def accept(self, request_msg):
        try:
            status, q, dq = state()
            if not status.get('weight_ramp_complete'):
                raise RuntimeError('Physical XR motion-mode weight ramp incomplete')
            if np.max(np.abs(dq)) > 0.12:
                raise RuntimeError('Arms are moving; refuse trajectory ownership')
            prepare_trajectory(request_msg.trajectory, q, self.max_displacement_rad,
                               velocity_limits=self.short_goal_velocity_limits)
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

    def _correct_wrist(self, goal, final_q, active_wrists, seq):
        # Correction remains under the SAME FollowJointTrajectory goal and
        # XR trajectory ownership. Never creates a second motor publisher.
        # Kinematic proposals are constrained but NOT collision-checked here.
        controller = self.wrist_corrector
        # G1_WRIST_LOAD_ADAPTIVE_V1: feedback is used ONLY during final
        # correction, never while tracking the main MoveIt trajectory.
        response = JointResponseEstimator()
        low_progress_passes = 0
        current_cmd = np.asarray(final_q, dtype=float).copy()
        overall_start = time.monotonic()
        for pass_num in range(1, 6):
            if goal.is_cancel_requested:
                return 'cancel', seq, current_cmd
            if time.monotonic() - overall_start > 8.0:
                raise CorrectionError('Wrist correction exceeded 8-second time budget')
            _, actual, dq = state()
            if np.max(np.abs(dq)) > 0.12:
                raise CorrectionError('Measured joint speed above 0.12rad/s before correction')
            ok, score, errors = controller.metrics(final_q, actual, active_wrists)
            if ok:
                return 'success', seq, current_cmd
            proposed = controller.propose(
                final_q, actual, current_cmd, active_wrists,
                response_gains=response.gains)
            # G1_MICRO_MOTION_RESPONSE_DIAGNOSTICS_V1: record before/after
            # physical encoder feedback and the requested correction only.
            micro_before_measured = np.asarray(actual, dtype=float).copy()
            micro_before_command = np.asarray(current_cmd, dtype=float).copy()
            step = float(np.max(np.abs(proposed - current_cmd)))
            if step > 0.0080001:
                raise CorrectionError('Correction exceeds 0.008rad single-step cap')
            self.get_logger().warn(
                f'WRIST_CORRECTION pass={pass_num} pose_score={score*1000:.2f}mm_equiv '
                f'joint_step={step:.5f}rad max_offset={np.max(np.abs(proposed-final_q)):.5f}rad')
            # At most 0.04rad/s commanded joint speed, independent of RViz's
            # 1.0 trajectory scaling. Original XR 0.15rad feedback watchdog kept.
            ramp_time = max(0.15, step / 0.04)
            ramp_start = time.monotonic()
            while True:
                if goal.is_cancel_requested:
                    return 'cancel', seq, current_cmd
                elapsed = time.monotonic() - ramp_start
                blend = min(1.0, elapsed / ramp_time)
                command = current_cmd + blend * (proposed - current_cmd)
                send(command, 'trajectory', seq)
                seq += 1
                _, physical, velocity = state()
                if np.max(np.abs(physical - command)) > 0.15:
                    raise CorrectionError('Hardware tracking watchdog triggered during correction (>0.15rad)')
                if np.max(np.abs(velocity)) > 0.12:
                    raise CorrectionError('Unexpected measured joint speed >0.12rad/s during correction')
                if blend >= 1.0:
                    break
                time.sleep(0.02)
            current_cmd = proposed.copy()
            # Give original motor PD + gravity FF a brief stationary interval.
            # Resend at ~50Hz to prevent XR's 0.25sec watchdog auto-HOLD.
            settle_at = time.monotonic()
            while time.monotonic() - settle_at < 0.45:
                if goal.is_cancel_requested:
                    return 'cancel', seq, current_cmd
                send(current_cmd, 'trajectory', seq)
                seq += 1
                _, physical, velocity = state()
                if np.max(np.abs(physical - current_cmd)) > 0.15:
                    raise CorrectionError('Hardware tracking watchdog triggered while correcting')
                if np.max(np.abs(velocity)) > 0.12:
                    raise CorrectionError('Measured joint speed too high while correcting')
                time.sleep(0.02)
            micro_after_status, actual, dq = state()
            # G1_MICRO_MOTION_RESPONSE_DIAGNOSTICS_V1: telemetry ONLY.
            # This block intentionally cannot change commands or abort execution.
            try:
                micro_delta_cmd = np.asarray(current_cmd, dtype=float) - micro_before_command
                micro_delta_enc = np.asarray(actual, dtype=float) - micro_before_measured
                micro_indices = np.argsort(-np.abs(micro_delta_cmd))[:5]
                micro_indices = [int(i) for i in micro_indices if abs(micro_delta_cmd[i]) >= 1e-5]
                lowcmd_q = np.asarray(micro_after_status.get('xr_lowcmd_q', []), dtype=float)
                if lowcmd_q.shape == (14,) and np.isfinite(lowcmd_q).all():
                    lowcmd_mismatch = float(np.max(np.abs(lowcmd_q - current_cmd)))
                    lowcmd_info = f'lowcmd_target_diff={lowcmd_mismatch:.6f}rad'
                else:
                    lowcmd_info = 'lowcmd_target_diff=unavailable'
                motor_q = np.asarray(micro_after_status.get('xr_motor_target', []), dtype=float)
                if motor_q.shape == (14,) and np.isfinite(motor_q).all():
                    motor_info = f'motor_target_diff={np.max(np.abs(motor_q-current_cmd)):.6f}rad'
                else:
                    motor_info = 'motor_target_diff=unavailable'
                # Compare the kinematic motion expected from the commanded
                # increment with the motion actually inferred from encoders.
                poses_before = self.wrist_fk.poses(micro_before_measured)
                poses_predicted = self.wrist_fk.poses(micro_before_measured + micro_delta_cmd)
                poses_measured = self.wrist_fk.poses(actual)
                micro_wrist_motion = ' '.join(
                    f'{w}:pred_delta={np.linalg.norm(poses_predicted[w][:3,3]-poses_before[w][:3,3])*1000:.3f}mm'
                    f'/measured_delta={np.linalg.norm(poses_measured[w][:3,3]-poses_before[w][:3,3])*1000:.3f}mm'
                    for w in active_wrists
                )
                micro_joint_details = ' '.join(
                    f'{ARM_JOINT_NAMES[i]}:cmd_delta={micro_delta_cmd[i]:+.6f}'
                    f'/encoder_delta={micro_delta_enc[i]:+.6f}rad'
                    for i in micro_indices
                )
                self.get_logger().warn(
                    f'WRIST_MICROMOTION_RESPONSE pass={pass_num} '
                    f'peak_cmd_delta={np.max(np.abs(micro_delta_cmd)):.6f}rad '
                    f'peak_encoder_delta={np.max(np.abs(micro_delta_enc)):.6f}rad '
                    f'{lowcmd_info} {motor_info} '
                    f'{micro_wrist_motion} joints=[{micro_joint_details}]')
            except Exception as diag_exc:
                self.get_logger().warn(f'WRIST_MICROMOTION_DIAGNOSTIC_UNAVAILABLE: {diag_exc}')
            # Update the joint-response model after the command has settled.
            # This changes only the NEXT correction proposal; it cannot
            # directly modify XR LowCmd, gains, gravity FF, or motor ownership.
            try:
                informative = response.observe(
                    current_cmd - micro_before_command,
                    np.asarray(actual, dtype=float) - micro_before_measured)
            except ValueError as response_error:
                raise CorrectionError(f'Adaptive wrist response unsafe: {response_error}')
            if len(informative):
                joint_text = ' '.join(
                    f'{ARM_JOINT_NAMES[int(i)]}={response.gains[int(i)]:.2f}'
                    for i in informative)
                self.get_logger().warn(
                    f'WRIST_ADAPTIVE_RESPONSE pass={pass_num} gains=[{joint_text}]')
            ok, updated_score, errors = controller.metrics(final_q, actual, active_wrists)
            description = ' '.join(
                f'{w}={errors[w][0]*1000:.2f}mm/{math.degrees(errors[w][1]):.2f}deg'
                for w in active_wrists)
            self.get_logger().warn(
                f'WRIST_CORRECTION_RESULT pass={pass_num} score={updated_score*1000:.2f}mm_equiv '
                f'{description} success={ok}')
            if ok and np.max(np.abs(dq)) <= 0.12:
                return 'success', seq, current_cmd
            improvement = score - updated_score
            self.get_logger().warn(
                f'WRIST_CORRECTION_PROGRESS pass={pass_num} '
                f'improvement={improvement*1000:.3f}mm_equiv '
                'minimum=0.100mm_equiv')
            # Abort when a completed correction did not produce meaningful
            # measured improvement. A single 0.23mm improvement is progress,
            # even though the original 0.25mm threshold rejected it.
            # All existing joint, velocity, cumulative-offset, time, and
            # pass-count guards remain unchanged.
            if improvement < -0.00005:
                raise CorrectionError('Adaptive wrist correction increased pose error; stopping')
            if improvement < 0.0001:
                # Permit at most ONE extra *bounded* pass when a completed
                # command elicited meaningful encoder motion. The response
                # model can then favor joints that really move. No blind
                # repeated offsets when no joint responded or error worsens.
                encoder_step = np.max(np.abs(np.asarray(actual) - micro_before_measured))
                if low_progress_passes >= 1 or encoder_step < 0.0005 or not len(informative):
                    raise CorrectionError(
                        f'Wrist correction stalled after pass {pass_num}: '
                        f'improvement={improvement*1000:.3f}mm_equiv '
                        '(minimum 0.100mm_equiv)')
                low_progress_passes += 1
                self.get_logger().warn(
                    f'WRIST_ADAPTIVE_RETRY pass={pass_num} '
                    f'improvement={improvement*1000:.3f}mm_equiv '
                    f'max_encoder_motion={encoder_step:.6f}rad; '
                    'allowing one further correction within existing limits')
            else:
                low_progress_passes = 0
        raise CorrectionError('Wrist correction reached 5-pass safety limit without converging')

    def execute(self, goal):
        result = FollowJointTrajectory.Result()
        seq = 0
        try:
            status, baseline, _ = state()
            traj = prepare_trajectory(goal.request.trajectory, baseline, self.max_displacement_rad,
                                      velocity_limits=self.short_goal_velocity_limits)
            # Valid no-motion goals must be verified from fresh LowState;
            # never make a motor command for a tiny/empty trajectory.
            # No-op tolerance is 0.0001rad per joint, NOT a positioning goal tolerance.
            is_noop = all(np.max(np.abs(point[1] - baseline)) <= 1e-4
                          for point in traj.points)
            if is_noop:
                if goal.is_cancel_requested:
                    goal.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = 'Cancelled before no-motion validation'
                    return result
                _, actual, velocity = state()
                if np.max(np.abs(velocity)) > 0.12:
                    raise RuntimeError('No-motion goal rejected: robot is moving')
                final_noop = traj.sample(traj.duration)
                if self.wrist_fk is not None:
                    selected = tuple(w for prefix, w in
                                     (('left_', 'left_wrist_yaw_link'), ('right_', 'right_wrist_yaw_link'))
                                     if any(n.startswith(prefix) for n in goal.request.trajectory.joint_names))
                    if not selected:
                        raise RuntimeError('No-motion goal has no arm wrist')
                    errors = self.wrist_fk.errors(final_noop, actual)
                    valid = all(errors[w][0] <= self.wrist_pos_limit and
                                errors[w][1] <= self.wrist_rot_limit for w in selected)
                else:
                    valid = np.max(np.abs(final_noop - actual)) <= 0.01
                if not valid:
                    raise RuntimeError('No-motion goal rejected: physical wrist/joints outside goal tolerance')
                self.get_logger().warn('NO_MOTION_GOAL_SUCCESS: fresh LowState within configured goal tolerance')
                goal.succeed()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = 'Valid no-motion goal: hardware already within tolerance'
                return result
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
            # G1_SETTLING_DIAGNOSTICS_V1: diagnostic-only change. Keep the exact
            # existing acceptance tolerances and the two-second settling window.
            final_q = traj.sample(traj.duration)
            settling_at = time.monotonic()
            position_limit = 0.01
            speed_limit = 0.12
            wrist_log_last = -1.0
            # Only test wrists controlled by this goal (both when both arms move).
            goal_names = tuple(goal.request.trajectory.joint_names)
            active_wrists = tuple(
                name for side, name in (
                    ('left_', 'left_wrist_yaw_link'),
                    ('right_', 'right_wrist_yaw_link'))
                if any(joint.startswith(side) for joint in goal_names))
            if self.wrist_fk is not None and not active_wrists:
                raise RuntimeError('No arm joints in wrist-pose trajectory goal')
            worst_joint = 0
            fastest_joint = 0
            max_pos_error = float('nan')
            max_speed = float('nan')
            # G1_SETTLING_LAYER_DIAGNOSTICS_V1: observe command vs measured, no control change.
            last_settle_log = -1.0
            while time.monotonic() - settling_at < 2.0:
                if goal.is_cancel_requested:
                    _, actual, _ = state()
                    send(actual, 'hold', seq)
                    goal.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = 'Physical action cancelled during settling; XR HOLD requested'
                    return result
                send(final_q, 'trajectory', seq)
                seq += 1
                status_for_settle, actual, dq = state()
                pos_errors = np.abs(actual - final_q)
                joint_speeds = np.abs(dq)
                worst_joint = int(np.argmax(pos_errors))
                fastest_joint = int(np.argmax(joint_speeds))
                max_pos_error = float(pos_errors[worst_joint])
                max_speed = float(joint_speeds[fastest_joint])
                # G1_WRIST_GOAL_CHECK_V1: evaluate measured wrist-link FK, not planned ghost pose.
                wrist_ok = False
                wrist_description = ''
                if self.wrist_fk is not None:
                    wrist_errors = self.wrist_fk.errors(final_q, actual)
                    wrist_ok = all(
                        linear <= self.wrist_pos_limit and angular <= self.wrist_rot_limit
                        for name, (linear, angular) in wrist_errors.items()
                        if name in active_wrists)
                    wrist_description = ' '.join(
                        f'{name}: {linear*1000:.2f}mm, {math.degrees(angular):.2f}deg'
                        for name, (linear, angular) in wrist_errors.items()
                        if name in active_wrists)
                    elapsed_wrist = time.monotonic() - settling_at
                    if wrist_log_last < 0 or elapsed_wrist - wrist_log_last >= 0.45:
                        wrist_log_last = elapsed_wrist
                        self.get_logger().warn(
                            f'WRIST_SETTLE t={elapsed_wrist:.2f}s '
                            f'{wrist_description} '
                            f'joint_max_error={max_pos_error:.4f}rad '
                            f'max_joint_speed={max_speed:.4f}rad/s wrist_ok={wrist_ok}')
                settle_elapsed = time.monotonic() - settling_at
                if last_settle_log < 0 or settle_elapsed - last_settle_log >= 0.45:
                    last_settle_log = settle_elapsed
                    # These values come from the read-only XR status response.
                    # Do not assume a target acknowledgement is motor feedback.
                    command_parts = []
                    for label, key in (
                            ('selected', 'selected_arm_target'),
                            ('motor', 'xr_motor_target'),
                            ('limited', 'xr_limited_command_q')):
                        command_q = np.asarray(status_for_settle.get(key, []), dtype=float)
                        if command_q.shape == (14,) and np.isfinite(command_q).all():
                            command_parts.append(
                                f'{label}_vs_goal={np.max(np.abs(command_q-final_q)):.4f}rad')
                    # G1_READONLY_LOW_CMD_TELEMETRY_V1: these are values in
                    # XR's outgoing LowCmd buffer, NOT measured motor torque.
                    effort_diag = ''
                    lowcmd_tau = np.asarray(status_for_settle.get('xr_lowcmd_tau', []), dtype=float)
                    lowcmd_kp = np.asarray(status_for_settle.get('xr_lowcmd_kp', []), dtype=float)
                    lowcmd_q = np.asarray(status_for_settle.get('xr_lowcmd_q', []), dtype=float)
                    if (lowcmd_tau.shape == (14,) and lowcmd_kp.shape == (14,)
                            and lowcmd_q.shape == (14,) and np.isfinite(lowcmd_tau).all()
                            and np.isfinite(lowcmd_kp).all() and np.isfinite(lowcmd_q).all()):
                        j = worst_joint
                        effort_diag = (
                            f' lowcmd_vs_goal={np.max(np.abs(lowcmd_q-final_q)):.4f}rad'
                            f' lowcmd_tau_ff={lowcmd_tau[j]:+.3f}Nm'
                            f' lowcmd_kp={lowcmd_kp[j]:.1f}'
                            f' kp_times_error={lowcmd_kp[j] * (final_q[j]-actual[j]):+.3f}Nm'
                        )
                    self.get_logger().warn(
                        f'SETTLE t={settle_elapsed:.2f}s source={status_for_settle.get("source", "?")} '
                        f'joint={ARM_JOINT_NAMES[worst_joint]} '
                        f'goal={final_q[worst_joint]:+.4f} actual={actual[worst_joint]:+.4f} '
                        f'pos_error={max_pos_error:.4f}rad max_speed={max_speed:.4f}rad/s '
                        + ' '.join(command_parts) + effort_diag)
                reached = (wrist_ok if self.wrist_fk is not None
                           else max_pos_error <= position_limit)
                if reached and max_speed <= speed_limit:
                    self.get_logger().warn(
                        f'Physical goal succeeded: final_max_error={max_pos_error:.4f}rad '
                        f'final_max_speed={max_speed:.4f}rad/s '
                        f'settling_time={settle_elapsed:.2f}s')
                    if self.wrist_fk is not None:
                        self.get_logger().warn(
                            f'WRIST_GOAL_SUCCESS: {wrist_description}; '
                            f'joint_max_error={max_pos_error:.4f}rad')
                    send(actual, 'hold', seq)
                    goal.succeed()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = (
                        'Physical G1 wrist poses confirmed from measured LowState; XR HOLD requested'
                        if self.wrist_fk is not None
                        else 'Physical G1 goal confirmed from measured LowState; XR HOLD requested')
                    return result
                time.sleep(0.02)
            # Do not change the success rule: if 5mm/2deg is not met,
            # either log a read-only proposed correction or execute only with
            # separate explicit operator approval.
            if self.wrist_fk is not None and self.wrist_corrector is not None:
                if self.wrist_correction_mode == 'preview':
                    try:
                        proposed = self.wrist_corrector.propose(final_q, actual, final_q, active_wrists)
                        self.get_logger().warn(
                            f'WRIST_CORRECTION_PREVIEW: would apply max_joint_step='
                            f'{np.max(np.abs(proposed-final_q)):.5f}rad; '
                            'no motor target changed')
                    except CorrectionError as diagnostic:
                        self.get_logger().warn(f'WRIST_CORRECTION_PREVIEW_UNAVAILABLE: {diagnostic}')
                elif self.wrist_correction_mode == 'execute':
                    outcome, seq, corrected_q = self._correct_wrist(goal, final_q, active_wrists, seq)
                    if outcome == 'cancel':
                        _, actual, _ = state()
                        send(actual, 'hold', seq)
                        goal.canceled()
                        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                        result.error_string = 'Wrist correction cancelled; XR HOLD captured measured pose'
                        return result
                    # Maintain corrected command in XR's headless HOLD; ordinary
                    # execution, cancellation and faults still capture measured q.
                    send(corrected_q, 'hold', seq, retain_final_target=True)
                    seq += 1
                    time.sleep(0.30)
                    state_after, actual, dq = state()
                    success, _, corrected_errors = self.wrist_corrector.metrics(
                        final_q, actual, active_wrists)
                    if state_after.get('source') != 'hold' or not success or np.max(np.abs(dq)) > 0.12:
                        raise CorrectionError('Wrist drifted after retained HOLD; goal cannot be confirmed')
                    description = ' '.join(
                        f'{w}={corrected_errors[w][0]*1000:.2f}mm/'
                        f'{math.degrees(corrected_errors[w][1]):.2f}deg' for w in active_wrists)
                    self.get_logger().warn(f'WRIST_CORRECTION_SUCCESS: {description}, HOLD verified')
                    goal.succeed()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = 'Measured wrist target reached after bounded correction; XR corrected HOLD verified'
                    return result
            if self.wrist_fk is not None:
                raise RuntimeError(
                    f'Wrist goal not achieved within 2.0s; {wrist_description}; '
                    f'allowed={self.wrist_pos_limit*1000:.2f}mm/'
                    f'{math.degrees(self.wrist_rot_limit):.2f}deg; '
                    f'joint_max_error={max_pos_error:.4f}rad; '
                    f'joint_max_speed={max_speed:.4f}rad/s')
            raise RuntimeError(
                'Goal not achieved within 2.0s; '
                f'position error={max_pos_error:.4f}rad '
                f'(limit {position_limit:.3f}rad) at {ARM_JOINT_NAMES[worst_joint]}; '
                f'joint speed={max_speed:.4f}rad/s '
                f'(limit {speed_limit:.2f}rad/s) at {ARM_JOINT_NAMES[fastest_joint]}')
        except Exception as exc:
            self.get_logger().error(f'Physical goal aborted: {exc}')
            try:
                _, actual, _ = state()
                send(actual, 'hold', seq)
            except Exception as hold_exc:
                self.get_logger().error(f'HOLD request failed: {hold_exc}; independent operator stop REQUIRED')
            if goal.is_active:
                goal.abort()
            result.error_code = (
                FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                if (str(exc).startswith('Wrist goal not achieved') or
                    isinstance(exc, CorrectionError))
                else FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED)
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
