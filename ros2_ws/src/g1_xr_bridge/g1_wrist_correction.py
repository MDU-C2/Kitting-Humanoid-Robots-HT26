"""Bounded torso-relative wrist correction proposals; NEVER publishes commands.

Pose goals are wrist_yaw_link origins and rotations from the G1 Inspire FTP URDF.
The integration must independently supervise hardware, motion and collisions.
"""
import math
import xml.etree.ElementTree as ET

import numpy as np

from g1_wrist_pose import WRIST_LINKS


class CorrectionError(ValueError):
    pass


def rotation_vector(R):
    """Return rotation vector (radians) from a finite 3x3 rotation matrix."""
    R = np.asarray(R, dtype=float)
    if R.shape != (3, 3) or not np.isfinite(R).all():
        raise CorrectionError('Invalid orientation matrix')
    c = float(np.clip((np.trace(R) - 1.0) * 0.5, -1.0, 1.0))
    theta = math.acos(c)
    v = np.array([R[2, 1]-R[1, 2], R[0, 2]-R[2, 0], R[1, 0]-R[0, 1]])
    if theta < 1e-5:
        return 0.5 * v
    if math.pi - theta < 1e-4:
        raise CorrectionError('Near-pi wrist attitude correction requires a new MoveIt plan')
    return (theta / (2.0 * math.sin(theta))) * v


def limits_from_urdf(path, names):
    root = ET.parse(path).getroot()
    joint_map = {j.get('name'): j for j in root.findall('joint')}
    bounds = []
    for name in names:
        joint = joint_map.get(name)
        limit = joint.find('limit') if joint is not None else None
        if joint is None or joint.get('type') != 'revolute' or limit is None:
            raise CorrectionError(f'Missing physical joint bounds for {name}')
        pair = (float(limit.get('lower')), float(limit.get('upper')))
        if not all(math.isfinite(x) for x in pair) or pair[0] >= pair[1]:
            raise CorrectionError(f'Invalid physical joint bounds for {name}')
        bounds.append(pair)
    return np.asarray(bounds, dtype=float)


# G1_WRIST_LOAD_ADAPTIVE_V1: per-goal response-aware final wrist correction
class JointResponseEstimator:
    """Conservative diagonal approximation dq_actual ~= S @ dq_command.

    Only use samples with meaningful command magnitude; use bounded EMA so
    a single noisy measurement cannot create an uncontrolled correction gain.
    """
    def __init__(self, count=14, min_cmd=0.001, min_gain=0.10, max_gain=1.25, alpha=0.75):
        self.gains = np.ones(count, dtype=float)
        self.samples = np.zeros(count, dtype=int)
        self.min_cmd = min_cmd
        self.min_gain = min_gain
        self.max_gain = max_gain
        self.alpha = alpha

    def observe(self, commanded_delta, measured_delta):
        requested = np.asarray(commanded_delta, dtype=float)
        observed = np.asarray(measured_delta, dtype=float)
        if requested.shape != self.gains.shape or observed.shape != self.gains.shape:
            raise ValueError('Invalid per-joint feedback shape')
        if not (np.isfinite(requested).all() and np.isfinite(observed).all()):
            raise ValueError('Non-finite joint-response feedback')
        informative = np.flatnonzero(np.abs(requested) >= self.min_cmd)
        for index in informative:
            ratio = float(observed[index] / requested[index])
            # Strong reversed motion under a deliberate command is not a
            # calibration sample: it is an unsafe/unmodelled response.
            if ratio < -0.20 and abs(observed[index]) > 0.0005:
                raise ValueError('Joint moved substantially opposite to the correction command')
            ratio = float(np.clip(max(0.0, ratio), self.min_gain, self.max_gain))
            self.gains[index] = (1 - self.alpha) * self.gains[index] + self.alpha * ratio
            self.samples[index] += 1
        return informative


class WristCorrection:
    """Damped-least-squares bounded incremental wrist-target correction.

    The increment is added to the *existing commanded joint target* rather
    than measured joints. That distinction is essential for steady-state PD
    errors: q_cmd <- q_cmd + J(measured)^# * measured_wrist_pose_error.
    The engine never publishes a command and never manages motor authority.
    """
    def __init__(self, fk, joint_limits, pos_tol=0.005, rot_tol_deg=2.0,
                 max_joint_step=0.008, max_total_offset=0.035,
                 max_cart_step=0.002, max_rot_step_deg=0.5,
                 joint_margin=0.015, damping=0.025, angular_weight=0.15):
        self.fk = fk
        self.limits = np.asarray(joint_limits, dtype=float)
        if self.limits.shape != (14, 2) or not np.isfinite(self.limits).all() or np.any(self.limits[:, 0] >= self.limits[:, 1]):
            raise CorrectionError('Expected valid 14-arm joint position bounds')
        self.pos_tol = float(pos_tol)
        self.rot_tol = math.radians(rot_tol_deg)
        self.max_joint_step = float(max_joint_step)
        self.max_total_offset = float(max_total_offset)
        self.max_cart_step = float(max_cart_step)
        self.max_rot_step = math.radians(max_rot_step_deg)
        self.joint_margin = float(joint_margin)
        self.damping = float(damping)
        self.angular_weight = float(angular_weight)
        if not all(math.isfinite(x) and x > 0 for x in (
                self.pos_tol, self.rot_tol, self.max_joint_step,
                self.max_total_offset, self.max_cart_step, self.max_rot_step,
                self.joint_margin, self.damping, self.angular_weight)):
            raise CorrectionError('Correction parameters must be positive and finite')

    def metrics(self, goal_q, measured_q, active_wrists):
        errors = self.fk.errors(goal_q, measured_q)
        selected = tuple(active_wrists)
        if not selected or len(set(selected)) != len(selected) or any(name not in WRIST_LINKS for name in selected):
            raise CorrectionError('Invalid active wrist selection')
        ok = all(errors[w][0] <= self.pos_tol and errors[w][1] <= self.rot_tol for w in selected)
        # mm-equivalent pose norm, balancing metres and radians without relying
        # on raw unscaled orientation values in a least-squares problem.
        score = max(math.hypot(errors[w][0], self.angular_weight * errors[w][1]) for w in selected)
        return ok, score, errors

    def propose(self, goal_q, measured_q, current_cmd_q, active_wrists,
                response_gains=None):
        q_goal = np.asarray(goal_q, dtype=float)
        q_meas = np.asarray(measured_q, dtype=float)
        q_cmd = np.asarray(current_cmd_q, dtype=float)
        if any(q.shape != (14,) or not np.isfinite(q).all() for q in (q_goal, q_meas, q_cmd)):
            raise CorrectionError('Expected finite 14-arm goal, measurement and command')
        # Adaptive response is only used for the final correction phase.
        # A 0.25 gain means a 0.004rad command produced ~0.001rad actual.
        if response_gains is None:
            response = np.ones(14, dtype=float)
        else:
            response = np.asarray(response_gains, dtype=float)
            if (response.shape != (14,) or not np.isfinite(response).all()
                    or np.any(response < 0.10) or np.any(response > 1.25)):
                raise CorrectionError('Unsafe or invalid measured joint-response gains')
        ok, _, _ = self.metrics(q_goal, q_meas, active_wrists)
        if ok:
            return q_cmd.copy()
        goal_poses = self.fk.poses(q_goal)
        actual_poses = self.fk.poses(q_meas)
        proposal = q_cmd.copy()
        for wrist in active_wrists:
            a, g = actual_poses[wrist], goal_poses[wrist]
            ep = g[:3, 3] - a[:3, 3]
            er = rotation_vector(g[:3, :3] @ a[:3, :3].T)
            if np.linalg.norm(ep) <= self.pos_tol and np.linalg.norm(er) <= self.rot_tol:
                continue
            # Limit *task-space* correction as well as the resulting joint step.
            pnorm, rnorm = np.linalg.norm(ep), np.linalg.norm(er)
            if pnorm > self.max_cart_step:
                ep *= self.max_cart_step / pnorm
            if rnorm > self.max_rot_step:
                er *= self.max_rot_step / rnorm
            indices = range(7) if wrist == 'left_wrist_yaw_link' else range(7, 14)
            J = np.zeros((6, 7), dtype=float)
            epsilon = 1e-5
            for column, idx in enumerate(indices):
                q_offset = q_meas.copy()
                q_offset[idx] += epsilon
                pose_delta = self.fk.poses(q_offset)[wrist]
                J[:3, column] = (pose_delta[:3, 3] - a[:3, 3]) / epsilon
                J[3:, column] = rotation_vector(pose_delta[:3, :3] @ a[:3, :3].T) / epsilon
            J[3:, :] *= self.angular_weight
            e = np.concatenate((ep, er * self.angular_weight))
            # Response-weighted DLS: predict ACTUAL wrist motion J*S*dq,
            # not the idealized J*dq. In particular, an under-responsive
            # loaded shoulder no longer dominates a proposed micro-adjustment.
            # Regularization penalizes large COMMAND increments to avoid
            # dividing by near-zero shoulder responsiveness.
            gains = response[list(indices)]
            J_eff = J * gains[np.newaxis, :]
            dq = J_eff.T @ np.linalg.solve(
                J_eff @ J_eff.T + np.eye(6) * self.damping**2, e)
            if not np.isfinite(dq).all():
                raise CorrectionError('Invalid Jacobian correction')
            peak = float(np.max(np.abs(dq)))
            if peak > self.max_joint_step:
                dq *= self.max_joint_step / peak
            proposal[list(indices)] += dq
        # Clamp total offset relative to original MoveIt final goal.
        if np.any(np.abs(proposal - q_goal) > self.max_total_offset + 1e-9):
            raise CorrectionError('Cumulative correction would exceed goal-relative joint-offset cap')
        if np.any(proposal < self.limits[:, 0] + self.joint_margin) or np.any(proposal > self.limits[:, 1] - self.joint_margin):
            raise CorrectionError('Correction too close to a URDF joint bound')
        if np.max(np.abs(proposal - q_cmd)) < 1e-6:
            raise CorrectionError('No meaningful safe wrist correction could be generated')
        # Predict geometry only; this does not guarantee a real motor response.
        approximate_measured = q_meas + response * (proposal - q_cmd)
        _, previous_score, _ = self.metrics(q_goal, q_meas, active_wrists)
        _, predicted_score, _ = self.metrics(q_goal, approximate_measured, active_wrists)
        if predicted_score >= previous_score - 1e-7:
            raise CorrectionError('Wrist correction is not predicted to improve the active pose error')
        return proposal
