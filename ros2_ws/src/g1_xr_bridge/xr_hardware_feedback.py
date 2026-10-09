"""Validate READ-ONLY G1 measurements and candidate arm trajectories.

No ROS, Unitree SDK, DDS, motor commands, or XR source changes. Intended for
future action-server feedback checks; it is NOT an actuator safety controller.
The measured source must be the existing read-only rt/lowstate observer.

FTP hand state is deliberately NOT fabricated: this contract covers only the
29 G1 body joints and its 14 arm joints. Original XR retains hand ownership.
"""
from dataclasses import dataclass
import math
from numbers import Real

import numpy as np

from g1_state_contract import ARM_INDICES, JOINT_NAMES_29


class FeedbackError(ValueError):
    """Reject a stale, malformed, or out-of-tolerance hardware measurement."""


def _finite_vector(value, size, label):
    if not isinstance(value, (list, tuple, np.ndarray)) or len(value) != size:
        raise FeedbackError(f'{label}: expected {size} numeric values')
    if any(isinstance(x, bool) or not isinstance(x, Real) or not math.isfinite(x)
           for x in value):
        raise FeedbackError(f'{label}: nonfinite or nonnumeric values')
    return np.asarray(value, dtype=np.float64).copy()


@dataclass(frozen=True)
class MeasuredArmFeedback:
    q: np.ndarray
    dq: np.ndarray
    mode_machine: int
    receipt_age_s: float
    estimated_receipt_at: float


def validate_lowstate_snapshot(data, *, now, max_age_s=0.25):
    """Verify one observer IPC response and extract its measured 14-arm state.

    Caller-provided monotonic `now` must use the same local clock as the
    observer's `age_s`. Being fresh does not authenticate which robot sent DDS.
    """
    if not isinstance(data, dict):
        raise FeedbackError('Observer response is not an object')
    if (data.get('kind') != 'state' or data.get('mode') != 'observe'
            or data.get('hardware_connected') is not True
            or data.get('read_only') is not True
            or data.get('source') != 'lowstate'
            or data.get('fresh') is not True):
        raise FeedbackError('Not a fresh read-only G1 LowState observer response')
    if data.get('joint_names') != list(JOINT_NAMES_29):
        raise FeedbackError('29-joint motor ordering mismatch')
    if isinstance(now, bool) or not isinstance(now, Real) or not math.isfinite(now):
        raise FeedbackError('Invalid local monotonic timestamp')
    if not math.isfinite(max_age_s) or max_age_s <= 0:
        raise FeedbackError('Invalid maximum feedback age')
    age = data.get('age_s')
    if (isinstance(age, bool) or not isinstance(age, Real)
            or not math.isfinite(age) or age < 0 or age > max_age_s):
        raise FeedbackError('Stale or invalid LowState receipt age')
    mode = data.get('mode_machine')
    if type(mode) is not int or not 0 <= mode <= 255:
        raise FeedbackError('Invalid mode_machine')
    q = _finite_vector(data.get('positions'), 29, 'measured positions')
    dq = _finite_vector(data.get('velocities'), 29, 'measured velocities')
    return MeasuredArmFeedback(q[list(ARM_INDICES)], dq[list(ARM_INDICES)], mode,
                               float(age), float(now - age))


class TrajectoryTrackingValidator:
    """Only assess whether measured arm feedback matches planned targets.

    This is validation, NOT motor authority. Integrating the validator with
    an action server and the robot's independent safety stop remains required.
    Tolerances must be selected and verified for the actual application.
    """

    def __init__(self, *, start_tolerance_rad=0.08, path_tolerance_rad=0.15,
                 goal_tolerance_rad=0.05, stopped_velocity_rad_s=0.1,
                 max_age_s=0.25):
        params = (start_tolerance_rad, path_tolerance_rad,
                  goal_tolerance_rad, stopped_velocity_rad_s, max_age_s)
        if any(isinstance(x, bool) or not isinstance(x, Real)
               or not math.isfinite(x) or x <= 0 for x in params):
            raise FeedbackError('Feedback tolerances must be positive finite numbers')
        self.start_tolerance_rad = float(start_tolerance_rad)
        self.path_tolerance_rad = float(path_tolerance_rad)
        self.goal_tolerance_rad = float(goal_tolerance_rad)
        self.stopped_velocity_rad_s = float(stopped_velocity_rad_s)
        self.max_age_s = float(max_age_s)

    @staticmethod
    def _error(desired, actual):
        wanted = _finite_vector(desired, 14, 'commanded positions')
        return float(np.max(np.abs(wanted - actual)))

    def validate_start(self, snapshot, start_q, *, now):
        actual = validate_lowstate_snapshot(snapshot, now=now, max_age_s=self.max_age_s)
        err = self._error(start_q, actual.q)
        if err > self.start_tolerance_rad:
            raise FeedbackError(f'Trajectory start differs from measured arms: {err:.4f} rad')
        return actual

    def validate_path(self, snapshot, desired_q, *, now):
        actual = validate_lowstate_snapshot(snapshot, now=now, max_age_s=self.max_age_s)
        err = self._error(desired_q, actual.q)
        if err > self.path_tolerance_rad:
            raise FeedbackError(f'Arm tracking error exceeds path tolerance: {err:.4f} rad')
        return actual, err

    def validate_goal(self, snapshot, goal_q, *, now):
        actual = validate_lowstate_snapshot(snapshot, now=now, max_age_s=self.max_age_s)
        err = self._error(goal_q, actual.q)
        speed = float(np.max(np.abs(actual.dq)))
        if err > self.goal_tolerance_rad:
            raise FeedbackError(f'Arm goal not reached: {err:.4f} rad')
        if speed > self.stopped_velocity_rad_s:
            raise FeedbackError(f'Arms still moving at goal: {speed:.4f} rad/s')
        return actual
