"""Offline trajectory / VR command arbiter for Unitree XR G1 arms.

This module never opens DDS, creates a robot controller, or publishes commands.
Feed it XR's existing VR (q, tau_ff) pair, or timed joint-space waypoints.
For trajectory torque, inject a callback using XR's existing Pinocchio model.
Not a complete ROS 2 FollowJointTrajectory controller or a hardware safety system.
"""
from dataclasses import dataclass
from enum import Enum
from threading import RLock
from typing import Callable, Optional, Sequence
import math
import numpy as np

ARM_JOINT_NAMES = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint",
    "left_wrist_yaw_joint", "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint",
    "right_wrist_pitch_joint", "right_wrist_yaw_joint",
)
N = len(ARM_JOINT_NAMES)

class TrajectoryError(ValueError):
    pass

class Mode(Enum):
    VR = "vr"
    TRAJECTORY = "trajectory"
    HOLD = "hold"

@dataclass(frozen=True)
class Waypoint:
    time_from_start: float
    positions: Sequence[float]
    velocities: Optional[Sequence[float]] = None

@dataclass(frozen=True)
class ArmCommand:
    q: np.ndarray
    tau_ff: np.ndarray
    source: Mode


def _vector(values, label):
    arr = np.asarray(values, dtype=float)
    if arr.shape != (N,) or not np.all(np.isfinite(arr)):
        raise TrajectoryError(f"{label} must be exactly {N} finite numbers")
    return arr.copy()

class Trajectory:
    """Validated, name-mapped joint trajectory. Position interpolation only.

    Cubic Hermite interpolation is used where both adjacent points specify
    velocities; otherwise linear interpolation is used (prototype behavior).
    """
    def __init__(self, joint_names: Sequence[str], waypoints: Sequence[Waypoint]):
        names = tuple(joint_names)
        if len(names) != N or len(set(names)) != N or set(names) != set(ARM_JOINT_NAMES):
            raise TrajectoryError("Trajectory must contain each of the 14 G1 arm joints exactly once")
        if not waypoints:
            raise TrajectoryError("Trajectory must contain at least one waypoint")
        indices = [names.index(name) for name in ARM_JOINT_NAMES]
        self.points = []
        last_t = -1.0
        for wp in waypoints:
            t = float(wp.time_from_start)
            if not math.isfinite(t) or t < 0 or t <= last_t:
                raise TrajectoryError("Waypoint times must be finite and strictly increasing from t >= 0")
            q = _vector(wp.positions, "waypoint positions")[indices]
            dq = None if wp.velocities is None else _vector(wp.velocities, "waypoint velocities")[indices]
            self.points.append((t, q, dq))
            last_t = t
        self.duration = last_t

    def sample(self, seconds: float) -> np.ndarray:
        if not math.isfinite(seconds):
            raise TrajectoryError("Elapsed time must be finite")
        if seconds <= self.points[0][0]:
            return self.points[0][1].copy()
        if seconds >= self.duration:
            return self.points[-1][1].copy()
        for (t0, q0, v0), (t1, q1, v1) in zip(self.points, self.points[1:]):
            if t0 <= seconds <= t1:
                dt = t1 - t0
                u = (seconds - t0) / dt
                if v0 is None or v1 is None:
                    return (1.0 - u) * q0 + u * q1
                return ((2*u**3 - 3*u**2 + 1) * q0
                        + (u**3 - 2*u**2 + u) * dt * v0
                        + (-2*u**3 + 3*u**2) * q1
                        + (u**3 - u**2) * dt * v1)
        raise AssertionError("Unreachable sample time")


class ArmCommandRouter:
    """Mutually exclusive VR / trajectory target selector, offline only.

    start_trajectory requires explicit caller intent. On completion or cancel
    holds the terminal/sampled pose; it does NOT silently return to VR.
    """
    def __init__(self, gravity_torque: Callable[[np.ndarray], Sequence[float]]):
        self._gravity = gravity_torque
        self._lock = RLock()
        self.mode = Mode.VR
        self._last_vr: Optional[ArmCommand] = None
        self._trajectory: Optional[Trajectory] = None
        self._start_time = 0.0
        self._hold_q: Optional[np.ndarray] = None

    def submit_vr(self, q: Sequence[float], tau_ff: Sequence[float]) -> None:
        with self._lock:
            self._last_vr = ArmCommand(_vector(q, "VR q"), _vector(tau_ff, "VR tau"), Mode.VR)

    def start_trajectory(self, trajectory: Trajectory, measured_q: Sequence[float],
                         now: float, max_start_error: float = 0.08) -> None:
        q_measured = _vector(measured_q, "measured q")
        if not math.isfinite(now) or max_start_error <= 0:
            raise TrajectoryError("Invalid start time or start tolerance")
        if np.max(np.abs(trajectory.points[0][1] - q_measured)) > max_start_error:
            raise TrajectoryError("Trajectory start disagrees with measured joints; refusing transfer")
        with self._lock:
            if self.mode is Mode.TRAJECTORY:
                raise TrajectoryError("A trajectory already owns the arms")
            self._trajectory = trajectory
            self._start_time = float(now)
            self.mode = Mode.TRAJECTORY

    def tick(self, now: float) -> Optional[ArmCommand]:
        """Sample intended command. Caller must decide whether to actuate."""
        with self._lock:
            if not math.isfinite(now):
                raise TrajectoryError("Time must be finite")
            if self.mode is Mode.VR:
                return self._last_vr
            if self.mode is Mode.TRAJECTORY:
                elapsed = max(0.0, now - self._start_time)
                q = self._trajectory.sample(elapsed)
                if elapsed >= self._trajectory.duration:
                    self._hold_q = q.copy()
                    self._trajectory = None
                    self.mode = Mode.HOLD
            else:
                q = self._hold_q.copy()
            tau = _vector(self._gravity(q), "XR gravity torque")
            return ArmCommand(q.copy(), tau, self.mode)

    def cancel_to_hold(self, measured_q: Sequence[float]) -> None:
        """Cancel planned motion; hold the measured arm configuration."""
        with self._lock:
            self._hold_q = _vector(measured_q, "measured q")
            self._trajectory = None
            self.mode = Mode.HOLD

    def resume_vr(self, measured_q: Sequence[float], max_error: float = 0.08) -> None:
        """Allow VR only if its latest target is near current measured joints."""
        q = _vector(measured_q, "measured q")
        with self._lock:
            if self.mode is Mode.TRAJECTORY:
                raise TrajectoryError("Cancel the trajectory before resuming VR")
            if self._last_vr is None:
                raise TrajectoryError("No valid VR target available")
            if np.max(np.abs(self._last_vr.q - q)) > max_error:
                raise TrajectoryError("VR target is too far from measured pose; resynchronize XR first")
            self._hold_q = None
            self.mode = Mode.VR
