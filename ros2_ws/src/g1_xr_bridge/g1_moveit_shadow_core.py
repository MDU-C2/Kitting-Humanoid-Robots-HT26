"""Read-only MoveIt DISPLAY plan -> virtual XR source-selection dry run.

This module has *no* connection to XR runtime, robot command channels, ROS
publishers or controllers. A MoveIt DisplayTrajectory is a visualization
artifact, NOT an execution request. An accepted shadow plan cannot cause
movement and does not demonstrate physical trajectory tracking.
"""
from dataclasses import dataclass
from numbers import Real
import math

import numpy as np

from xr_arm_input_mux import XrArmInputMux, Source
from xr_arm_trajectory import ARM_JOINT_NAMES, Trajectory, TrajectoryError, Waypoint
from xr_hardware_feedback import TrajectoryTrackingValidator, validate_lowstate_snapshot


MAX_POINTS = 2500
MAX_DURATION_S = 60.0
VIRTUAL_STEP_S = 0.02


@dataclass(frozen=True)
class ShadowResult:
    joint_count: int
    waypoint_count: int
    duration_s: float
    max_virtual_displacement_rad: float
    max_virtual_step_rad: float
    virtual_samples: int
    final_virtual_source: str
    start_mode_machine: int
    read_only: bool = True
    execution_enabled: bool = False
    physical_execution: bool = False


def _seconds(duration):
    sec, ns = getattr(duration, 'sec', None), getattr(duration, 'nanosec', None)
    if type(sec) is not int or type(ns) is not int or not (0 <= ns < 1_000_000_000):
        raise TrajectoryError('Invalid MoveIt time_from_start')
    return sec + ns * 1e-9


def _make_trajectory(jt, measured_q):
    """Convert 7 or 14 arm joints, leaving an unplanned arm at measured q."""
    names = tuple(jt.joint_names)
    permitted = (set(ARM_JOINT_NAMES), set(ARM_JOINT_NAMES[:7]), set(ARM_JOINT_NAMES[7:]))
    if (len(names) not in (7, 14) or len(set(names)) != len(names)
            or set(names) not in permitted):
        raise TrajectoryError('Display plan must contain exactly one or both G1 arms; no other joints')
    if not 2 <= len(jt.points) <= MAX_POINTS:
        raise TrajectoryError('Display plan requires 2..2500 waypoints')
    expanded = []
    for point in jt.points:
        t = _seconds(point.time_from_start)
        positions = np.asarray(point.positions, dtype=float)
        if positions.shape != (len(names),) or not np.isfinite(positions).all():
            raise TrajectoryError('Invalid MoveIt waypoint positions')
        velocities = None
        if point.velocities:
            velocities = np.asarray(point.velocities, dtype=float)
            if velocities.shape != (len(names),) or not np.isfinite(velocities).all():
                raise TrajectoryError('Invalid MoveIt waypoint velocities')
        if len(names) == 7:
            # Missing arm holds its REAL measured starting pose in this preview.
            q = measured_q.copy()
            v = np.zeros(14) if velocities is not None else None
            for i, joint in enumerate(names):
                index = ARM_JOINT_NAMES.index(joint)
                q[index] = positions[i]
                if v is not None:
                    v[index] = velocities[i]
            expanded.append(Waypoint(t, q, v))
        else:
            expanded.append(Waypoint(t, positions, velocities))
    trajectory = Trajectory(ARM_JOINT_NAMES if len(names) == 7 else names, expanded)
    if abs(trajectory.points[0][0]) > 1e-6:
        raise TrajectoryError('First MoveIt waypoint must start at t=0 for shadow check')
    if trajectory.duration > MAX_DURATION_S:
        raise TrajectoryError('MoveIt shadow plan exceeds 60 seconds')
    return trajectory, len(names)


def inspect_display_plan(display, snapshot, *, now):
    """Read real G1 q ONCE; replay trajectory only in an isolated virtual mux.

    This function never interprets a plan as motor authority. After initial
    measured-start validation, all future selector feedback is virtual/synthetic;
    no tracking metrics or simulated motion are reported as physical results.
    """
    if not isinstance(now, Real) or isinstance(now, bool) or not math.isfinite(now):
        raise TrajectoryError('Invalid monotonic shadow timestamp')
    measured = validate_lowstate_snapshot(snapshot, now=now)
    candidates = display.trajectory
    if len(candidates) != 1:
        raise TrajectoryError('Shadow accepts exactly one MoveIt RobotTrajectory')
    # A message may have a multi-DoF section: don't silently ignore motion.
    robot_trajectory = candidates[0]
    mdof = getattr(robot_trajectory, 'multi_dof_joint_trajectory', None)
    if mdof is not None and (getattr(mdof, 'joint_names', None) or getattr(mdof, 'points', None)):
        raise TrajectoryError('Multi-DoF MoveIt trajectories are not supported by the arm shadow')
    trajectory, joint_count = _make_trajectory(robot_trajectory.joint_trajectory, measured.q)
    TrajectoryTrackingValidator().validate_start(
        snapshot, trajectory.points[0][1], now=now)

    # This mux is detached from VR, XR and the robot. In virtual replay only,
    # feed it virtual q as if ideal simulated tracking occurred. NEVER publish.
    mux = XrArmInputMux(lambda _q: np.zeros(14))
    mux.update_measured(measured.q, 0.0)
    last_q = measured.q.copy()
    max_displacement = 0.0
    max_step = 0.0
    count = max(1, math.ceil(trajectory.duration / VIRTUAL_STEP_S))
    if count > 3000:
        raise TrajectoryError('Excessive virtual samples')
    for i in range(count + 1):
        t = min(i * VIRTUAL_STEP_S, trajectory.duration)
        q = trajectory.sample(t)
        mux.update_measured(q, t)  # VIRTUAL replay state; NOT hardware state
        mux.accept_sample(q, 'trajectory', i, t)
        max_step = max(max_step, float(np.max(np.abs(q - last_q))))
        max_displacement = max(max_displacement, float(np.max(np.abs(q - measured.q))))
        last_q = q
    mux.enter_hold(trajectory.duration)
    if mux.source is not Source.HOLD:
        raise AssertionError('Virtual selector did not enter HOLD')
    return ShadowResult(joint_count, len(trajectory.points), trajectory.duration,
                        max_displacement, max_step, count + 1,
                        mux.source.value, measured.mode_machine)
