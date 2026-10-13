"""Shared MoveIt trajectory validation/conversion; no ROS node or motor ownership.

Extracted verbatim from the CURRENT offline server, to keep trajectory semantics.
"""
import numpy as np
from xr_arm_trajectory import ARM_JOINT_NAMES, Trajectory, TrajectoryError, Waypoint


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
