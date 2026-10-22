"""MoveIt Inspire FTP hand trajectories, checked against hand joint URDF limits.

The source URDF provides position and velocity limits. Physical FTP motor
calibration is still provisional; this is NOT contact or force control.
"""
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from g1_ftp_hand_math import URDF_MAX_RAD, URDF_SUFFIXES

# G1_FTP_NATIVE_URDF_MOTION_LIMITS_V1: no per-goal displacement, normalized speed or per-command lead caps.
# Goal acceptance tolerance is not a motion restriction.
GOAL_TOL_NORM = 0.015


def urdf_limits():
    """Read actual limits from the same Inspire FTP robot description MoveIt uses."""
    path = Path(__file__).resolve().parent.parent / 'g1_description' / 'urdf' / 'g1_29dof_with_inspire_hand_ftp.urdf'
    root = ET.parse(path).getroot()
    found = {}
    for side in ('left', 'right'):
        for suffix, configured_upper in zip(URDF_SUFFIXES, URDF_MAX_RAD):
            name = f'{side}_{suffix}_joint'
            joint = root.find(f"./joint[@name='{name}']")
            if joint is None or joint.get('type') != 'revolute':
                raise RuntimeError(f'Missing revolute hand joint in URDF: {name}')
            limit = joint.find('limit')
            if limit is None:
                raise RuntimeError(f'Missing URDF limit for {name}')
            lower = float(limit.attrib['lower'])
            upper = float(limit.attrib['upper'])
            velocity = float(limit.attrib['velocity'])
            if not all(map(math.isfinite, (lower, upper, velocity))) or lower != 0.0 or upper <= lower or velocity <= 0:
                raise RuntimeError(f'Invalid or unsupported hand URDF limits for {name}')
            # The existing hardware converter maps a normalized range 0..1
            # linearly to 0..URDF_MAX_RAD. Do not silently mix different geometry.
            if not math.isclose(upper, configured_upper, rel_tol=0, abs_tol=1e-5):
                raise RuntimeError(f'FTP motor/URDF mapping mismatch for {name}: {upper} != {configured_upper}')
            found[name] = (lower, upper, velocity)
    return found


# Fail closed at action startup rather than move against unknown geometry.
URDF_LIMITS = urdf_limits()


def joint_names(side):
    if side not in ('left', 'right'):
        raise ValueError('Expected left or right hand')
    return tuple(f'{side}_{s}_joint' for s in URDF_SUFFIXES)


def q_to_motor(q):
    if len(q) != 6:
        raise ValueError('Six independent hand joint positions required')
    values = []
    for j, maxrad in zip(q, URDF_MAX_RAD):
        if type(j) not in (int, float) or not math.isfinite(j) or j < 0 or j > maxrad:
            raise ValueError('Hand joint position outside URDF bounds')
        values.append(1.0 - float(j) / maxrad)
    return values


def motor_to_q(motor):
    if len(motor) != 6 or not all(type(v) in (int, float) and math.isfinite(v) and 0 <= v <= 1 for v in motor):
        raise ValueError('Invalid six-motor feedback')
    return [(1.0-float(x))*lim for x, lim in zip(motor, URDF_MAX_RAD)]


def validate_points(names, points, side):
    expected = joint_names(side)
    if len(names) != 6 or len(set(names)) != 6 or set(names) != set(expected):
        raise ValueError(f'Expected exactly six independent {side} hand joint names')
    if not points:
        raise ValueError('Trajectory must contain a waypoint')
    order = [names.index(n) for n in expected]
    prev_t = -1.0
    prev_q = None
    samples = []
    for p in points:
        d = p.time_from_start
        t = float(d.sec) + float(d.nanosec) / 1e9
        if not math.isfinite(t) or t < 0 or t <= prev_t:
            raise ValueError('Hand trajectory timestamps must be nonnegative and strictly increasing')
        if len(p.positions) != 6:
            raise ValueError('Each point must include six joint positions')
        ordered = [p.positions[i] for i in order]
        normalized = q_to_motor(ordered)
        joint_vel_limits = [URDF_LIMITS[n][2] for n in expected]
        # Also respect explicit waypoint velocities, if the planner supplies them.
        provided_velocities = getattr(p, 'velocities', [])
        if len(provided_velocities) == 6 and any(
                abs(provided_velocities[names.index(joint)]) > upper + 1e-8
                for joint, upper in zip(expected, joint_vel_limits)):
            raise ValueError('Hand trajectory waypoint velocity exceeds URDF joint limit')
        # Never silently ignore velocities, accelerations or efforts that imply an
        # unsupported controller behavior; MoveIt normally provides velocity arrays.
        for field in ('velocities','accelerations','effort'):
            a = getattr(p, field, [])
            if len(a) not in (0, 6) or not all(math.isfinite(v) for v in a):
                raise ValueError(f'Invalid {field} in hand trajectory')
        if prev_q is not None:
            if any(abs(a-b)/(t-prev_t) > maxvel+1e-8
                   for a,b,maxvel in zip(ordered,prev_q,joint_vel_limits)):
                raise ValueError('Hand trajectory segment exceeds URDF joint velocity limit')
        samples.append((t, normalized))
        prev_q = ordered
        prev_t = t
    return samples


def validate_start_and_range(measured, samples):
    if len(measured) != 6 or not all(type(v) in (int,float) and math.isfinite(v) and 0<=v<=1 for v in measured):
        raise ValueError('Fresh measured six-channel hand state is required')
    max_change = max(abs(a-b) for a,b in zip(samples[0][1],measured))
    if samples[0][0] == 0:
        if max_change > GOAL_TOL_NORM:
            raise ValueError('First t=0 hand waypoint differs from measured start state')
    else:
        for side in ('left', 'right'):
            names = joint_names(side)
            velocities = [URDF_LIMITS[n][2] for n in names]
            if any(abs(a-b)*upper/samples[0][0] > velocity+1e-8
                   for a,b,upper,velocity in zip(samples[0][1],measured,URDF_MAX_RAD,velocities)):
                raise ValueError('First hand waypoint exceeds URDF joint velocity limit')


def interpolate(samples, t, initial):
    if t <= 0:
        return list(initial)
    last_t, last = 0.0, initial
    for next_t, target in samples:
        if t <= next_t:
            alpha = max(0., min(1., (t-last_t)/(next_t-last_t)))
            return [float(a + (b-a)*alpha) for a,b in zip(last,target)]
        last_t, last = next_t,target
    return list(samples[-1][1])

