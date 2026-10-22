"""Independent Inspire FTP hand feedback mapping, no ROS/DDS imports.

Original XR FTP SDK: 6 motor channels per side in order
(little, ring, middle, index, thumb bend, thumb rotation).
angle_act: 0..1000, where 1000 is open, 0 closed.
URDF 12 non-mimic hand joints are angular joint coordinates (rad).
This is a provisional linear mapping to be checked against physical motion.
"""
import math

MOTORS = ('little', 'ring', 'middle', 'index', 'thumb_bend', 'thumb_rotation')
URDF_MAX_RAD = (1.4381, 1.4381, 1.4381, 1.4381, 0.5864, 1.1641)
URDF_SUFFIXES = ('little_1', 'ring_1', 'middle_1', 'index_1', 'thumb_2', 'thumb_1')
SOCKET = '/tmp/g1_xr_inspire_ftp.sock'


def normalize_raw(raw):
    if not isinstance(raw, (list, tuple)) or len(raw) != 6:
        raise ValueError('Expected exactly six Inspire FTP angle_act values')
    vals = []
    for a in raw:
        if isinstance(a, bool) or not isinstance(a, (int, float)) or not math.isfinite(a):
            raise ValueError('Invalid FTP angle_act')
        if a < 0 or a > 1000:
            raise ValueError('FTP angle_act outside 0..1000; refusing uncalibrated values')
        vals.append(float(a) / 1000.0)
    return vals


def urdf_positions(side, normalized):
    if side not in ('left', 'right') or len(normalized) != 6:
        raise ValueError('Invalid hand side/state')
    if not all(type(x) in (int, float) and math.isfinite(x) and 0 <= x <= 1 for x in normalized):
        raise ValueError('Invalid normalized FTP motor state')
    return [f'{side}_{suffix}_joint' for suffix in URDF_SUFFIXES], [
        (1.0 - n) * upper for n, upper in zip(normalized, URDF_MAX_RAD)
    ]


def changed_motor_state(measured, motor, direction, amount):
    if len(measured) != 6 or not all(type(x) in (float, int) and math.isfinite(x) and 0 <= x <= 1 for x in measured):
        raise ValueError('Invalid motor state')
    if motor not in MOTORS or direction not in ('open', 'close'):
        raise ValueError('Invalid finger or direction')
    if type(amount) not in (int, float) or not math.isfinite(amount) or not 0 < amount <= 0.05:
        raise ValueError('One command may change at most 0.05 normalized')
    result = list(measured)
    idx = MOTORS.index(motor)
    sign = 1 if direction == 'open' else -1
    result[idx] = max(0.0, min(1.0, result[idx] + sign * amount))
    return result
