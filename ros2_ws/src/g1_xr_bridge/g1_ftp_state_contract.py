"""Read-only Inspire FTP actuator-state contract (not URDF joint angles).

The original XR Inspire_Controller_FTP reads rt/inspire_hand/state/{l,r}
angle_act, six values per hand in [0, 1000]. Its state uses angle_act/1000.
This module only validates those measured normalized actuator values. It MUST
NOT be used as 12 revolute URDF joint positions without a verified mapping.
No Unitree SDK, ROS, motor commands, or XR imports.
"""
from dataclasses import dataclass
import math
from numbers import Real


FTP_NAMES = tuple(
    f'{side}:{name}'
    for side in ('left', 'right')
    for name in ('pinky', 'ring', 'middle', 'index', 'thumb_bend', 'thumb_rotation')
)
FTP_SOCKET = '/tmp/g1_xr_ftp_readonly.sock'


class FtpFeedbackError(ValueError):
    """Malformed, stale or incomplete FTP hand state."""


def normalize_angle_act(raw):
    """Validate the 6 raw Inspire actuator encoder values; 0 is valid."""
    if not isinstance(raw, (list, tuple)) or len(raw) != 6:
        raise FtpFeedbackError('FTP angle_act must contain 6 values')
    for value in raw:
        if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
            raise FtpFeedbackError('FTP angle_act must be finite numeric values')
        if value < 0 or value > 1000:
            raise FtpFeedbackError('FTP angle_act must be in [0, 1000]')
    return [float(value) / 1000.0 for value in raw]


@dataclass(frozen=True)
class FtpFeedback:
    normalized: tuple
    left_age_s: float
    right_age_s: float
    left_received_at: float
    right_received_at: float


def validate_ftp_snapshot(snapshot, *, now, max_age_s=0.25):
    """Check measured normalized actuators from a local read-only observer.

    `now` is local monotonic time in the same host/container timebase as the
    observer's ages. Successful validation is NOT robot identity verification.
    """
    if (not isinstance(snapshot, dict) or snapshot.get('kind') != 'ftp_state'
            or snapshot.get('source') != 'inspire_ftp_state'
            or snapshot.get('hardware_connected') is not True
            or snapshot.get('read_only') is not True
            or snapshot.get('both_fresh') is not True
            or snapshot.get('actuator_names') != list(FTP_NAMES)):
        raise FtpFeedbackError('Not a fresh read-only Inspire FTP response')
    if not isinstance(now, Real) or isinstance(now, bool) or not math.isfinite(now):
        raise FtpFeedbackError('Invalid local monotonic timestamp')
    if not isinstance(max_age_s, Real) or not math.isfinite(max_age_s) or max_age_s <= 0:
        raise FtpFeedbackError('Invalid FTP maximum age')
    ages = []
    for name in ('left_age_s', 'right_age_s'):
        value = snapshot.get(name)
        if (isinstance(value, bool) or not isinstance(value, Real)
                or not math.isfinite(value) or value < 0 or value > max_age_s):
            raise FtpFeedbackError(f'Stale/missing FTP {name}')
        ages.append(float(value))
    pos = snapshot.get('normalized_angle_act')
    if not isinstance(pos, list) or len(pos) != 12:
        raise FtpFeedbackError('Expected 12 normalized FTP actuator values')
    if any(isinstance(x, bool) or not isinstance(x, Real)
           or not math.isfinite(x) or x < 0 or x > 1 for x in pos):
        raise FtpFeedbackError('Invalid normalized FTP actuator value')
    return FtpFeedback(tuple(float(v) for v in pos), ages[0], ages[1],
                       float(now - ages[0]), float(now - ages[1]))
