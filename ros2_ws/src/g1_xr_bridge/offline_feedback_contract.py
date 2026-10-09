"""OFFLINE ONLY: extract ideal synthetic XR joint feedback for ROS action UI.

This is **not** hardware measurement, not a physical tracking check, and never
supplies an actuator safety decision. Fail closed on malformed synthetic data.
"""
import math
from numbers import Real


class SyntheticFeedbackError(ValueError):
    pass


def synthetic_arm_feedback(status, desired):
    """Return (actual, desired-minus-actual) or None for non-offline XR sim.

    The XR harness exports 29 body positions, with 14 arms in slots 15..28.
    Real Isaac simulator status has sim_joint_positions instead; do not call
    those values synthetic or claim they are validated physical measurements.
    """
    if not isinstance(status, dict) or status.get('hardware_connected') is not False:
        raise SyntheticFeedbackError('Offline-only feedback: receiver identity invalid')
    values = status.get('synthetic_joint_positions')
    if values is None:
        return None  # XR isolated simulator: no synthetic harness; leave blank
    if (status.get('kind') != 'status_ack' or status.get('mode') != 'sim'
            or status.get('xr_loop_fresh') is not True
            or status.get('feedback_fresh') is not True):
        raise SyntheticFeedbackError('Offline synthetic feedback is stale/untrusted')
    if (not isinstance(values, list) or len(values) != 29
            or not isinstance(desired, (list, tuple)) or len(desired) != 14):
        raise SyntheticFeedbackError('Invalid offline synthetic joint lengths')
    for v in values + list(desired):
        if isinstance(v, bool) or not isinstance(v, Real) or not math.isfinite(v):
            raise SyntheticFeedbackError('Nonfinite/invalid offline synthetic joint')
    actual = [float(x) for x in values[15:29]]
    return actual, [float(d) - a for d, a in zip(desired, actual)]
