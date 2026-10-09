"""Experimental motion-mode blend watchdog for ISOLATED SIMULATOR ONLY.

Not a functional safety system, motor torque limit or a G1-rated E-stop.
Must never be used as the sole justification to enable physical execution.
"""
import math
from threading import Lock
import time


class XrMotionGuard:
    def __init__(self, *, ramp_up_s=1.0, ramp_down_s=0.25,
                 heartbeat_timeout_s=0.20, feedback_timeout_s=0.25):
        if any(not math.isfinite(v) or v <= 0 for v in (
                ramp_up_s, ramp_down_s, heartbeat_timeout_s,
                feedback_timeout_s)):
            raise ValueError('Invalid ramp/watchdog settings')
        self.ramp_up_s = ramp_up_s
        self.ramp_down_s = ramp_down_s
        self.heartbeat_timeout_s = heartbeat_timeout_s
        self.feedback_timeout_s = feedback_timeout_s
        self._lock = Lock()
        self._heartbeat_at = None
        self._last_step_at = None
        self._weight = 0.0
        self._shutdown = False

    def heartbeat(self, now=None):
        when = time.monotonic() if now is None else float(now)
        if not math.isfinite(when):
            raise ValueError('Invalid heartbeat')
        with self._lock:
            if not self._shutdown:
                self._heartbeat_at = when

    def disarm(self):
        with self._lock:
            self._shutdown = True
            self._heartbeat_at = None

    def step(self, lowstate_at, now=None):
        when = time.monotonic() if now is None else float(now)
        if not math.isfinite(when):
            self.disarm()
            return 0.0
        try:
            received = None if lowstate_at is None else float(lowstate_at)
        except (ValueError, TypeError):
            received = None
        if received is not None and not math.isfinite(received):
            received = None
        with self._lock:
            # Invalid/missing timestamps must prevent ramping IN.
            heartbeat_ok = (self._heartbeat_at is not None and
                            0 <= when - self._heartbeat_at <= self.heartbeat_timeout_s)
            feedback_ok = (received is not None and
                           0 <= when - received <= self.feedback_timeout_s)
            enable = not self._shutdown and heartbeat_ok and feedback_ok
            dt = (0.0 if self._last_step_at is None else
                  max(0.0, min(when - self._last_step_at, 0.02)))
            self._last_step_at = when
            if enable:
                self._weight = min(1.0, self._weight + dt / self.ramp_up_s)
            else:
                self._weight = max(0.0, self._weight - dt / self.ramp_down_s)
            return self._weight
