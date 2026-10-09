"""Offline-validated arm source selector intended for the XR command loop.

Pure Python + NumPy: no SDK, ROS imports, DDS, or motor publications.
The caller owns actual measured state and calls `step` once per control tick.
HOLD is a target-generation request, not a claim that holding is physically safe.
"""
from dataclasses import dataclass
from enum import Enum
from threading import RLock
from typing import Callable
import math
import numpy as np


class BridgeError(ValueError):
    pass


class Source(Enum):
    VR = 'vr'
    TRAJECTORY = 'trajectory'
    HOLD = 'hold'


@dataclass(frozen=True)
class SelectedArmCommand:
    q: np.ndarray
    tau_ff: np.ndarray
    source: Source


def vector14(values, field='joints'):
    try:
        arr = np.asarray(values, dtype=float)
    except (TypeError, ValueError) as exc:
        raise BridgeError(f'{field}: invalid values') from exc
    if arr.shape != (14,) or not np.isfinite(arr).all():
        raise BridgeError(f'{field}: expected 14 finite values')
    return arr.copy()


class XrArmInputMux:
    """Single consumer of VR or sampled ROS targets.

    Before accepting a trajectory, require a recent measured pose and a nearby
    first command. A missing ROS heartbeat changes TRAJECTORY -> HOLD, never VR.
    HOLD captures measured q (not planned q); explicit VR resume checks distance.
    This is not a complete physical safety mechanism.
    """

    def __init__(self, gravity: Callable, *, timeout_s=0.35,
                 feedback_timeout_s=0.25, start_tolerance=0.08,
                 vr_resume_tolerance=0.08):
        if any(not math.isfinite(v) or v <= 0 for v in
               (timeout_s, feedback_timeout_s, start_tolerance, vr_resume_tolerance)):
            raise BridgeError('All timeouts and tolerances must be positive and finite')
        self.gravity = gravity
        self.timeout_s = timeout_s
        self.feedback_timeout_s = feedback_timeout_s
        self.start_tolerance = start_tolerance
        self.vr_resume_tolerance = vr_resume_tolerance
        self._lock = RLock()
        self.source = Source.VR
        self._measured = None
        self._measured_at = None
        self._target = None
        self._hold = None
        self._last_sample_at = None
        self._last_seq = -1

    def update_measured(self, q, now):
        q = vector14(q, 'measured q')
        if not math.isfinite(now):
            raise BridgeError('Invalid feedback time')
        with self._lock:
            self._measured = q
            self._measured_at = float(now)

    def _fresh_measured(self, now):
        if (self._measured is None or self._measured_at is None
                or now < self._measured_at
                or now - self._measured_at > self.feedback_timeout_s):
            raise BridgeError('Measured joint positions unavailable or stale')
        return self._measured.copy()

    def _enter_hold(self, now):
        self._hold = self._fresh_measured(now)
        self._target = None
        self.source = Source.HOLD

    def accept_sample(self, q, source, seq, now):
        q = vector14(q, 'ROS target')
        if source not in ('trajectory', 'hold') or type(seq) is not int or seq < 0:
            raise BridgeError('Invalid source or sequence')
        if not math.isfinite(now):
            raise BridgeError('Invalid sample time')
        with self._lock:
            measured = self._fresh_measured(now)
            if source == 'hold':
                if self.source not in (Source.TRAJECTORY, Source.HOLD):
                    raise BridgeError('Cannot hold a trajectory that does not own the arms')
                if self.source is Source.TRAJECTORY and seq <= self._last_seq:
                    raise BridgeError('Out-of-order trajectory sample')
                self._enter_hold(now)
                self._last_sample_at = now
                return

            if self.source is not Source.TRAJECTORY:
                if np.max(np.abs(q - measured)) > self.start_tolerance:
                    raise BridgeError('First trajectory target differs from measured arm position')
                self._last_seq = -1  # New trajectory stream, including one after HOLD.
            elif seq <= self._last_seq:
                raise BridgeError('Out-of-order trajectory sample')
            self._target = q
            self._last_seq = seq
            self._last_sample_at = now
            self.source = Source.TRAJECTORY

    def enter_hold(self, now):
        with self._lock:
            self._enter_hold(now)

    def resume_vr(self, vr_q, now):
        vr_q = vector14(vr_q, 'VR q')
        with self._lock:
            if self.source is Source.TRAJECTORY:
                raise BridgeError('Hold or cancel trajectory before resuming VR')
            measured = self._fresh_measured(now)
            if np.max(np.abs(vr_q - measured)) > self.vr_resume_tolerance:
                raise BridgeError('VR target is too far from measured arm position')
            self._hold = None
            self.source = Source.VR

    def step(self, vr_q, vr_tau, now):
        """Return intended command; caller must independently gate actuation."""
        if not math.isfinite(now):
            raise BridgeError('Invalid time')
        with self._lock:
            if self.source is Source.TRAJECTORY and (
                    self._last_sample_at is None or now - self._last_sample_at > self.timeout_s):
                self._enter_hold(now)
            if self.source is Source.VR:
                q = vector14(vr_q, 'VR q')
                tau = vector14(vr_tau, 'VR tau')
                return SelectedArmCommand(q, tau, self.source)
            self._fresh_measured(now)
            q = self._target if self.source is Source.TRAJECTORY else self._hold
            if q is None:
                raise BridgeError('No active arm command')
            q = q.copy()
            tau = vector14(self.gravity(q), 'XR torque')
            return SelectedArmCommand(q, tau, self.source)

    def status(self, now):
        with self._lock:
            return {'source': self.source.value,
                    'measured_arm_positions': None if self._measured is None else self._measured.tolist(),
                    'feedback_fresh': self._measured_at is not None and
                    0 <= now - self._measured_at <= self.feedback_timeout_s,
                    'trajectory_last_seq': self._last_seq,
                    'trajectory_age_s': None if self._last_sample_at is None else
                    max(0.0, now - self._last_sample_at)}
