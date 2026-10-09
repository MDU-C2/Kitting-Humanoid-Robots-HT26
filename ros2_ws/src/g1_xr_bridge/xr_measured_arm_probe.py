"""Read-only experiment: feed validated physical LowState into the arm selector.

No Unitree SDK, DDS, ROS, motor commands, XR source patching or actuator path.
The simulated trajectory takeover below DOES NOT OWN the physical robot.
Physical XR continues to be blocked by g1_pipeline.py --mode hardware.

This class reuses the same XrArmInputMux and TrajectoryTrackingValidator
that our eventual integration must respect. It tests *readiness* and
*measured-vs-intended* arm targets, not physical tracking of a command.
"""
from dataclasses import dataclass

import numpy as np

from xr_arm_input_mux import Source, XrArmInputMux, vector14
from xr_hardware_feedback import (FeedbackError, TrajectoryTrackingValidator,
                                  validate_lowstate_snapshot)


@dataclass(frozen=True)
class ProbeStatus:
    source: str
    arm_positions: tuple
    arm_velocities: tuple
    receipt_age_s: float
    max_tracking_error_rad: float
    # This is *always* false; the probe has no actuator interface.
    physically_executed: bool = False


class ReadOnlyMeasuredArmProbe:
    """Dry-run selector consuming *fresh physical* 14-arm measurements.

    A simulated takeover cannot produce a Unitree command. The private mux is
    never handed to the XR controller; preview operations return telemetry only.
    """

    def __init__(self, *, validator=None):
        self.validator = validator or TrajectoryTrackingValidator()
        # A dummy gravity function is never evaluated: this probe never calls
        # mux.step() and cannot provide a motor-target/torque command.
        self._mux = XrArmInputMux(lambda _q: (_ for _ in ()).throw(
            RuntimeError('Read-only probe must never calculate actuator torques')))
        self._receipt_at = None
        self._sequence = -1
        self._started = False
        self._last_target = None
        self._last_measured = None
        self._progress_count = 0

    @property
    def progress_count(self):
        return self._progress_count

    def observe(self, snapshot, *, now):
        measured = validate_lowstate_snapshot(
            snapshot, now=now, max_age_s=self.validator.max_age_s)
        # A fresh reply alone is not evidence of a *new* LowState receipt.
        receipt_at = measured.estimated_receipt_at
        if self._receipt_at is not None:
            if receipt_at + 0.003 < self._receipt_at:
                raise FeedbackError('Measured LowState receipt moved backwards')
            if receipt_at > self._receipt_at + 0.003:
                self._progress_count += 1
        self._receipt_at = max(receipt_at, self._receipt_at or receipt_at)
        self._last_measured = measured
        # Use observer's actual receipt time, NOT poll time. Polling an old
        # snapshot must never refresh the mux's feedback heartbeat.
        self._mux.update_measured(measured.q, self._receipt_at)
        return measured

    def preview_start(self, snapshot, target_q, *, now):
        """Check first MoveIt target against measured state; no motor command."""
        measured = self.observe(snapshot, now=now)
        target = vector14(target_q, 'proposed start q')
        self.validator.validate_start(snapshot, target, now=now)
        self._mux.accept_sample(target, 'trajectory', 0, now)
        self._sequence = 0
        self._started = True
        self._last_target = target
        return self._status(measured, target)

    def preview_path(self, snapshot, target_q, *, now):
        """Check next hypothetical waypoint using actual robot feedback."""
        if not self._started:
            raise FeedbackError('No preview trajectory has been started')
        measured = self.observe(snapshot, now=now)
        target = vector14(target_q, 'proposed waypoint q')
        _, err = self.validator.validate_path(snapshot, target, now=now)
        self._sequence += 1
        self._mux.accept_sample(target, 'trajectory', self._sequence, now)
        self._last_target = target
        return self._status(measured, target, err)

    def preview_hold(self, snapshot, *, now):
        """Exercise the mux's HOLD transition WITHOUT physically holding G1."""
        measured = self.observe(snapshot, now=now)
        self._mux.enter_hold(now)
        self._started = False
        return self._status(measured, measured.q)

    def _status(self, measured, target, err=None):
        if err is None:
            err = float(np.max(np.abs(measured.q - target)))
        return ProbeStatus(source=self._mux.source.value,
                           arm_positions=tuple(float(q) for q in measured.q),
                           arm_velocities=tuple(float(dq) for dq in measured.dq),
                           receipt_age_s=measured.receipt_age_s,
                           max_tracking_error_rad=float(err))
