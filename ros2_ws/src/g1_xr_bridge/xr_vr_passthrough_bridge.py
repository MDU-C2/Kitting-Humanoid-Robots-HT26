"""Opt-in VR-first selector for an isolated XR simulation.

Before the FIRST accepted trajectory, forwards upstream XR's IK position and
feedforward values unchanged. After takeover, reuse the already-tested arm mux.
No DDS/SDK/controller import or physical hardware authorization lives here.
"""
import time

from xr_arm_input_mux import SelectedArmCommand, Source, vector14
from xr_inprocess_bridge import XrInProcessBridge


class VrFirstBridge(XrInProcessBridge):
    def __init__(self, arm_ik=None, *, gravity_fn=None, socket_path=None):
        kwargs = {'gravity_fn': gravity_fn}
        if socket_path is not None:
            kwargs['socket_path'] = socket_path
        super().__init__(arm_ik, **kwargs)
        self._trajectory_ever_accepted = False

    def frame(self, measured_q, vr_q, vr_tau, *, vr_ready, now=None):
        now = time.monotonic() if now is None else now
        with self._lock:
            if self._trajectory_ever_accepted:
                return super().frame(measured_q, vr_q, vr_tau,
                                     vr_ready=vr_ready, now=now)
            # IMPORTANT: No MoveIt ownership yet. Forward the exact original
            # XR values, including its start/VR-ready behaviour. Do not send
            # a HOLD command or replace zero initialization.
            self.mux.update_measured(measured_q, now)
            self._last_frame_at = now
            self._vr_ready = bool(vr_ready)
            if vr_ready:
                self._vr_q = vector14(vr_q, 'VR IK q')
                self._vr_tau = vector14(vr_tau, 'VR IK torque')
                self._last_vr_at = now
            return SelectedArmCommand(vr_q, vr_tau, Source.VR)

    def handle(self, request):
        with self._lock:
            reply = super().handle(request)
            if isinstance(request, dict) and request.get('kind') == 'sample' and reply.get('kind') == 'sample_ack':
                # Do not engage arbitration on a rejected/invalid command.
                self._trajectory_ever_accepted = True
            return reply
