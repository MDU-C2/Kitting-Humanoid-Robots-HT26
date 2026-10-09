"""In-process ROS-to-XR command selector for G1 29-DoF arms.

This module does NOT import Unitree DDS or publish motor commands. It is an
adapter for the *original* XR loop. The companion entrypoint only enables this
adapter with XR's --sim AND --motion arguments; real hardware is hard-blocked.
"""
import json
import os
import socketserver
import threading
import time

import numpy as np

from xr_arm_input_mux import BridgeError, Source, XrArmInputMux, vector14
from xr_arm_trajectory import ARM_JOINT_NAMES
from xr_ipc_protocol import MAX_LINE, PROTOCOL, _vector14

SIM_SOCKET = '/tmp/g1_xr_bridge_inprocess_sim.sock'
VR_TIMEOUT_S = 0.25


class XrInProcessBridge:
    def __init__(self, arm_ik=None, *, gravity_fn=None, socket_path=SIM_SOCKET):
        self._lock = threading.RLock()
        self._last_frame_at = None
        self._last_vr_at = None
        self._synthetic_joints = None  # offline harness only, never real feedback
        self._sim_joints = None  # actual simulator LowState supplied by XR main
        self._vr_q = None
        self._vr_tau = None
        self._socket_path = socket_path
        self._server = None
        self._thread = None
        if gravity_fn is not None:
            self._gravity = gravity_fn
        else:
            # Preserve the original XR model and the same RNEA(v=0,a=0).
            # Separate Pinocchio data avoids racing XR's IK solver data.
            import pinocchio as pin
            model = arm_ik.reduced_robot.model
            if (model.nq != 14 or model.nv != 14 or
                    tuple(model.names[1:]) != ARM_JOINT_NAMES):
                raise BridgeError('XR dynamics joint order does not match 14 G1 arms')
            data = model.createData()
            zeros = np.zeros(model.nv)

            def xr_gravity(q):
                with self._lock:
                    return pin.rnea(model, data, vector14(q), zeros, zeros).copy()
            self._gravity = xr_gravity
        self.mux = XrArmInputMux(self._gravity)

    def frame(self, measured_q, vr_q, vr_tau, *, vr_ready, now=None):
        """Called ONLY from XR's main loop. Returns a selected arm command.

        XR remains sole owner of the eventual ctrl_dual_arm(q,tau_ff) call.
        VR IK is still evaluated by the caller exactly as in upstream XR.
        """
        now = time.monotonic() if now is None else now
        with self._lock:
            self.mux.update_measured(measured_q, now)
            self._last_frame_at = now
            if vr_ready:
                self._vr_q = vector14(vr_q, 'VR IK q')
                self._vr_tau = vector14(vr_tau, 'VR IK torque')
                self._last_vr_at = now
            elif self.mux.source is Source.VR:
                # Never drive XR VR from unready tracking in integrated mode.
                self.mux.enter_hold(now)
            safe_vr_q = self._vr_q if self._vr_q is not None else measured_q
            safe_vr_tau = self._vr_tau if self._vr_tau is not None else np.zeros(14)
            return self.mux.step(safe_vr_q, safe_vr_tau, now)

    def update_synthetic_state(self, measured_arms):
        """OFFLINE harness only: ideal state, NOT physical nor Isaac feedback."""
        with self._lock:
            joints = [0.0] * 29
            joints[15:29] = vector14(measured_arms).tolist()
            self._synthetic_joints = joints

    def update_sim_state(self, all_motor_q):
        """XR simulator only: forward its actual LowState, not fake tracking."""
        values = np.asarray(all_motor_q, dtype=float)
        if values.shape != (35,) or not np.isfinite(values).all():
            raise BridgeError('Invalid simulator 35-motor feedback')
        with self._lock:
            self._sim_joints = values[:29].tolist()

    def handle(self, request):
        if not isinstance(request, dict) or request.get('protocol') != PROTOCOL:
            raise BridgeError('Invalid bridge protocol')
        now = time.monotonic()
        kind = request.get('kind')
        with self._lock:
            status = self.mux.status(now)
            status['xr_loop_fresh'] = (self._last_frame_at is not None
                                       and 0 <= now - self._last_frame_at <= 0.25)
            if kind == 'ping':
                return {'kind': 'pong', 'hardware_connected': False,
                        'mode': 'sim', 'xr_loop_fresh': status['xr_loop_fresh']}
            if kind == 'status':
                return {'kind': 'status_ack', 'hardware_connected': False,
                        'mode': 'sim',
                        'synthetic_joint_positions': self._synthetic_joints,
                        'sim_joint_positions': self._sim_joints,
                        **status}
            if kind == 'resume_vr':
                if (not status['xr_loop_fresh'] or self._vr_q is None
                        or self._last_vr_at is None
                        or now - self._last_vr_at > VR_TIMEOUT_S):
                    raise BridgeError('Cannot resume VR: no fresh tracking/IK')
                self.mux.resume_vr(self._vr_q, now)
                return {'kind': 'resume_vr_ack', 'source': 'vr',
                        'hardware_connected': False}
            if kind != 'sample':
                raise BridgeError('Unsupported command')
            if not status['xr_loop_fresh']:
                raise BridgeError('XR loop is not running or feedback is stale')
            q = _vector14(request.get('q'))
            source = request.get('source')
            seq = request.get('seq')
            self.mux.accept_sample(q, source, seq, now)
            # Accepted != executed. The XR main loop applies selected target
            # at its next tick. This ACK is only for ROS SIMULATION testing.
            tau_ff = vector14(self._gravity(q), 'XR gravity torque')
            return {'kind': 'sample_ack', 'source': source, 'seq': seq,
                    'tau_ff': tau_ff.tolist(), 'hardware_connected': False,
                    'selected_source': self.mux.source.value,
                    'executed': False}

    def start(self):
        with self._lock:
            if self._server is not None:
                raise BridgeError('IPC already running')
            if os.path.lexists(self._socket_path):
                raise BridgeError(f'IPC socket exists: {self._socket_path}; stop old process')
            server = _Server(self._socket_path, _Handler)
            server.receiver = self
            os.chmod(self._socket_path, 0o600)
            self._server = server
            self._thread = threading.Thread(target=server.serve_forever,
                                            kwargs={'poll_interval': 0.05},
                                            daemon=True)
            self._thread.start()

    def stop(self):
        with self._lock:
            server = self._server
            thread = self._thread
            self._server = None
            self._thread = None
        if server is not None:
            server.shutdown()
            server.server_close()
            if thread is not None:
                thread.join(timeout=1.0)
            if os.path.lexists(self._socket_path):
                os.unlink(self._socket_path)


class _Handler(socketserver.StreamRequestHandler):
    def handle(self):
        try:
            data = self.rfile.readline(MAX_LINE + 1)
            if not data or len(data) > MAX_LINE or not data.endswith(b'\n'):
                raise BridgeError('Malformed/oversized command')
            result = self.server.receiver.handle(json.loads(data.decode('utf-8')))
            reply = {'protocol': PROTOCOL, 'ok': True, **result}
        except Exception as exc:
            reply = {'protocol': PROTOCOL, 'ok': False, 'error': str(exc)}
        self.wfile.write(json.dumps(reply, allow_nan=False).encode() + b'\n')


class _Server(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True
    allow_reuse_address = False
