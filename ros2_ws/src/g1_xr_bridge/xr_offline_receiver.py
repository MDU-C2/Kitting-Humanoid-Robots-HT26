"""OFFLINE XR-side IPC receiver using original G1_29_ArmIK Pinocchio model.

This script NEVER imports or constructs G1_29_ArmController or initialises DDS.
It only computes the same gravity RNEA formula used in XR solve_ik.
Run with the isolated XR Micromamba interpreter, NOT system ROS Python.
"""
import argparse
import json
import os
import socketserver
import sys
import threading
import time
from pathlib import Path

from xr_ipc_protocol import MAX_LINE, PROTOCOL, SOCKET_PATH, _vector14


class GravityReceiver:
    def __init__(self):
        import numpy as np
        import pinocchio as pin
        xr_root = Path('/workspace/src/xr_teleoperate')
        if not (xr_root / 'teleop/robot_control/robot_arm_ik.py').exists():
            raise RuntimeError(f'XR repository not found: {xr_root}')
        sys.path.insert(0, str(xr_root))
        # The pinned XR class uses URDF paths relative to its teleop directory.
        os.chdir(xr_root / 'teleop')
        from teleop.robot_control.robot_arm_ik import G1_29_ArmIK
        print('Initializing ORIGINAL XR IK model (offline; no DDS or robot)...', flush=True)
        ik = G1_29_ArmIK(Unit_Test=False, Visualization=False)
        from xr_arm_trajectory import ARM_JOINT_NAMES
        model = ik.reduced_robot.model
        if model.nq != 14 or model.nv != 14 or tuple(model.names[1:]) != ARM_JOINT_NAMES:
            raise RuntimeError('XR model does not match the expected 14 arm joints')
        self.pin = pin
        self.np = np
        self.model = model
        self.data = ik.reduced_robot.data
        self.lock = threading.Lock()
        self.count = 0
        self.last_log = 0.0
        print('Original XR Pinocchio model loaded: 14 DoF; OFFLINE RECEIVER READY', flush=True)

    def handle(self, request):
        if not isinstance(request, dict) or request.get('protocol') != PROTOCOL:
            raise ValueError('Invalid request/protocol')
        kind = request.get('kind')
        if kind == 'ping':
            return {'kind': 'pong', 'hardware_connected': False}
        if kind != 'sample':
            raise ValueError('Unknown request kind')
        source = request.get('source')
        seq = request.get('seq')
        if source not in ('trajectory', 'hold') or type(seq) is not int or seq < 0:
            raise ValueError('Invalid sample source/sequence')
        q = _vector14(request.get('q'))
        with self.lock:
            zero = self.np.zeros(14)
            # This is the ORIGINAL XR RNEA formula with velocity/acceleration=0.
            tau = self.pin.rnea(self.model, self.data, self.np.asarray(q), zero, zero).copy()
            self.count += 1
            now = time.monotonic()
            if now - self.last_log >= 0.5 or source == 'hold' and self.count == 1:
                print(f'XR OFFLINE seq={seq}, mode={source}, left_elbow={q[3]:+.4f} rad, '
                      f'XR gravity tau={tau[3]:+.4f} Nm; NO ACTUATION', flush=True)
                self.last_log = now
        return {'kind': 'sample_ack', 'source': source, 'seq': seq,
                'tau_ff': tau.tolist(), 'hardware_connected': False}


class RequestHandler(socketserver.StreamRequestHandler):
    def handle(self):
        try:
            line = self.rfile.readline(MAX_LINE + 1)
            if not line or len(line) > MAX_LINE or not line.endswith(b'\n'):
                raise ValueError('Malformed/oversized request')
            request = json.loads(line.decode('utf-8'))
            result = self.server.receiver.handle(request)
            response = {'protocol': PROTOCOL, 'ok': True, **result}
        except Exception as exc:
            response = {'protocol': PROTOCOL, 'ok': False, 'error': str(exc)}
        self.wfile.write(json.dumps(response, allow_nan=False).encode() + b'\n')


class LocalServer(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True
    allow_reuse_address = True


def run(path=SOCKET_PATH, receiver=None):
    receiver = receiver or GravityReceiver()
    # Use /tmp only; do not unlink an active listener's socket.
    if os.path.exists(path):
        raise RuntimeError(f'{path} already exists. Stop the old receiver first, then remove stale socket if necessary.')
    server = LocalServer(path, RequestHandler)
    server.receiver = receiver
    os.chmod(path, 0o600)
    try:
        print(f'Listening on local UNIX socket {path} (offline only)', flush=True)
        server.serve_forever(poll_interval=0.1)
    finally:
        server.server_close()
        if os.path.exists(path):
            os.unlink(path)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='XR gravity receiver, NO hardware')
    parser.add_argument('--socket', default=SOCKET_PATH)
    args = parser.parse_args()
    run(args.socket)
