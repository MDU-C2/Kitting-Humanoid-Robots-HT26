"""G1 read-only LowState observer. No command publishers, no motor writes.

Explicit robot-facing mode: only subscribe to Unitree DDS rt/lowstate on the
operator-specified interface. Exposes a local Unix socket for ROS joint states.
This does NOT claim exclusive command ownership elsewhere on the network.
"""
import argparse
import json
import math
import os
import socketserver
import threading
import time
from pathlib import Path

from g1_state_contract import JOINT_NAMES_29

SOCKET = '/tmp/g1_xr_readonly_state.sock'


class StateCache:
    def __init__(self):
        self.lock = threading.Lock()
        self.last_at = None
        self.positions = None
        self.velocities = None
        self.mode_machine = None

    def update(self, msg):
        q = [float(msg.motor_state[i].q) for i in range(29)]
        dq = [float(msg.motor_state[i].dq) for i in range(29)]
        if not all(math.isfinite(v) for v in q + dq):
            return  # never forward an invalid measurement
        with self.lock:
            self.positions, self.velocities = q, dq
            self.mode_machine = int(msg.mode_machine)
            self.last_at = time.monotonic()

    def snapshot(self):
        with self.lock:
            age = None if self.last_at is None else time.monotonic() - self.last_at
            return {'kind': 'state', 'mode': 'observe', 'hardware_connected': True,
                    'read_only': True, 'source': 'lowstate',
                    'fresh': age is not None and 0 <= age <= 0.25,
                    'age_s': age, 'joint_names': JOINT_NAMES_29,
                    'positions': self.positions, 'velocities': self.velocities,
                    'mode_machine': self.mode_machine}


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        try:
            msg = self.rfile.readline(1025)
            if len(msg) > 1024 or not msg.endswith(b'\n'):
                return
            if json.loads(msg) != {'kind': 'status'}:
                return
            self.wfile.write((json.dumps(self.server.cache.snapshot(), allow_nan=False) + '\n').encode())
        except (ValueError, OSError):
            return


class UnixServer(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True


def main(argv=None):
    parser = argparse.ArgumentParser(description='READ-ONLY G1 LowState DDS observer')
    parser.add_argument('--interface', required=True, help='Explicit Unitree-facing NIC, e.g. eth0')
    parser.add_argument('--socket', default=SOCKET)
    opts = parser.parse_args(argv)
    if not opts.interface or '/' in opts.interface or opts.interface == 'lo':
        parser.error('Specify actual robot-connected DDS interface (not loopback)')
    if not Path('/sys/class/net', opts.interface).exists():
        parser.error(f'Network interface does not exist: {opts.interface}')
    if os.path.lexists(opts.socket):
        parser.error(f'Socket already exists (possible other owner): {opts.socket}')

    # Deliberately do not import original XR controller or any ChannelPublisher.
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as HgLowState
    ChannelFactoryInitialize(0, networkInterface=opts.interface)
    subscriber = ChannelSubscriber('rt/lowstate', HgLowState)
    subscriber.Init()
    cache = StateCache()
    server = UnixServer(opts.socket, Handler)
    server.cache = cache
    os.chmod(opts.socket, 0o600)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f'READ-ONLY G1 rt/lowstate on {opts.interface}; zero command publishers', flush=True)
    try:
        while True:
            data = subscriber.Read()
            if data is not None:
                cache.update(data)
            time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=1)
        os.unlink(opts.socket)


if __name__ == '__main__':
    main()
