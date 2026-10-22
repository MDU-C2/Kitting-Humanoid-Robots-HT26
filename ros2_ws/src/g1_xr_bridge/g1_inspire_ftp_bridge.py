#!/usr/bin/env python3
"""Explicitly isolated Inspire FTP hand feedback and operator-authorized micro-step controller.

No XR arm code changed. --enable-motion is opt-in and requires its own ACK.
Start *only* with the headless MoveIt XR pipeline, not alongside VR hand control.
"""
import argparse
import json
import math
import os
import signal
import socket
import socketserver
import secrets
import stat
import threading
import time
from pathlib import Path

from g1_ftp_hand_math import SOCKET, MOTORS, normalize_raw, urdf_positions, changed_motor_state
from g1_ftp_hand_trajectory_math import URDF_LIMITS, joint_names

ARM_SOCKET = '/tmp/g1_xr_headless_moveit.sock'


class Cache:
    def __init__(self):
        self.lock = threading.RLock()
        self.raw = {'left': None, 'right': None}
        self.at = {'left': None, 'right': None}
        self.motion_enabled = False
        self.cmd = {}
        self.publisher = {}
        self.default_module = None
        # G1_INSPIRE_FTP_MOVEIT_TRAJECTORY_V1: one owner per hand, fails closed after timeout.
        self.trajectory_lease = {'left': None, 'right': None}
        self.last_step = {'left': 0.0, 'right': 0.0}
        self.last_trajectory_cmd = {'left': None, 'right': None}

    def update(self, side, msg):
        values = getattr(msg, 'angle_act', None)
        try:
            values = list(values)
            normalized = normalize_raw(values)
        except (TypeError, ValueError):
            return
        with self.lock:
            self.raw[side] = normalized
            self.at[side] = time.monotonic()

    def arm_is_active(self):
        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
                s.settimeout(0.3)
                s.connect(ARM_SOCKET)
                s.sendall(b'{"protocol":1,"kind":"status"}\n')
                r = s.makefile('rb').readline(16384)
            v = json.loads(r)
            return (v.get('kind') == 'status_ack' and v.get('hardware_connected') is True
                    and v.get('mode') == 'hardware' and v.get('xr_loop_fresh') is True
                    and v.get('feedback_fresh') is True and v.get('weight_ramp_complete') is True
                    and v.get('source') == 'hold')
        except (OSError, ValueError, TypeError):
            return False

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            hands = {}
            for side in ('left', 'right'):
                age = None if self.at[side] is None else now - self.at[side]
                fresh = age is not None and 0 <= age <= 0.25
                entry = {'fresh': fresh, 'age_s': age}
                if fresh:
                    names, q = urdf_positions(side, self.raw[side])
                    entry.update(normalized=self.raw[side][:], joint_names=names, joint_positions=q)
                hands[side] = entry
            return {'kind': 'inspire_ftp_state', 'read_only': not self.motion_enabled,
                    'source': 'inspire_ftp_angle_act', 'hands': hands}

    def step(self, side, motor, direction, amount):
        if not self.motion_enabled:
            raise ValueError('Hand motion disabled; start explicit supervised mode')
        if side not in ('left', 'right'):
            raise ValueError('Invalid hand side')
        if not self.arm_is_active():
            raise RuntimeError('XR headless controller must be healthy and in HOLD (not trajectory)')
        with self.lock:
            self._expire_lease(side)
            if self.trajectory_lease[side] is not None:
                raise RuntimeError('MoveIt trajectory owns this hand; CLI step refused')
            age = None if self.at[side] is None else time.monotonic() - self.at[side]
            if age is None or age > 0.20:
                raise RuntimeError(f'{side} Inspire FTP feedback stale')
            desired = changed_motor_state(self.raw[side], motor, direction, amount)
            # Hardware receives a full 6-motor target for exactly one hand.
            # The other 5 motors are commanded at their current measured values.
            msg = self.default_module.get_inspire_hand_ctrl()
            msg.angle_set = [int(round(v * 1000)) for v in desired]
            msg.mode = 0b0001
            if self.publisher[side].Write(msg) is False:
                raise RuntimeError('Hand DDS publish rejected')
            self.cmd[side] = (time.monotonic(), desired)
            self.last_step[side] = time.monotonic()
        return {'kind': 'step_ack', 'side': side, 'motor': motor,
                'direction': direction, 'requested_normalized': desired}


    def _expire_lease(self, side):
        lease = self.trajectory_lease[side]
        if lease is not None and time.monotonic() > lease['expires']:
            self.trajectory_lease[side] = None

    def trajectory_claim(self, side):
        if side not in ('left','right'):
            raise ValueError('Invalid hand side')
        if not self.motion_enabled:
            raise RuntimeError('Physical hand motion not authorized')
        if not self.arm_is_active():
            raise RuntimeError('MoveIt arm must be healthy and in HOLD before hand movement')
        with self.lock:
            self._expire_lease(side)
            if self.trajectory_lease[side] is not None:
                raise RuntimeError('Hand already owned by trajectory executor')
            if time.monotonic()-self.last_step[side] < 0.5:
                raise RuntimeError('CLI hand step cooldown; cannot claim hand')
            age = None if self.at[side] is None else time.monotonic()-self.at[side]
            if age is None or age > 0.20:
                raise RuntimeError('Hand feedback stale; refusing trajectory')
            token = secrets.token_hex(16)
            self.trajectory_lease[side] = {'token':token,'expires':time.monotonic()+2.0}
            self.last_trajectory_cmd[side] = (time.monotonic(), list(self.raw[side]))
        return {'kind':'trajectory_claim_ack','side':side,'token':token}

    def trajectory_target(self, side, token, normalized):
        if side not in ('left','right'):
            raise ValueError('Invalid hand side')
        if not self.motion_enabled or not self.arm_is_active():
            raise RuntimeError('Arm not safely in HOLD or hand motion disabled')
        if not isinstance(normalized,list) or len(normalized) != 6 or not all(
                type(v) in (int,float) and math.isfinite(v) and 0 <= v <= 1 for v in normalized):
            raise ValueError('Invalid six-channel trajectory command')
        with self.lock:
            self._expire_lease(side)
            lease = self.trajectory_lease[side]
            if lease is None or token != lease['token']:
                raise RuntimeError('Hand trajectory lease missing or expired')
            age = None if self.at[side] is None else time.monotonic() - self.at[side]
            if age is None or age > 0.20:
                raise RuntimeError('Hand feedback stale; no motor commands sent')
            # G1_FTP_NATIVE_URDF_MOTION_LIMITS_V1: no custom normalized step cap. Ensure the
            # issued commands nevertheless respect each joint's source URDF
            # velocity limit even if a ROS callback executes late.
            now = time.monotonic()
            last_t, last_target = self.last_trajectory_cmd[side]
            elapsed = now - last_t
            for name, previous, desired in zip(joint_names(side), last_target, normalized):
                maxrad, urdf_velocity = URDF_LIMITS[name][1:]
                if abs(desired - previous) * maxrad > urdf_velocity * elapsed + 0.001:
                    raise RuntimeError('Hand command would exceed URDF velocity: '+name)
            msg = self.default_module.get_inspire_hand_ctrl()
            msg.angle_set = [int(round(v*1000)) for v in normalized]
            msg.mode = 0b0001
            if self.publisher[side].Write(msg) is False:
                raise RuntimeError('Hand DDS trajectory publish rejected')
            lease['expires'] = time.monotonic() + 2.0
            self.last_trajectory_cmd[side] = (time.monotonic(), list(normalized))
            return {'kind':'trajectory_target_ack','side':side}

    def trajectory_release(self, side, token):
        if side not in ('left','right'):
            raise ValueError('Invalid hand side')
        with self.lock:
            self._expire_lease(side)
            lease = self.trajectory_lease[side]
            if lease is None:
                return {'kind':'trajectory_release_ack','side':side,'already_released':True}
            if lease['token'] != token:
                raise RuntimeError('Cannot release another hand owner')
            self.trajectory_lease[side] = None
            self.last_trajectory_cmd[side] = None
        return {'kind':'trajectory_release_ack','side':side}


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        try:
            line = self.rfile.readline(4097)
            if not line.endswith(b'\n') or len(line) > 4096:
                return
            cmd = json.loads(line)
            if not isinstance(cmd, dict):
                return
            if cmd == {'kind': 'status'}:
                result = self.server.cache.snapshot()
            elif set(cmd) == {'kind', 'side', 'motor', 'direction', 'amount', 'ack'} and cmd['kind'] == 'step' and cmd['ack'] == 'G1_INSPIRE_FTP_MICROSTEP_SUPERVISED':
                result = self.server.cache.step(cmd['side'], cmd['motor'], cmd['direction'], cmd['amount'])
            elif set(cmd) == {'kind','side','ack'} and cmd['kind'] == 'trajectory_claim' and cmd['ack'] == 'G1_INSPIRE_FTP_TRAJECTORY_SUPERVISED':
                result = self.server.cache.trajectory_claim(cmd['side'])
            elif set(cmd) == {'kind','side','token','normalized','ack'} and cmd['kind'] == 'trajectory_target' and cmd['ack'] == 'G1_INSPIRE_FTP_TRAJECTORY_SUPERVISED':
                result = self.server.cache.trajectory_target(cmd['side'],cmd['token'],cmd['normalized'])
            elif set(cmd) == {'kind','side','token'} and cmd['kind'] == 'trajectory_release':
                result = self.server.cache.trajectory_release(cmd['side'],cmd['token'])
            else:
                raise ValueError('Unsupported hand request')
            self.wfile.write((json.dumps({'ok': True, **result}, allow_nan=False) + '\n').encode())
        except (ValueError, TypeError, OSError, RuntimeError) as exc:
            try:
                self.wfile.write((json.dumps({'ok': False, 'error': str(exc)}) + '\n').encode())
            except OSError:
                pass


class Server(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True


def main():
    parser = argparse.ArgumentParser(description='Inspire FTP state and optional supervised hand microstep')
    parser.add_argument('--interface', required=True)
    parser.add_argument('--enable-motion', action='store_true')
    parser.add_argument('--operator-ack', default='')
    args = parser.parse_args()
    if (not args.interface or args.interface == 'lo' or '/' in args.interface or
            not Path('/sys/class/net', args.interface).exists()):
        parser.error('G1 interface must be present and explicitly selected')
    if args.enable_motion and args.operator_ack != 'G1_INSPIRE_FTP_SUPERVISED':
        parser.error('Hand command publisher requires G1_INSPIRE_FTP_SUPERVISED')
    if os.path.lexists(SOCKET):
        parser.error(f'Socket {SOCKET} already exists; never steal another hand controller')
    # Reuse exactly the Unitree topics and IDL classes in the original XR FTP controller.
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber, ChannelPublisher
    from inspire_sdkpy import inspire_dds
    import inspire_sdkpy.inspire_hand_defaut as defaults
    ChannelFactoryInitialize(0, networkInterface=args.interface)
    state_topic = {'left': 'rt/inspire_hand/state/l', 'right': 'rt/inspire_hand/state/r'}
    cmd_topic = {'left': 'rt/inspire_hand/ctrl/l', 'right': 'rt/inspire_hand/ctrl/r'}
    cache = Cache()
    cache.motion_enabled = args.enable_motion
    cache.default_module = defaults
    subscribers = {}
    for side in ('left', 'right'):
        sub = ChannelSubscriber(state_topic[side], inspire_dds.inspire_hand_state)
        sub.Init()
        subscribers[side] = sub
    if args.enable_motion:
        for side in ('left', 'right'):
            pub = ChannelPublisher(cmd_topic[side], inspire_dds.inspire_hand_ctrl)
            pub.Init()
            cache.publisher[side] = pub
    terminating = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: terminating.set())
    signal.signal(signal.SIGINT, lambda *_: terminating.set())
    server = None
    server_thread = None
    inode = None
    try:
        server = Server(SOCKET, Handler)
        inode = os.lstat(SOCKET)
        os.chmod(SOCKET, 0o600)
        server.cache = cache
        server_thread = threading.Thread(target=server.serve_forever, daemon=True)
        server_thread.start()
        print('INSPIRE FTP: ' + ('SUPERVISED MICROSTEP enabled' if args.enable_motion else 'READ-ONLY state only') + f'; interface={args.interface}', flush=True)
        last_health_at = time.monotonic()
        while not terminating.wait(0.005):
            for side, sub in subscribers.items():
                msg = sub.Read()
                if msg is not None:
                    cache.update(side, msg)
            # If motor publishing was enabled, stop hand control process when arm controller exits.
            now = time.monotonic()
            if args.enable_motion and now - last_health_at >= 0.5:
                last_health_at = now
                # During a MoveIt arm trajectory status isn't HOLD; do not exit just for that.
                try:
                    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
                        s.settimeout(0.2)
                        s.connect(ARM_SOCKET)
                        s.sendall(b'{"protocol":1,"kind":"status"}\n')
                        reply = json.loads(s.makefile('rb').readline(16384))
                    healthy = reply.get('hardware_connected') is True and reply.get('xr_loop_fresh') is True
                except (OSError, ValueError, TypeError):
                    healthy = False
                if not healthy:
                    raise RuntimeError('XR headless controller stopped; closing Inspire FTP command publishers')
    finally:
        if server is not None:
            server.shutdown()
            server.server_close()
            if server_thread is not None:
                server_thread.join(timeout=1)
        if inode is not None:
            try:
                cur = os.lstat(SOCKET)
                if (stat.S_ISSOCK(cur.st_mode) and
                        (cur.st_dev, cur.st_ino) == (inode.st_dev, inode.st_ino)):
                    os.unlink(SOCKET)
            except FileNotFoundError:
                pass
        print('Inspire FTP bridge stopped; hand state/commands unavailable', flush=True)


if __name__ == '__main__':
    main()
