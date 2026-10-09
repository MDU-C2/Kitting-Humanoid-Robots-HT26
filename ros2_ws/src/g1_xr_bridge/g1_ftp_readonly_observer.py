#!/usr/bin/env python3
"""Subscribe ONLY to real Inspire FTP hand-state DDS; no command publishers.

Requires inspire_sdkpy in the XR Micromamba environment. Does not import the
original XR controller, write DDS commands, or publish ROS /joint_states.
"""
import argparse
import json
import os
import signal
import socketserver
import stat
import threading
import time
from pathlib import Path

from g1_ftp_state_contract import FTP_NAMES, FTP_SOCKET, FtpFeedbackError, normalize_angle_act


class FtpStateCache:
    def __init__(self):
        self.lock = threading.Lock()
        self.states = {'left': None, 'right': None}
        self.receipts = {'left': None, 'right': None}
        # Diagnostics are passive; never alter DDS subscriptions or control paths.
        self.received = {'left': 0, 'right': 0}
        self.rejected = {'left': 0, 'right': 0}
        self.last_received = {'left': None, 'right': None}
        self.last_rejection = {'left': None, 'right': None}

    def update(self, hand, msg, *, now=None):
        if hand not in ('left', 'right'):
            raise ValueError('Unknown hand')
        if now is None:
            now = time.monotonic()
        try:
            data = normalize_angle_act(list(msg.angle_act))
            reason = None
        except (AttributeError, TypeError, ValueError, FtpFeedbackError) as exc:
            data = None
            reason = f'{type(exc).__name__}: {str(exc)[:120]}'
        with self.lock:
            self.received[hand] += 1
            self.last_received[hand] = now
            if data is None:
                self.rejected[hand] += 1
                self.last_rejection[hand] = reason
                return False
            self.states[hand] = data
            self.receipts[hand] = now
        return True

    def snapshot(self, *, now=None):
        if now is None:
            now = time.monotonic()
        with self.lock:
            a = {h: None if self.receipts[h] is None else now - self.receipts[h]
                 for h in ('left', 'right')}
            both = all(age is not None and 0 <= age <= 0.25 for age in a.values())
            # Do not expose old measured values as valid when even one hand is stale.
            data = (list(self.states['left']) + list(self.states['right'])
                    if both else None)
            diagnostics = {
                'received_messages': dict(self.received),
                'rejected_messages': dict(self.rejected),
                'last_rejection': dict(self.last_rejection),
                'last_message_age_s': {
                    h: None if self.last_received[h] is None
                    else now - self.last_received[h]
                    for h in ('left', 'right')
                }
            }
        return {'kind': 'ftp_state', 'source': 'inspire_ftp_state',
                'hardware_connected': True, 'read_only': True,
                'both_fresh': both, 'actuator_names': list(FTP_NAMES),
                'normalized_angle_act': data,
                'left_age_s': a['left'], 'right_age_s': a['right'],
                'units': 'fraction_of_full_range_not_radians',
                'diagnostics': diagnostics}


class Handler(socketserver.StreamRequestHandler):
    def handle(self):
        try:
            request = self.rfile.readline(1025)
            if len(request) > 1024 or not request.endswith(b'\n'):
                return
            if json.loads(request) != {'kind': 'ftp_status'}:
                return
            reply = self.server.cache.snapshot()
            self.wfile.write((json.dumps(reply, allow_nan=False) + '\n').encode())
        except (ValueError, OSError):
            return


class UnixServer(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', required=True)
    parser.add_argument('--socket', default=FTP_SOCKET)
    opts = parser.parse_args(argv)
    if not opts.interface or '/' in opts.interface or opts.interface == 'lo':
        parser.error('Specify the real robot-facing network interface')
    if not (Path('/sys/class/net') / opts.interface).exists():
        parser.error('Network interface not found: ' + opts.interface)
    if os.path.lexists(opts.socket):
        parser.error('FTP observer socket already exists; refusing to take ownership')

    # Fail clearly if the optional hand-state SDK is absent. No change to XR.
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from inspire_sdkpy import inspire_dds
    except ImportError as exc:
        parser.error('Missing read-only FTP SDK dependency: ' + str(exc))

    ChannelFactoryInitialize(0, networkInterface=opts.interface)
    cache = FtpStateCache()
    left = ChannelSubscriber('rt/inspire_hand/state/l', inspire_dds.inspire_hand_state)
    right = ChannelSubscriber('rt/inspire_hand/state/r', inspire_dds.inspire_hand_state)
    # The SDK's synchronous Read() can block indefinitely with no incoming data.
    # Use its read-only callback API instead so SIGINT/SIGTERM can stop promptly.
    # Do not create a DDS command writer or initialize the original XR hand controller.
    stop_event = threading.Event()

    def request_stop(_signum, _frame):
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    server = None
    thread = None
    socket_inode = None
    try:
        left.Init(lambda msg: cache.update('left', msg))
        right.Init(lambda msg: cache.update('right', msg))

        # Refuse to replace an existing IPC endpoint, even when stopping.
        server = UnixServer(opts.socket, Handler)
        server.cache = cache
        socket_inode = os.stat(opts.socket).st_ino
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        print(f'READ-ONLY Inspire FTP state on {opts.interface}; '
              'NO HAND COMMAND PUBLISHERS', flush=True)
        while not stop_event.wait(0.1):
            pass
    except KeyboardInterrupt:
        # Also handle an interrupt delivered outside the registered handler.
        stop_event.set()
    finally:
        stop_event.set()
        if server is not None:
            if thread is not None and thread.is_alive():
                server.shutdown()
            server.server_close()
        if thread is not None:
            thread.join(timeout=1)
        # Close DDS readers only; there are no motor/hand command writers.
        try:
            left.Close()
        finally:
            right.Close()
        # Never unlink a different process's replacement endpoint.
        if socket_inode is not None:
            try:
                entry = os.lstat(opts.socket)
            except FileNotFoundError:
                pass
            else:
                if stat.S_ISSOCK(entry.st_mode) and entry.st_ino == socket_inode:
                    os.unlink(opts.socket)


if __name__ == '__main__':
    main()
