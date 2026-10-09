#!/usr/bin/env python3
"""Monitor fresh real FTP measurements; fail if either hand stops updating.

No Unitree SDK import and no DDS/ROS publishers.
"""
import argparse
import json
import socket
import sys
import time

from g1_ftp_state_contract import FTP_SOCKET, FtpFeedbackError, validate_ftp_snapshot


def fetch_snapshot(socket_path=FTP_SOCKET, timeout=0.3):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(timeout)
        client.connect(socket_path)
        client.sendall(b'{"kind":"ftp_status"}\n')
        with client.makefile('rb') as stream:
            reply = stream.readline(16385)
    if not reply or len(reply) > 16384 or not reply.endswith(b'\n'):
        raise FtpFeedbackError('FTP observer returned no complete response')
    try:
        return json.loads(reply)
    except (ValueError, UnicodeError) as exc:
        raise FtpFeedbackError('Invalid FTP observer JSON response') from exc


def verify_stream(fetch, *, duration_s=3.0, interval_s=0.1,
                  clock=time.monotonic, sleep=time.sleep):
    if duration_s <= 0 or interval_s <= 0 or duration_s < interval_s * 2:
        raise FtpFeedbackError('Invalid FTP preflight duration')
    started = clock()
    beginning = None
    last = None
    count = 0
    while True:
        measured = validate_ftp_snapshot(fetch(), now=clock())
        current = (measured.left_received_at, measured.right_received_at)
        if beginning is None:
            beginning = current
        if last is not None and any(t + 0.003 < p for t, p in zip(current, last)):
            raise FtpFeedbackError('FTP receipt timestamp moved backwards')
        last = tuple(max(t, p) for t, p in zip(current, last or current))
        count += 1
        if clock() - started >= duration_s:
            break
        sleep(interval_s)
    progress = [last[i] - beginning[i] for i in range(2)]
    if count < 3 or any(p < duration_s * 0.5 for p in progress):
        raise FtpFeedbackError('FTP hand state stream stalled')
    return {'samples': count, 'left_progress_s': progress[0],
            'right_progress_s': progress[1], 'actuators': 12,
            'units': 'normalized_not_radians', 'read_only': True,
            'execution_enabled': False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--socket', default=FTP_SOCKET)
    parser.add_argument('--watch', action='store_true')
    opts = parser.parse_args(argv)
    try:
        # Startup may take a few seconds to discover both DDS hand publishers.
        deadline = time.monotonic() + 8.0
        while True:
            try:
                validate_ftp_snapshot(fetch_snapshot(opts.socket), now=time.monotonic())
                break
            except (OSError, ValueError):
                if time.monotonic() >= deadline:
                    raise FtpFeedbackError('No fresh LEFT+RIGHT FTP state within 8 seconds')
                time.sleep(0.1)
        while True:
            summary = verify_stream(lambda: fetch_snapshot(opts.socket))
            print('G1 FTP READ-ONLY PREFLIGHT PASS: ' + json.dumps(summary), flush=True)
            if not opts.watch:
                return 0
    except KeyboardInterrupt:
        return 0
    except (OSError, ValueError) as exc:
        print('G1 FTP READ-ONLY PREFLIGHT FAILED: ' + str(exc), file=sys.stderr, flush=True)
        return 2


if __name__ == '__main__':
    sys.exit(main())
