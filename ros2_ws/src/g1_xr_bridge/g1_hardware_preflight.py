#!/usr/bin/env python3
"""Read-only physical G1 feedback preflight; does NOT control any actuators.

Polls the existing g1_readonly_observer Unix socket. Checks new LowState
receipts continue arriving and that 29-body/14-arm measured feedback is valid.
It does NOT authenticate the sending robot, detect all competing DDS command
publishers, confirm physical safeguards, or authorize XR motion mode.
"""
import argparse
import json
import math
import socket
import sys
import time

from g1_readonly_observer import SOCKET
from xr_hardware_feedback import FeedbackError, validate_lowstate_snapshot


def fetch_snapshot(socket_path=SOCKET, timeout=0.3):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(timeout)
        client.connect(socket_path)
        client.sendall(b'{"kind":"status"}\n')
        with client.makefile('rb') as stream:
            line = stream.readline(16385)
    if not line or len(line) > 16384 or not line.endswith(b'\n'):
        raise FeedbackError('Read-only observer returned no complete reply')
    try:
        return json.loads(line)
    except (UnicodeError, ValueError) as exc:
        raise FeedbackError('Read-only observer returned invalid JSON') from exc


def verify_stream(fetch, *, duration_s=3.0, interval_s=0.1,
                  max_age_s=0.25, clock=time.monotonic, sleep=time.sleep):
    """Measure freshness and progress; fail if snapshots merely repeat.

    Receipt timestamp is reconstructed from the observer's monotonic age.
    Observer and checker run locally in the same container/monotonic clock.
    """
    if (not math.isfinite(duration_s) or duration_s <= 0
            or not math.isfinite(interval_s) or interval_s <= 0
            or duration_s < interval_s * 2):
        raise FeedbackError('Invalid preflight sampling duration or interval')
    begin = clock()
    first_receipt = None
    latest_receipt = None
    count = 0
    mode = None
    while True:
        snapshot = fetch()
        now = clock()
        measured = validate_lowstate_snapshot(snapshot, now=now, max_age_s=max_age_s)
        if first_receipt is None:
            first_receipt = measured.estimated_receipt_at
        # Require no backward jump beyond sub-ms scheduling/IPC jitter.
        if (latest_receipt is not None
                and measured.estimated_receipt_at + 0.003 < latest_receipt):
            raise FeedbackError('LowState receipt time moved backwards')
        latest_receipt = max(measured.estimated_receipt_at,
                             latest_receipt if latest_receipt is not None else float('-inf'))
        mode = measured.mode_machine
        count += 1
        if now - begin >= duration_s:
            break
        sleep(interval_s)
    if count < 3 or latest_receipt - first_receipt < duration_s * 0.5:
        raise FeedbackError('LowState receipts did not advance throughout preflight')
    return {'samples': count, 'receipt_progress_s': latest_receipt - first_receipt,
            'mode_machine': mode, 'validated_arm_joints': 14,
            'read_only': True, 'execution_enabled': False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--socket', default=SOCKET)
    parser.add_argument('--duration', type=float, default=3.0)
    parser.add_argument('--watch', action='store_true', help='Keep checking; terminate pipeline on stale feedback')
    args = parser.parse_args(argv)
    try:
        if args.watch:
            # Observer and Unitree DDS can need a short startup interval.
            # This retry occurs only before the first successful read;
            # subsequent stale/disconnected feedback terminates monitoring.
            deadline = time.monotonic() + 5.0
            while True:
                try:
                    validate_lowstate_snapshot(fetch_snapshot(args.socket), now=time.monotonic())
                    break
                except (FeedbackError, OSError, TypeError, ValueError):
                    if time.monotonic() >= deadline:
                        raise FeedbackError('No valid LowState from observer within 5 seconds')
                    time.sleep(0.1)
        first = True
        while True:
            summary = verify_stream(lambda: fetch_snapshot(args.socket), duration_s=args.duration)
            print('G1 READ-ONLY PREFLIGHT PASS: ' + json.dumps(summary), flush=True)
            if first:
                print('HARDWARE COMMANDS: BLOCKED. No motor publisher or trajectory action created.', flush=True)
                print('NOT VERIFIED: unique DDS command ownership, identity, FTP hand state, robot stop, or tracking.', flush=True)
                first = False
            if not args.watch:
                return 0
    except KeyboardInterrupt:
        return 0
    except (FeedbackError, OSError, TypeError, ValueError) as exc:
        print(f'G1 READ-ONLY PREFLIGHT FAILED: {exc}', file=sys.stderr, flush=True)
        return 2


if __name__ == '__main__':
    sys.exit(main())
