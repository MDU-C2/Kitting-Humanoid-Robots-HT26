#!/usr/bin/env python3
"""READ-ONLY measured-arm selector probe; NEVER publishes to physical G1.

Checks hypothetical *stationary* trajectory against real G1 LowState and
exercises the existing XR selector's accept/hold logic without commanding it.
No ROS ActionServer, Unitree SDK, DDS participant or actuator publisher.
"""
import argparse
import json
import math
import sys
import time

from g1_hardware_preflight import fetch_snapshot
from g1_readonly_observer import SOCKET
from xr_hardware_feedback import FeedbackError, validate_lowstate_snapshot
from xr_measured_arm_probe import ReadOnlyMeasuredArmProbe


def run_probe(fetch, *, duration_s=3.0, interval_s=0.1,
              clock=time.monotonic, sleep=time.sleep):
    if (not math.isfinite(duration_s) or duration_s <= 0
            or not math.isfinite(interval_s) or interval_s <= 0
            or duration_s < 2 * interval_s):
        raise FeedbackError('Invalid probe duration or interval')
    probe = ReadOnlyMeasuredArmProbe()
    started = clock()
    sample_count = 0
    first_receipt = None
    status = None
    while True:
        snapshot = fetch()
        now = clock()
        if status is None:
            # First target is *exactly* measured q, so no physical movement
            # is requested or even possible by this independent process.
            measured = validate_lowstate_snapshot(snapshot, now=now)
            target = measured.q.copy()
            first_receipt = measured.estimated_receipt_at
            status = probe.preview_start(snapshot, target, now=now)
        else:
            status = probe.preview_path(snapshot, target, now=now)
        sample_count += 1
        if now - started >= duration_s:
            break
        sleep(interval_s)
    if (sample_count < 3 or probe.progress_count < 2
            or probe._receipt_at - first_receipt < duration_s * 0.5):
        raise FeedbackError('Real LowState receipts did not progress during probe')
    return {'samples': sample_count,
            'receipt_progress_s': float(probe._receipt_at - first_receipt),
            'validated_arm_joints': len(status.arm_positions),
            'hypothetical_selected_source': status.source,
            'max_tracking_error_rad': status.max_tracking_error_rad,
            'read_only': True, 'execution_enabled': False,
            'physical_execution': False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--socket', default=SOCKET)
    parser.add_argument('--duration', type=float, default=3.0)
    parser.add_argument('--watch', action='store_true')
    args = parser.parse_args(argv)
    try:
        # Read-only observer may need startup time. Do not start XR or motors.
        deadline = time.monotonic() + 5.0
        while True:
            try:
                validate_lowstate_snapshot(fetch_snapshot(args.socket), now=time.monotonic())
                break
            except (FeedbackError, OSError, ValueError, TypeError):
                if time.monotonic() >= deadline:
                    raise FeedbackError('No valid real LowState within 5 seconds')
                time.sleep(0.1)
        while True:
            summary = run_probe(lambda: fetch_snapshot(args.socket), duration_s=args.duration)
            print('G1 ARM SELECTOR READ-ONLY PROBE PASS: ' + json.dumps(summary), flush=True)
            print('NO ACTUATION: selector takeover and tracking are hypothetical only.', flush=True)
            if not args.watch:
                return 0
    except KeyboardInterrupt:
        return 0
    except (FeedbackError, OSError, ValueError, TypeError) as exc:
        print('G1 ARM SELECTOR READ-ONLY PROBE FAILED: ' + str(exc), file=sys.stderr, flush=True)
        return 2


if __name__ == '__main__':
    sys.exit(main())
