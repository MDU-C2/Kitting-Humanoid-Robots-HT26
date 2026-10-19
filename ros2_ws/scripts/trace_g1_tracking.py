#!/usr/bin/env python3
"""Read-only 14-arm XR tracking logger for supervised G1 diagnostics.

Polls only the existing headless XR IPC `status` request. Never imports the
Unitree SDK, opens DDS, acquires motor ownership, or sends a trajectory.

The XR published command is a snapshot of the latest q computed by its
250 Hz publisher; it is not an acknowledgement from the robot's motor drive.
"""
import argparse
import csv
from datetime import datetime
import math
from pathlib import Path
import sys
import time

BRIDGE = Path(__file__).resolve().parents[1] / 'src' / 'g1_xr_bridge'
sys.path.insert(0, str(BRIDGE))
from xr_arm_trajectory import ARM_JOINT_NAMES  # noqa: E402
from xr_ipc_protocol import transact  # noqa: E402

SOCKET = '/tmp/g1_xr_headless_moveit.sock'
VECTOR_FIELDS = (
    ('selected_arm_target', 'desired'),
    ('xr_limited_command_q', 'limited_command'),
    ('measured_arm_positions', 'measured'),
    ('measured_arm_velocities', 'measured_velocity'),
)
HEADER = [
    'wall_time_unix_s', 'sample_monotonic_s', 'source', 'mode_machine',
    'xr_speed_limit_rad_s', 'xr_lead_limit_rad',
    'max_desired_command_error_rad', 'max_command_measured_error_rad',
    'max_desired_measured_error_rad',
] + [f'{label}_{joint}' for _, label in VECTOR_FIELDS for joint in ARM_JOINT_NAMES]


def validate_vector(value, field):
    if not isinstance(value, (list, tuple)) or len(value) != 14:
        raise ValueError(f'{field}: expected 14 joint values')
    try:
        values = [float(v) for v in value]
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{field}: nonnumeric joint values') from exc
    if not all(math.isfinite(v) for v in values):
        raise ValueError(f'{field}: nonfinite joint values')
    return values


def status_to_row(status, wall_time):
    if (status.get('kind') != 'status_ack' or
            status.get('hardware_connected') is not True or
            status.get('feedback_fresh') is not True or
            status.get('xr_loop_fresh') is not True):
        raise ValueError('XR physical controller not ready or feedback stale')
    vectors = [validate_vector(status.get(key), key) for key, _ in VECTOR_FIELDS]
    desired, command, measured, _ = vectors
    speed = float(status['xr_headless_speed_limit_rad_s'])
    lead = float(status['xr_headless_max_lead_rad'])
    # G1_MOVEIT_DIRECT_TRACE_V1: lead=0 means the headless clamp is disabled;
    # speed reflects the YAML limit, not an additional XR headless limiter.
    if not (math.isfinite(speed) and speed > 0 and math.isfinite(lead) and lead >= 0):
        raise ValueError('Invalid XR motion-limit diagnostics')
    monotonic_s = float(status['status_monotonic_s'])
    if not math.isfinite(monotonic_s):
        raise ValueError('Invalid status timestamp')
    max_dc = max(abs(a-b) for a, b in zip(desired, command))
    max_cm = max(abs(a-b) for a, b in zip(command, measured))
    max_dm = max(abs(a-b) for a, b in zip(desired, measured))
    return [
        wall_time, monotonic_s, status.get('source', ''), status.get('mode_machine', ''),
        speed, lead, max_dc, max_cm, max_dm,
        *(value for vec in vectors for value in vec),
    ]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=None,
                        help='CSV output file (default: /workspace/g1_headless_logs/diagnostics/<timestamp>.csv)')
    parser.add_argument('--rate-hz', type=float, default=20.0,
                        help='Read-only status samples per second, 1..50 (default: 20)')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='Seconds to record; 0 means until Ctrl+C')
    args = parser.parse_args(argv)
    if not (math.isfinite(args.rate_hz) and 1 <= args.rate_hz <= 50):
        parser.error('--rate-hz must be between 1 and 50')
    if not (math.isfinite(args.duration) and args.duration >= 0):
        parser.error('--duration must be >= 0')

    output = args.output or (Path('/workspace/g1_headless_logs/diagnostics') /
                              ('xr_tracking_' + datetime.now().strftime('%Y%m%d_%H%M%S') + '.csv'))
    # Verify a healthy physical controller before creating an output file.
    try:
        first = transact({'kind': 'status'}, SOCKET, timeout=0.6)
        first_row = status_to_row(first, time.time())
    except Exception as exc:
        print(f'No healthy physical XR status endpoint: {exc}', file=sys.stderr)
        return 1

    output.parent.mkdir(parents=True, exist_ok=True)
    interval = 1.0 / args.rate_hz
    started = time.monotonic()
    next_sample = started
    rows = 0
    try:
        with output.open('w', newline='', buffering=131072) as stream:
            writer = csv.writer(stream)
            writer.writerow(HEADER)
            writer.writerow(first_row)
            rows += 1
            print(f'Read-only XR trajectory diagnostics: {output}', flush=True)
            print('Waiting for MoveIt trajectories. Press Ctrl+C to finish recording.', flush=True)
            last_report = started
            while args.duration == 0 or time.monotonic() - started < args.duration:
                next_sample += interval
                remaining = next_sample - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
                else:
                    next_sample = time.monotonic()
                # Read-only IPC; never sends a `sample` or activates a publisher.
                status = transact({'kind': 'status'}, SOCKET, timeout=0.6)
                row = status_to_row(status, time.time())
                writer.writerow(row)
                rows += 1
                if time.monotonic() - last_report >= 1.0:
                    stream.flush()
                    last_report = time.monotonic()
                    print(f'{row[2]:>10} | desired-command {row[6]:.3f} rad | '
                          f'command-measured {row[7]:.3f} rad | '
                          f'desired-measured {row[8]:.3f} rad', flush=True)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'Tracking stopped: {exc}', file=sys.stderr)
        print(f'Partial CSV retained: {output} ({rows} rows)', file=sys.stderr)
        return 1
    print(f'Saved {rows} read-only samples to {output}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
