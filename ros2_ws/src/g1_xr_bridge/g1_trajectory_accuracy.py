#!/usr/bin/env python3
"""Analyze G1 /arm_sdk versus /lowstate from g1_arm_sdk_elbow_recorder.py.

Offline-only: Python standard library; no ROS, DDS, or motor commands.
The measured quantity is tracking of the outgoing /arm_sdk position command,
not exact tracking of the original MoveIt planned trajectory.
"""
import argparse
import csv
import math
from pathlib import Path
from statistics import mean, median


def read_samples(path):
    rows = []
    with open(path, newline='', encoding='utf-8') as file:
        reader = csv.DictReader(file)
        expected = {'elapsed_s', 'joint', 'command_q_rad', 'measured_q_rad', 'weight'}
        missing = expected - set(reader.fieldnames or ())
        if missing:
            raise ValueError('Missing CSV columns: ' + ', '.join(sorted(missing)))
        for line, record in enumerate(reader, 2):
            try:
                t, cmd, actual, weight = (float(record[k]) for k in
                                          ('elapsed_s', 'command_q_rad',
                                           'measured_q_rad', 'weight'))
                if not all(map(math.isfinite, (t, cmd, actual, weight))):
                    raise ValueError('non-finite value')
                rows.append((t, cmd, actual, weight))
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(f'CSV row {line}: {exc}') from exc
    if len(rows) < 5:
        raise ValueError('Not enough valid samples (need at least 5)')
    if any(b[0] <= a[0] for a, b in zip(rows, rows[1:])):
        raise ValueError('Elapsed time must strictly increase')
    return rows


def windows(rows, threshold, baseline_window_s, min_duration_s):
    t0 = rows[0][0]
    baseline_rows = [r for r in rows if r[0] - t0 <= baseline_window_s]
    if len(baseline_rows) < 3:
        baseline_rows = rows[:min(10, len(rows))]
    baseline = median([r[1] for r in baseline_rows])
    active = [abs(row[1] - baseline) >= threshold for row in rows]
    groups = []
    start = None
    for i, flag in enumerate(active + [False]):
        if flag and start is None:
            start = i
        if not flag and start is not None:
            end = i - 1
            if rows[end][0] - rows[start][0] >= min_duration_s:
                groups.append((start, end))
            start = None
    return baseline, groups


def summarize(rows, baseline, groups):
    cmd = [x[1] for x in rows]
    actual = [x[2] for x in rows]
    errors = [c - q for c, q in zip(cmd, actual)]
    dt = median([b[0] - a[0] for a, b in zip(rows, rows[1:])])
    print(f'Samples: {len(rows)} | duration: {rows[-1][0]-rows[0][0]:.2f} s | median period: {dt:.3f} s')
    print(f'Initial command (median): {baseline:+.5f} rad')
    print(f'Command range: {max(cmd)-min(cmd):.5f} rad')
    print(f'Measured range: {max(actual)-min(actual):.5f} rad')
    print(f'Peak absolute command-vs-feedback error: {max(map(abs,errors)):.5f} rad')
    print(f'Overall RMS error: {(mean(e*e for e in errors)**0.5):.5f} rad')
    print(f'Final recorded command: {cmd[-1]:+.5f} rad')
    print(f'Final recorded measured: {actual[-1]:+.5f} rad')
    print(f'Final recorded error: {errors[-1]:+.5f} rad')
    print(f'Control weight range: {min(r[3] for r in rows):.3f}..{max(r[3] for r in rows):.3f}')
    if not groups:
        print('No movement window detected; try --threshold 0.0005 or capture an entire movement.')
        return
    print(f'\nDetected command excursions: {len(groups)}')
    for number, (first, last) in enumerate(groups, 1):
        section = rows[first:last + 1]
        section_errors = [r[1] - r[2] for r in section]
        start_cmd = section[0][1]
        max_excursion = max(section, key=lambda r: abs(r[1] - baseline))
        print(f'  Excursion {number}: t={section[0][0]:.2f}..{section[-1][0]:.2f}s')
        print(f'    Command excursion from initial: {max_excursion[1]-baseline:+.5f} rad at t={max_excursion[0]:.2f}s')
        print(f'    Measured position at command peak: {max_excursion[2]:+.5f} rad')
        print(f'    Instantaneous error at command peak: {max_excursion[1]-max_excursion[2]:+.5f} rad')
        print(f'    Max absolute error during excursion: {max(map(abs,section_errors)):.5f} rad')
        print(f'    RMS error during excursion: {mean(e*e for e in section_errors)**0.5:.5f} rad')
        if first > 0:
            before = rows[max(0, first - min(10, first)):first]
            q0 = median([r[2] for r in before])
            measured_excursion = max((abs(r[2]-q0) for r in section), default=0)
            print(f'    Largest measured excursion from pre-event baseline: {measured_excursion:.5f} rad')
    print('\nInterpretation: Data shows tracking of /arm_sdk position commands, not the MoveIt goal.')
    print('If the command returns to HOLD during the recording, final error may be small even if the goal was missed.')
    print('Joint load, controller authority, torque compensation, and any timeouts require separate validation.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', nargs='?', default='/tmp/g1_elbow_sdk_trace.csv',
                        help='CSV produced by g1_arm_sdk_elbow_recorder.py')
    parser.add_argument('--threshold', type=float, default=0.001,
                        help='Movement detection threshold in radians (default: 0.001)')
    parser.add_argument('--baseline-seconds', type=float, default=2.0)
    parser.add_argument('--min-excursion-seconds', type=float, default=0.1)
    args = parser.parse_args()
    if (args.threshold <= 0 or args.baseline_seconds <= 0 or
            args.min_excursion_seconds < 0):
        parser.error('Threshold and baseline time must be positive; min duration non-negative')
    if not Path(args.csv).is_file():
        parser.error(f'No CSV at {args.csv!r}. Copy the existing trace there or pass its path.')
    rows = read_samples(args.csv)
    baseline, groups = windows(rows, args.threshold, args.baseline_seconds,
                               args.min_excursion_seconds)
    summarize(rows, baseline, groups)

if __name__ == '__main__':
    main()
