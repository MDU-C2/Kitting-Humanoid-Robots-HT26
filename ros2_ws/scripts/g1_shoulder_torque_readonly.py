#!/usr/bin/env python3
"""Observe existing G1 XR LowCmd and read-only G1 LowState, NO DDS or actuation.

Requires the read-only observer update shipped with this script. Reads two local
Unix sockets: XR control status (commanded q, ff, kp), observer LowState
(measured q,dq,tau_est). Does NOT publish, set gains, or send samples.

`tau_est` is a firmware/vendor-reported estimate, NOT a calibrated measurement
of motor output torque. We do not assert that it equals LowCmd tau + Kp error.

Save CSV for comparing shoulder pitch/roll during supervised, previously
validated motions. When testing different directions, use the same joint goal,
not merely the same Cartesian Z coordinate.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import socket
import sys
import time

XR_SOCKET = '/tmp/g1_xr_headless_moveit.sock'
OBSERVER_SOCKET = '/tmp/g1_xr_readonly_state.sock'
# indices in Unitree G1 29-motor LowState (NOT zero-based 14-arm order)
JOINTS = (
    ('left_shoulder_pitch_joint', 15, 0),
    ('left_shoulder_roll_joint', 16, 1),
    ('right_shoulder_pitch_joint', 22, 7),
    ('right_shoulder_roll_joint', 23, 8),
)


def read_json_socket(path, request, timeout=0.45):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.settimeout(timeout)
        sock.connect(path)
        sock.sendall((json.dumps(request) + '\n').encode('ascii'))
        result = bytearray()
        while not result.endswith(b'\n'):
            chunk = sock.recv(16384)
            if not chunk or len(result) + len(chunk) > 120000:
                raise ValueError('Incomplete or oversized socket reply')
            result.extend(chunk)
    return json.loads(result)


def numeric_array(obj, key, count, optional=False):
    val = obj.get(key)
    if optional and val is None:
        return None
    if (not isinstance(val, list) or len(val) != count or
            any(isinstance(x, bool) or not isinstance(x, (float, int)) or
                not math.isfinite(x) for x in val)):
        raise ValueError(f'Invalid {key}: expected {count} finite numeric values')
    return val


def acquire(xr_socket=XR_SOCKET, observer_socket=OBSERVER_SOCKET):
    xr = read_json_socket(xr_socket, {'protocol': 1, 'kind': 'status'})
    obs = read_json_socket(observer_socket, {'kind': 'status'})
    if not (xr.get('ok') is True and xr.get('kind') == 'status_ack' and
            xr.get('mode') == 'hardware' and xr.get('feedback_fresh') is True):
        raise ValueError('XR hardware status is missing or stale; refusing to report')
    if not (obs.get('kind') == 'state' and obs.get('read_only') is True and
            obs.get('fresh') is True and obs.get('source') == 'lowstate'):
        raise ValueError('Read-only G1 LowState observer status missing or stale')
    q_target = numeric_array(xr, 'selected_arm_target', 14)
    q_xr = numeric_array(xr, 'measured_arm_positions', 14)
    lowcmd_q = numeric_array(xr, 'xr_lowcmd_q', 14)
    lowcmd_ff = numeric_array(xr, 'xr_lowcmd_tau', 14)
    lowcmd_kp = numeric_array(xr, 'xr_lowcmd_kp', 14)
    q_obs = numeric_array(obs, 'positions', 29)
    dq_obs = numeric_array(obs, 'velocities', 29)
    tau_est = numeric_array(obs, 'efforts', 29)
    names = obs.get('joint_names')
    if not isinstance(names, list) or len(names) != 29:
        raise ValueError('Invalid G1 observer joint mapping')
    rows = []
    for name, j, i in JOINTS:
        if names[j] != name:
            raise ValueError(f'Unexpected G1 observer joint at index {j}: {names[j]}')
        rows.append({
            'wall_time_s': round(time.time(), 4),
            'source': xr.get('source', ''),
            'joint': name,
            'target_rad': q_target[i],
            'lowcmd_q_rad': lowcmd_q[i],
            'actual_rad': q_obs[j],
            'error_rad': lowcmd_q[i] - q_obs[j],
            'velocity_rad_s': dq_obs[j],
            'tau_ff_cmd_Nm': lowcmd_ff[i],
            'kp_cmd': lowcmd_kp[i],
            'kp_error_Nm': lowcmd_kp[i] * (lowcmd_q[i] - q_obs[j]),
            'nominal_ff_plus_p_Nm': lowcmd_ff[i] + lowcmd_kp[i] * (lowcmd_q[i] - q_obs[j]),
            'motor_tau_est_Nm': tau_est[j],
            'xr_observer_q_diff_rad': q_xr[i] - q_obs[j],
        })
    return rows


FIELDS = ('wall_time_s', 'source', 'joint', 'target_rad', 'lowcmd_q_rad',
          'actual_rad', 'error_rad', 'velocity_rad_s', 'tau_ff_cmd_Nm',
          'kp_cmd', 'kp_error_Nm', 'nominal_ff_plus_p_Nm',
          'motor_tau_est_Nm', 'xr_observer_q_diff_rad')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--xr-socket', default=XR_SOCKET)
    parser.add_argument('--observer-socket', default=OBSERVER_SOCKET)
    parser.add_argument('--samples', type=int, default=5)
    parser.add_argument('--interval', type=float, default=0.5,
                        help='Seconds between read-only snapshots, 0.05–5')
    parser.add_argument('--csv', type=Path, help='Optional CSV output; will not overwrite existing files')
    args = parser.parse_args(argv)
    if not 1 <= args.samples <= 500 or not 0.05 <= args.interval <= 5:
        parser.error('Samples must be 1..500 and interval 0.05..5 seconds')
    print('READ ONLY: both sockets are status-only. No new robot DDS connection or motor commands.')
    print('tau_est is a vendor estimate; commanded feedforward and estimated motor torque are not directly interchangeable.')
    if args.csv is not None and args.csv.exists():
        parser.error(f'Refusing to overwrite CSV: {args.csv}')
    writer, csvfile = None, None
    try:
        if args.csv is not None:
            csvfile = args.csv.open('x', newline='', encoding='utf-8')
            writer = csv.DictWriter(csvfile, fieldnames=FIELDS)
            writer.writeheader()
        for n in range(args.samples):
            start = time.monotonic()
            rows = acquire(args.xr_socket, args.observer_socket)
            print(f'\nSnapshot {n+1}/{args.samples}: source={rows[0]["source"]}')
            print(f'{"joint":27s}  {"error(rad)":>11s} {"dq(rad/s)":>10s} {"FF_cmd":>9s} {"Kp*err":>9s} {"FF+P":>9s} {"tau_est":>9s} {"XR-obs":>10s}')
            for row in rows:
                print(f'{row["joint"]:27s} {row["error_rad"]:+11.5f} {row["velocity_rad_s"]:+10.4f} '
                      f'{row["tau_ff_cmd_Nm"]:+9.3f} {row["kp_error_Nm"]:+9.3f} '
                      f'{row["nominal_ff_plus_p_Nm"]:+9.3f} {row["motor_tau_est_Nm"]:+9.3f} '
                      f'{row["xr_observer_q_diff_rad"]:+10.5f}')
                if writer:
                    writer.writerow(row)
            if csvfile:
                csvfile.flush()
            if n + 1 < args.samples:
                time.sleep(max(0, args.interval - (time.monotonic()-start)))
    finally:
        if csvfile is not None:
            csvfile.close()
    print('\nFF+P omits damping; not a predicted measured torque. Observe sign, direction and saturation patterns.')
    if args.csv is not None:
        print(f'CSV: {args.csv}')


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, RuntimeError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        print('Ensure XR AND the read-only observer are running in the SAME Docker container.', file=sys.stderr)
        sys.exit(1)
