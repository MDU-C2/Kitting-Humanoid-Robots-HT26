#!/usr/bin/env python3
"""Read-only Unitree G1 XR vs Inspire FTP gravity model inspection.

No publishers, DDS, motor writes, robot motion or gain modifications.
Offline: checks URDF Pinocchio gravity calculations and selected example poses.
--live: reads existing XR controller's Unix IPC status ONLY, checks live
        shoulder commands, and compares the outgoing LowCmd feedforward with
        both models. Controller's outgoing buffer is NOT actuator torque feedback.

Run from inside existing Docker with the XR Micromamba Python and PYTHONPATH unset.
"""
import argparse
import json
import os
from pathlib import Path
import socket
import sys
import time

import numpy as np
import pinocchio as pin

NAMES = (
    'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint',
    'left_elbow_joint', 'left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint',
    'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint',
    'right_elbow_joint', 'right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint',
)
SHOULDERS = (0, 7)


def build(urdf: Path, label: str):
    if not urdf.is_file():
        raise RuntimeError(f'{label} model not found: {urdf}')
    full = pin.buildModelFromUrdf(str(urdf))
    missing = [n for n in NAMES if not full.existJointName(n)]
    if missing:
        raise RuntimeError(f'{label}: arm joints missing: {missing}')
    lock = [j for j in range(1, full.njoints) if full.names[j] not in NAMES]
    model = pin.buildReducedModel(full, lock, pin.neutral(full))
    if model.nq != 14 or model.nv != 14 or tuple(model.names[1:]) != NAMES:
        raise RuntimeError(f'{label}: reduced joint order is not the expected 14-arm motor order: {model.names}')
    return model


def g(model, q):
    q = np.asarray(q, dtype=float)
    if q.shape != (14,) or not np.isfinite(q).all():
        raise RuntimeError('Invalid 14-arm position vector')
    tau = np.asarray(pin.computeGeneralizedGravity(model, model.createData(), q), dtype=float)
    if tau.shape != (14,) or not np.isfinite(tau).all():
        raise RuntimeError('Nonfinite / invalid model gravity torque')
    return tau


def status(socket_path: str):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as s:
        s.settimeout(2.0)
        s.connect(socket_path)
        s.sendall((json.dumps({'protocol': 1, 'kind': 'status'}) + '\n').encode())
        response = bytearray()
        while not response.endswith(b'\n'):
            chunk = s.recv(65536)
            if not chunk or len(response) + len(chunk) > 100000:
                raise RuntimeError('Truncated / oversized XR status')
            response += chunk
    out = json.loads(response)
    if out.get('ok') is not True or out.get('kind') != 'status_ack' or out.get('feedback_fresh') is not True:
        raise RuntimeError('XR status is not a fresh successful status_ack: ' + str(out))
    return out


def vector(msg, key, required=True):
    value = msg.get(key)
    if value is None and not required:
        return None
    v = np.asarray(value, dtype=float)
    if v.shape != (14,) or not np.isfinite(v).all():
        raise RuntimeError(f'XR status field {key} missing/invalid 14-vector')
    return v


def fmt(a):
    return f'{a:+8.3f}'


def compare_at_pose(xr, ftp, q, label):
    original, inspire = g(xr, q), g(ftp, q)
    print(f'\n{label}')
    print('                     XR_Nm    FTP_Nm   FTP-XR_Nm')
    for i in (0, 3, 7, 10):
        print(f'{NAMES[i]:28s} {fmt(original[i])} {fmt(inspire[i])} {fmt(inspire[i]-original[i])}')


def live_report(xr, ftp, sample, sample_number):
    q_cmd = vector(sample, 'selected_arm_target')
    q = vector(sample, 'measured_arm_positions')
    dq = vector(sample, 'measured_arm_velocities')
    lowcmd = vector(sample, 'xr_lowcmd_tau', required=False)
    kp = vector(sample, 'xr_lowcmd_kp', required=False)
    lowcmd_q = vector(sample, 'xr_lowcmd_q', required=False)
    xr_tau = g(xr, q_cmd)
    ftp_tau = g(ftp, q_cmd)
    print(f'\nLIVE snapshot {sample_number}: source={sample.get("source")}, mode={sample.get("mode")}, '
          f'max_arm_speed={max(abs(dq)):.4f} rad/s')
    print('                        goal     actual    error    XR_g_Nm    FTP_g_Nm   lowcmd_ff  Kp*error')
    for i in SHOULDERS:
        feed = f'{lowcmd[i]:+10.3f}' if lowcmd is not None else '    (n/a) '
        pd = f'{kp[i]*(q_cmd[i]-q[i]):+10.3f}' if kp is not None else '    (n/a) '
        print(f'{NAMES[i]:28s} {q_cmd[i]:+8.4f} {q[i]:+8.4f} {q_cmd[i]-q[i]:+8.4f} '
              f'{xr_tau[i]:+10.3f} {ftp_tau[i]:+10.3f} {feed} {pd}')
        if lowcmd is not None:
            d_ftp = abs(float(lowcmd[i] - ftp_tau[i]))
            d_xr = abs(float(lowcmd[i] - xr_tau[i]))
            print(f'   |LowCmd-FTP|={d_ftp:.3f} Nm; |LowCmd-XR|={d_xr:.3f} Nm'
                  + ('  (models hard to distinguish at this pose)' if abs(xr_tau[i]-ftp_tau[i]) < 0.2 else ''))
    if lowcmd is None:
        print('No outgoing LowCmd torque telemetry in IPC status; check XR diagnostic version.')
    if lowcmd_q is not None:
        print(f'Outgoing LowCmd q vs selected target max deviation: {max(abs(lowcmd_q - q_cmd)):.5f} rad')
    if max(abs(dq)) > 0.03:
        print('CAUTION: joint speeds are not near zero; torque comparison reflects a moving snapshot.')
    print('Outgoing LowCmd tau is requested gravity feedforward, NOT verified motor-produced torque.')
    print('Dynamic status may be captured between 50-Hz command cycles: small mismatches are normal.')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--workspace', default='/workspace', help='ROS workspace root INSIDE Docker')
    p.add_argument('--live', action='store_true', help='Read existing XR socket; NEVER actuate')
    p.add_argument('--socket', default='/tmp/g1_xr_headless_moveit.sock')
    p.add_argument('--samples', type=int, default=1, help='Read-only live snapshots (1..20)')
    args = p.parse_args()
    if not 1 <= args.samples <= 20:
        p.error('--samples must be 1..20')
    ws = Path(args.workspace)
    xr_urdf = ws / 'src/xr_teleoperate/assets/g1/g1_body29_hand14.urdf'
    ftp_urdf = ws / 'src/g1_description/urdf/g1_29dof_with_inspire_hand_ftp.urdf'
    print('READ ONLY: no XR publisher started, no robot commands sent.')
    print(f'Original XR URDF: {xr_urdf}')
    print(f'Inspire FTP URDF: {ftp_urdf}')
    xr, ftp = build(xr_urdf, 'Original XR'), build(ftp_urdf, 'Inspire FTP')
    print('PASS: both models load, have 14 arm DOFs, and match motor joint order.')
    zeros = np.zeros(14)
    compare_at_pose(xr, ftp, zeros, 'Neutral arms (all arm q=0)')
    if args.live:
        for i in range(args.samples):
            live_report(xr, ftp, status(args.socket), i+1)
            if i + 1 < args.samples:
                time.sleep(0.5)
    else:
        print('\nOFFLINE MODEL-ONLY CHECK; no measured loads. Pass --live while XR runs for read-only LowCmd comparison.')
    print('\nLIMITATION: model comparison cannot determine the true mass/COM of installed hands or prove gravity accuracy.')
    print('The active XR model must also be confirmed from the current xr.log GRAVITY MODEL SELECTED message.')


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        sys.exit(1)
