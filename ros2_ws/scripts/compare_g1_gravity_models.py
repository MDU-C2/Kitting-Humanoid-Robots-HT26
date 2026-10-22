#!/usr/bin/env python3
"""Read-only comparison: XR gravity model vs installed Inspire FTP URDF.

Run INSIDE existing G1 container with the XR Python environment:
  /opt/mamba/envs/xr/bin/python /workspace/scripts/compare_g1_gravity_models.py
or, with an already running physical XR endpoint, to read measured pose:
  /opt/mamba/envs/xr/bin/python /workspace/scripts/compare_g1_gravity_models.py --live

NO DDS initialization, no ROS publisher, no motor commands, no gain changes.
Both URDF models are reduced with *all non-arm joints fixed at neutral*,
using the same approximation as XR. No version-sensitive .pkl is loaded. Predicted torque differences are not measured motor
forces, nor proof of the true mass distribution of the fitted hands.
"""
import argparse
import json
import socket
import sys
from pathlib import Path

import numpy as np
import pinocchio as pin

NAMES = (
    'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint',
    'left_elbow_joint', 'left_wrist_roll_joint', 'left_wrist_pitch_joint',
    'left_wrist_yaw_joint', 'right_shoulder_pitch_joint', 'right_shoulder_roll_joint',
    'right_shoulder_yaw_joint', 'right_elbow_joint', 'right_wrist_roll_joint',
    'right_wrist_pitch_joint', 'right_wrist_yaw_joint',
)
KP = np.array([80, 80, 80, 80, 40, 40, 40] * 2, dtype=float)


def read_xr_status(socket_path):
    payload = (json.dumps({'protocol': 1, 'kind': 'status'}) + '\n').encode()
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.settimeout(2.0)
        sock.connect(str(socket_path))
        sock.sendall(payload)
        buf = b''
        while not buf.endswith(b'\n'):
            part = sock.recv(65536)
            if not part or len(buf) > 131072:
                raise RuntimeError('XR status response closed/truncated')
            buf += part
    msg = json.loads(buf)
    if msg.get('ok') is not True or msg.get('kind') != 'status_ack':
        raise RuntimeError('XR status refused: ' + str(msg.get('error', msg)))
    if msg.get('feedback_fresh') is not True:
        raise RuntimeError('XR feedback is not fresh')
    target = np.asarray(msg['selected_arm_target'], dtype=float)
    actual = np.asarray(msg['measured_arm_positions'], dtype=float)
    dq = np.asarray(msg['measured_arm_velocities'], dtype=float)
    for key, a in [('target', target), ('actual', actual), ('speed', dq)]:
        if a.shape != (14,) or not np.isfinite(a).all():
            raise RuntimeError(f'XR {key} invalid')
    return msg, target, actual, dq


def verify_14(model, label):
    names = tuple(model.names[1:])
    if model.nq != 14 or model.nv != 14 or names != NAMES:
        raise RuntimeError(
            f'{label} reduced model does not match the 14-arm order.\n'
            f'  nq={model.nq}, nv={model.nv}\n  joints={names}\n'
            'No comparison is safe until this is resolved.')


def reduced_model_from_urdf(path, label):
    """Rebuild from URDF; never deserialize a version-sensitive Pinocchio cache."""
    complete = pin.buildModelFromUrdf(str(path))
    keep = set(NAMES)
    for n in keep:
        if not complete.existJointName(n):
            raise RuntimeError(f'{label} URDF lacks required arm joint: {n}')
    lock_ids = [i for i in range(1, complete.njoints)
                if complete.names[i] not in keep]
    # Exactly as XR's reduced robot: all non-arm joints fixed at 0.
    neutral_zero = pin.neutral(complete)
    reduced = pin.buildReducedModel(complete, lock_ids, neutral_zero)
    verify_14(reduced, label)
    return reduced


def grav(model, q):
    return np.asarray(pin.computeGeneralizedGravity(model, model.createData(), q), dtype=float)


def run():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--live', action='store_true', help='Read the current XR measured/target pose from the existing Unix socket (never actuates)')
    p.add_argument('--socket', default='/tmp/g1_xr_headless_moveit.sock')
    p.add_argument('--workspace', default='/workspace')
    args = p.parse_args()
    ws = Path(args.workspace)
    xr_urdf = ws / 'src/xr_teleoperate/assets/g1/g1_body29_hand14.urdf'
    ftp_urdf = ws / 'src/g1_description/urdf/g1_29dof_with_inspire_hand_ftp.urdf'
    if not xr_urdf.is_file() or not ftp_urdf.is_file():
        p.error(f'Missing required URDF files: {xr_urdf}, {ftp_urdf}. Run inside the project container.')

    print('Building reduced gravity models directly from URDF (no cached pickle)...', flush=True)
    try:
        xr = reduced_model_from_urdf(xr_urdf, 'XR')
    except Exception as exc:
        raise RuntimeError(f'XR URDF model build failed: {exc}') from exc
    try:
        ftp = reduced_model_from_urdf(ftp_urdf, 'FTP')
    except Exception as exc:
        raise RuntimeError(f'FTP URDF model build failed: {exc}') from exc
    if args.live:
        status, target, actual, dq = read_xr_status(args.socket)
        print('Read-only XR status: source=%s, mode=%s, max |dq|=%.4f rad/s' %
              (status.get('source'), status.get('mode'), np.max(np.abs(dq))))
        if np.max(np.abs(dq)) > 0.12:
            print('WARNING: arms are moving. Interpret this as a dynamic snapshot, not a steady-state result.')
    else:
        target = np.zeros(14)
        actual = target.copy()
        print('OFFLINE: neutral pose q=0 for all 14 arm joints. No robot connection.')

    xr_tau_target = grav(xr, target)
    ftp_tau_actual = grav(ftp, actual)
    ftp_tau_target = grav(ftp, target)
    pd_tau = KP * (target - actual)  # predicted P term, not measured effort

    print('\nXR model rebuilt from original URDF: ' + str(xr_urdf))
    print('The reference FTP model is: ' + str(ftp_urdf))
    print('NOTE: a running XR process may use a differently serialized cached model.\n'
          'This comparison describes the two supplied URDFs, not an independently verified runtime cache.')
    print('All non-arm joints held at zero (same simplifying assumption as XR).')
    print('\n%-28s %8s %8s %9s %9s %9s %9s' %
          ('joint', 'q_cmd', 'q_meas', 'XR_ff', 'FTP_g', 'FTP-XR', 'Kp*err'))
    print(' ' * 48 + '(torques in Nm, model-based)')
    for i, name in enumerate(NAMES):
        print('%-28s %+8.4f %+8.4f %+9.3f %+9.3f %+9.3f %+9.3f' %
              (name.replace('_joint', ''), target[i], actual[i], xr_tau_target[i],
               ftp_tau_actual[i], ftp_tau_actual[i] - xr_tau_target[i], pd_tau[i]))
    model_delta = ftp_tau_target - xr_tau_target
    j = int(np.argmax(abs(model_delta)))
    print('\nLargest model-only gravity difference at commanded pose: %s: %+.3f Nm' %
          (NAMES[j], model_delta[j]))
    print('Largest resulting first-order position-equivalent error |delta_tau/Kp|: %.4f rad (%.2f deg)' %
          (np.max(abs(model_delta / KP)), np.degrees(np.max(abs(model_delta / KP)))))
    print('\nINTERPRETATION: FTP-XR is the extra gravity torque that the FTP model predicts.')
    print('Kp*err is the *predicted* P effort, not an SDK/motor measured torque.')
    print('Agreement in sign and magnitude supports (but cannot prove) gravity-model mismatch.')
    print('This test does NOT change torque feedforward, gains, motor commands, or XR source.')


if __name__ == '__main__':
    try:
        run()
    except Exception as exc:
        print('ERROR:', exc, file=sys.stderr)
        sys.exit(1)
