#!/usr/bin/env python3
"""Operator-guided, one-at-a-time Inspire FTP motor mapping validation.

Uses the existing g1_inspire_ftp_bridge local Unix socket only. This tool never
creates DDS publishers and never starts the robot. Motor commands require the
bridge's explicit supervised motion mode and healthy arm HOLD state.
"""
import json
import math
import socket
import sys
import time
from pathlib import Path

SOCKET = '/tmp/g1_xr_inspire_ftp.sock'
MOTORS = ('little', 'ring', 'middle', 'index', 'thumb_bend', 'thumb_rotation')
ACK = 'G1_INSPIRE_FTP_MICROSTEP_SUPERVISED'
AMOUNT = 0.02


def request(payload):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.settimeout(2)
        sock.connect(SOCKET)
        sock.sendall((json.dumps(payload) + '\n').encode())
        line = sock.makefile('rb').readline(16384)
    if not line:
        raise RuntimeError('Hand bridge returned no response')
    result = json.loads(line)
    if not result.get('ok'):
        raise RuntimeError(str(result.get('error', result)))
    return result


def measured(side):
    hands = request({'kind': 'status'})['hands']
    data = hands[side]
    if not data.get('fresh'):
        raise RuntimeError(f'{side} hand feedback is STALE; do not command')
    values = data.get('normalized')
    if not isinstance(values, list) or len(values) != 6 or not all(
        isinstance(v, (int, float)) and math.isfinite(v) and 0 <= v <= 1
        for v in values
    ):
        raise RuntimeError('Unexpected motor feedback format')
    return values


def ask(prompt):
    try:
        return input(prompt).strip().lower()
    except (EOFError, KeyboardInterrupt):
        print('\nStopped without sending another command.')
        raise SystemExit(0)


def main():
    print('G1 INSPIRE FTP — PHYSICAL MOTOR MAPPING (operator guided)')
    print('One 2% microstep per motor, ONLY after explicit confirmation.')
    print('Do not run with VR or another hand command publisher active.')
    print('Requires supervised G1, clear fingers, arms in HOLD, tested independent stop.')
    print('No autonomous sweeps; no collision/force validation.\n')
    if ask('Type READY to begin (anything else exits): ') != 'ready':
        return
    for side in ('right', 'left'):
        for index, motor in enumerate(MOTORS):
            print(f'\n--- {side.upper()} {motor} (motor channel {index}) ---')
            try:
                before = measured(side)
            except (RuntimeError, OSError, ValueError) as exc:
                print('STOP:', exc)
                return
            print('Measured before:', [round(v, 3) for v in before])
            action = ask('Choose [c]lose 2%, [o]pen 2%, [s]kip, or [q]uit: ')
            if action == 'q':
                return
            if action not in ('c', 'o'):
                print('Skipped.')
                continue
            direction = 'close' if action == 'c' else 'open'
            if ((direction == 'close' and before[index] < AMOUNT) or
                (direction == 'open' and before[index] > 1 - AMOUNT)):
                print('Too near the selected endpoint; choose opposite direction or skip.')
                continue
            phrase = f'{side} {motor} {direction}'
            if ask(f'Physically verify hand is clear; type exactly "{phrase}" to send ONE command: ') != phrase:
                print('Not confirmed. Skipped.')
                continue
            try:
                result = request({'kind': 'step', 'side': side, 'motor': motor,
                                  'direction': direction, 'amount': AMOUNT, 'ack': ACK})
                print('Bridge accepted request:', result['requested_normalized'])
                time.sleep(0.7)
                after = measured(side)
            except (RuntimeError, OSError, ValueError) as exc:
                print('STOP:', exc)
                return
            delta = [round(b-a, 4) for a, b in zip(before, after)]
            print('Measured after: ', [round(v, 3) for v in after])
            print('Measured change:', delta)
            print('Expected motor index:', index, 'expected sign:', '+' if direction == 'open' else '-')
            print('NOTE: changed values are feedback, not proof of correct physical finger.')
            answer = ask('Did ONLY the intended physical finger move in the intended direction? [y/n/q]: ')
            print(f'RESULT {side} {motor}: {answer}; feedback_delta={delta}', flush=True)
            if answer != 'y':
                print('STOP. Investigate channel mapping/unwanted movement before further commands.')
                return
    print('\nAll selected motor steps completed with operator confirmation.')
    print('URDF angle calibration, collision geometry, force and MoveIt trajectory control remain unverified.')


if __name__ == '__main__':
    main()
