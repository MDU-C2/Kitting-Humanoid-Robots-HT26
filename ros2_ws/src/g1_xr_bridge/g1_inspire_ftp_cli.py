#!/usr/bin/env python3
"""Inspire FTP read-only status or one small operator-authorized motor step.

Examples:
  python3 g1_inspire_ftp_cli.py status
  python3 g1_inspire_ftp_cli.py step right index close --amount 0.02 --ack G1_INSPIRE_FTP_MICROSTEP_SUPERVISED
No DDS initialization happens here; all requests go to our local bridge.
"""
import argparse
import json
import socket

from g1_ftp_hand_math import SOCKET, MOTORS


def transact(cmd):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.settimeout(1.0)
        sock.connect(SOCKET)
        sock.sendall((json.dumps(cmd) + '\n').encode())
        result = json.loads(sock.makefile('rb').readline(16384))
    if not result.get('ok'):
        raise SystemExit('Hand request refused: '+str(result.get('error')))
    return result


def main():
    p = argparse.ArgumentParser()
    sp = p.add_subparsers(dest='cmd', required=True)
    sp.add_parser('status', help='Read actual feedback, never move')
    step = sp.add_parser('step', help='Single tiny movement; needs explicit motion startup/ack')
    step.add_argument('side', choices=('left', 'right'))
    step.add_argument('motor', choices=MOTORS)
    step.add_argument('direction', choices=('open', 'close'))
    step.add_argument('--amount', type=float, default=0.02, help='Normalized 0..0.05 max; 0.02 = 2 percent range')
    step.add_argument('--ack', required=True)
    args = p.parse_args()
    if args.cmd == 'status':
        response = transact({'kind': 'status'})
        for side, values in response['hands'].items():
            print(f'{side}: fresh={values["fresh"]} age_s={values["age_s"]} normalized(open=1)={values.get("normalized")}')
            print(f'   URDF joints={dict(zip(values.get("joint_names",[]), values.get("joint_positions",[])))}')
    else:
        response = transact(dict(kind='step', side=args.side, motor=args.motor,
                                 direction=args.direction, amount=args.amount, ack=args.ack))
        print('Applied ONE incremental hand request. Confirm physical response and state before another.', response)


if __name__ == '__main__':
    main()
