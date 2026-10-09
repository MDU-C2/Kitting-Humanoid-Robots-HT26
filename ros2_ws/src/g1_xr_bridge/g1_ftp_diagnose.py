#!/usr/bin/env python3
"""Read-only local diagnostic of the already-running FTP observer socket."""
import argparse
import json
import socket
import time

from g1_ftp_state_contract import FTP_SOCKET


def fetch(path):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.settimeout(2)
        sock.connect(path)
        sock.sendall(b'{"kind":"ftp_status"}\n')
        reply = b''
        while not reply.endswith(b'\n'):
            part = sock.recv(4096)
            if not part:
                break
            reply += part
            if len(reply) > 32768:
                raise ValueError('Excessive FTP diagnostic response size')
    return json.loads(reply)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--socket', default=FTP_SOCKET)
    parser.add_argument('--samples', type=int, default=5)
    parser.add_argument('--interval', type=float, default=1.0)
    args = parser.parse_args()
    if not (1 <= args.samples <= 60 and 0.1 <= args.interval <= 30):
        parser.error('samples must be 1..60; interval must be 0.1..30 seconds')
    for i in range(args.samples):
        try:
            state = fetch(args.socket)
        except (OSError, ValueError) as exc:
            parser.exit(2, f'FTP observer socket unavailable: {exc}\n')
        diag = state.get('diagnostics')
        if diag is None:
            parser.exit(2, 'FTP observer is running without diagnostics; restart with updated source.\n')
        print(f'Sample {i + 1} | both_fresh={state.get("both_fresh")}')
        for hand in ('left', 'right'):
            print(f'  {hand:5s}: received={diag["received_messages"][hand]} '
                  f'rejected={diag["rejected_messages"][hand]} '
                  f'last_msg_age={diag["last_message_age_s"][hand]} '
                  f'valid_age={state.get(hand + "_age_s")} '
                  f'last_error={diag["last_rejection"][hand]}')
        if i + 1 < args.samples:
            time.sleep(args.interval)


if __name__ == '__main__':
    main()
