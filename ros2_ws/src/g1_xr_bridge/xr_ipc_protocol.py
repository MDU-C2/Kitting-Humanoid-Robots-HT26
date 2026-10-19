"""Local Unix-domain request/reply transport for OFFLINE XR bridge tests.

Deliberately transports desired joint targets only. No Unitree SDK, DDS,
actuator publisher, or hardware connection appears in this module.
"""
import json
import math
from numbers import Real
import os
import socket

SOCKET_PATH = '/tmp/g1_xr_bridge_offline.sock'
PROTOCOL = 1
MAX_LINE = 16384


class IpcError(RuntimeError):
    pass


def _readline(sock):
    buf = bytearray()
    while True:
        chunk = sock.recv(4096)
        if not chunk:
            raise IpcError('XR process closed connection before replying')
        buf.extend(chunk)
        if len(buf) > MAX_LINE:
            raise IpcError('IPC message exceeds limit')
        if b'\n' in buf:
            line, rest = bytes(buf).split(b'\n', 1)
            if rest:
                raise IpcError('Unexpected trailing IPC data')
            try:
                return json.loads(line.decode('utf-8'))
            except (UnicodeError, ValueError) as exc:
                raise IpcError(f'Invalid IPC JSON: {exc}') from exc


def _vector14(values):
    try:
        values = list(values)
    except (TypeError, ValueError) as exc:
        raise IpcError('Expected 14 joint values') from exc
    # Trajectory interpolation returns NumPy scalar values (e.g. np.float64).
    # Accept real numeric scalars, not strings, booleans, complex or NaN/Inf.
    # Keep this protocol NumPy-independent so both ROS and XR can import it.
    if len(values) != 14 or any(
        isinstance(x, bool) or not isinstance(x, Real) or not math.isfinite(x)
        for x in values
    ):
        raise IpcError('Expected exactly 14 finite numeric joint values')
    return [float(x) for x in values]


def transact(message, path=SOCKET_PATH, timeout=1.0):
    """Send one JSON request, get one response, fail closed on transport errors."""
    req = dict(message)
    req['protocol'] = PROTOCOL
    payload = json.dumps(req, separators=(',', ':'), allow_nan=False).encode() + b'\n'
    if len(payload) > MAX_LINE:
        raise IpcError('IPC request too large')
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
            sock.settimeout(timeout)
            sock.connect(path)
            sock.sendall(payload)
            response = _readline(sock)
    except (OSError, ValueError) as exc:
        raise IpcError(f'XR receiver unavailable: {exc}') from exc
    if not isinstance(response, dict) or response.get('protocol') != PROTOCOL:
        raise IpcError('XR receiver protocol mismatch')
    if response.get('error'):
        raise IpcError(f"XR receiver rejected command: {response['error']}")
    if response.get('ok') is not True:
        raise IpcError('XR receiver did not acknowledge command')
    return response


def ping(path=SOCKET_PATH):
    answer = transact({'kind': 'ping'}, path)
    if answer.get('kind') != 'pong' or answer.get('hardware_connected') is not False:
        raise IpcError('Receiver did not identify as offline')
    return answer


def send_sample(q, source, seq, path=SOCKET_PATH):
    if source not in ('trajectory', 'hold'):
        raise IpcError('Only offline trajectory/hold samples permitted')
    if type(seq) is not int or seq < 0:
        raise IpcError('Sequence must be a non-negative integer')
    answer = transact({'kind': 'sample', 'source': source, 'seq': seq,
                       'q': _vector14(q)}, path)
    if answer.get('kind') != 'sample_ack' or answer.get('seq') != seq or answer.get('source') != source:
        raise IpcError('XR sample acknowledgement mismatch')
    answer['tau_ff'] = _vector14(answer.get('tau_ff'))
    return answer
