#!/usr/bin/env python3
"""Read-only G1 LowState DDS preflight. Never creates a DDS publisher.

Requires the pinned Unitree SDK in the XR Micromamba environment. This only
proves fresh, plausible DDS samples, NOT G1 identity or motion authorization.
"""
import math
import sys
import time


def main():
    if len(sys.argv) != 2:
        sys.exit('Usage: check_g1_dds.py <network-interface>')
    interface = sys.argv[1]
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as HgLowState

    ChannelFactoryInitialize(0, networkInterface=interface)
    subscriber = ChannelSubscriber('rt/lowstate', HgLowState)
    subscriber.Init()
    deadline = time.monotonic() + 10.0
    count = 0
    while time.monotonic() < deadline:
        state = subscriber.Read(timeout=0.25)
        if state is None:
            continue
        try:
            q = [float(state.motor_state[i].q) for i in range(29)]
            dq = [float(state.motor_state[i].dq) for i in range(29)]
            mode = int(state.mode_machine)
        except (ValueError, TypeError, IndexError, AttributeError):
            continue
        if not all(map(math.isfinite, q + dq)):
            continue
        count += 1
        if count >= 2:
            print(f'PASS: read-only rt/lowstate DDS samples received on {interface} (mode_machine={mode}).', flush=True)
            return
    sys.exit('FAIL: no fresh, valid rt/lowstate samples within 10 seconds')


if __name__ == '__main__':
    main()
