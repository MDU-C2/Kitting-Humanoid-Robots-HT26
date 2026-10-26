#!/usr/bin/env python3
"""READ ONLY: prints the G1's waist joint angles from rt/lowstate, for g1_tf_ros2.py.

Uses Unitree's Python SDK (unitree_sdk2py) on the robot's internal network (eth0). It only
subscribes; it has no publishers and never commands anything. Started by
`start_head_camera.sh --tf` and piped into g1_tf_ros2.py (separate processes: the SDK's
CycloneDDS and ROS 2 Foxy's CycloneDDS are different versions and must not share a process).

    python g1_waist_reader.py [--if eth0] [--rate 100]

Output, one line per sample (at most --rate per second): "yaw roll pitch" in radians.
"""
import argparse
import sys
import threading
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

# G1 29-DoF motor order: 12 waist_yaw, 13 waist_roll, 14 waist_pitch
WAIST_YAW, WAIST_ROLL, WAIST_PITCH = 12, 13, 14


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--if", dest="iface", default="eth0")
    ap.add_argument("--rate", type=float, default=100.0)
    args = ap.parse_args()

    latest = [None]
    lock = threading.Lock()

    def on_lowstate(msg):
        m = msg.motor_state
        with lock:
            latest[0] = (m[WAIST_YAW].q, m[WAIST_ROLL].q, m[WAIST_PITCH].q)

    ChannelFactoryInitialize(0, args.iface)
    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(on_lowstate, 10)
    print(f"# reading rt/lowstate on {args.iface} (read only)", file=sys.stderr, flush=True)

    period = 1.0 / args.rate
    try:
        while True:
            with lock:
                q, latest[0] = latest[0], None
            if q is not None:
                sys.stdout.write(f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f}\n")
                sys.stdout.flush()
            time.sleep(period)
    except (KeyboardInterrupt, BrokenPipeError):
        pass


if __name__ == "__main__":
    main()
