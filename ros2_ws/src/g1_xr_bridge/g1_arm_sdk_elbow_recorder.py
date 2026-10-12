#!/usr/bin/env python3
"""READ-ONLY G1 elbow command/feedback recorder (ROS 2 Humble).

Subscribes to /arm_sdk and /lowstate. Does not create motor publishers.
Run while stationary to establish baseline; optionally capture one separately
approved small physical trajectory under trained operator supervision.
"""
import argparse
import csv
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from unitree_hg.msg import LowCmd, LowState

JOINTS = {"left_elbow": 18, "right_elbow": 25}


class Recorder(Node):
    def __init__(self, joint, output):
        super().__init__('g1_arm_sdk_elbow_readonly_recorder')
        self.idx = JOINTS[joint]
        self.joint = joint
        self.output = open(output, 'w', newline='', encoding='utf-8')
        self.csv = csv.writer(self.output)
        self.csv.writerow(['monotonic_s', 'elapsed_s', 'joint', 'command_q_rad',
                           'measured_q_rad', 'error_rad', 'kp', 'kd',
                           'weight', 'mode_machine'])
        self.t0 = time.monotonic()
        self.last_print = self.t0
        self.cmd = None
        self.state = None
        self.command_count = 0
        self.state_count = 0
        self.samples = 0
        self.cmd_min = float('inf')
        self.cmd_max = float('-inf')
        self.state_min = float('inf')
        self.state_max = float('-inf')
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LowCmd, '/arm_sdk', self.on_cmd, qos)
        self.create_subscription(LowState, '/lowstate', self.on_state, qos)
        self.create_timer(0.05, self.sample)

    def on_cmd(self, msg):
        motor = msg.motor_cmd[self.idx]
        self.cmd = (float(motor.q), float(motor.kp), float(motor.kd),
                    float(msg.motor_cmd[29].q))
        self.command_count += 1

    def on_state(self, msg):
        self.state = (float(msg.motor_state[self.idx].q), int(msg.mode_machine))
        self.state_count += 1

    def sample(self):
        now = time.monotonic()
        if self.cmd is None or self.state is None:
            if now - self.last_print >= 1.0:
                print(f'Waiting for topics: /arm_sdk={self.command_count} /lowstate={self.state_count}', flush=True)
                self.last_print = now
            return
        cq, kp, kd, weight = self.cmd
        sq, mode = self.state
        if not all(map(math.isfinite, (cq, sq, kp, kd, weight))):
            print('WARNING: nonfinite command or state; sample ignored', flush=True)
            return
        self.samples += 1
        self.cmd_min = min(self.cmd_min, cq)
        self.cmd_max = max(self.cmd_max, cq)
        self.state_min = min(self.state_min, sq)
        self.state_max = max(self.state_max, sq)
        self.csv.writerow([f'{now:.6f}', f'{now-self.t0:.3f}', self.joint,
                           f'{cq:.6f}', f'{sq:.6f}', f'{cq-sq:.6f}',
                           f'{kp:.3f}', f'{kd:.3f}', f'{weight:.3f}', mode])
        if now - self.last_print >= 1.0:
            self.output.flush()
            print(f't={now-self.t0:5.1f}s cmd={cq:+.5f} measured={sq:+.5f} '
                  f'err={cq-sq:+.5f} kp={kp:g} weight={weight:.2f} '
                  f'mode={mode} | cmd_range={self.cmd_max-self.cmd_min:.5f} '
                  f'measured_range={self.state_max-self.state_min:.5f}', flush=True)
            self.last_print = now

    def close(self):
        self.output.flush()
        self.output.close()
        print(f'RESULT samples={self.samples} cmd_msgs={self.command_count} '
              f'state_msgs={self.state_count}', flush=True)
        if self.samples:
            print(f'COMMAND RANGE={self.cmd_max-self.cmd_min:.5f} rad; '
                  f'MEASURED RANGE={self.state_max-self.state_min:.5f} rad', flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--joint', choices=list(JOINTS), default='left_elbow')
    p.add_argument('--seconds', type=float, default=20.0)
    p.add_argument('--output', default='/tmp/g1_elbow_sdk_trace.csv')
    args = p.parse_args()
    if not (0 < args.seconds <= 300):
        p.error('--seconds must be between 0 and 300')
    rclpy.init()
    node = Recorder(args.joint, args.output)
    print(f'READ-ONLY monitoring /arm_sdk + /lowstate, {args.joint} index={node.idx}; '
          f'{args.seconds:g}s; CSV={args.output}', flush=True)
    try:
        end = time.monotonic() + args.seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
