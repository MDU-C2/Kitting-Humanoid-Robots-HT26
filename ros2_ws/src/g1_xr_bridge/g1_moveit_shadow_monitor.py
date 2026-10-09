#!/usr/bin/env python3
"""Subscribe to MoveIt *displayed plans*; inspect versus real G1 state ONLY.

No ActionServer, publishers, command samples, XR process or DDS motor access.
Do not confuse an inspected displayed plan with an execute command.
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from moveit_msgs.msg import DisplayTrajectory

from g1_hardware_preflight import fetch_snapshot
from g1_moveit_shadow_core import inspect_display_plan


class MoveItShadowMonitor(Node):
    def __init__(self):
        super().__init__('g1_moveit_plan_shadow_readonly')
        # Subscriber VOLATILE matches both volatile and transient-local writers.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(DisplayTrajectory, '/display_planned_path',
                                 self.inspect, qos)
        self.get_logger().warn(
            'READ-ONLY MoveIt plan shadow listening on /display_planned_path. '
            'A plan is NEVER an execution command; no XR/robot motor path exists.')

    def inspect(self, message):
        try:
            # Fresh LowState is required at inspection time. No cached plan
            # or observer state is trusted as a valid physical baseline.
            now = time.monotonic()
            snapshot = fetch_snapshot()
            result = inspect_display_plan(message, snapshot, now=time.monotonic())
            self.get_logger().info(
                'MOVEIT PLAN SHADOW PASS — NOT EXECUTED: '
                + json.dumps(result.__dict__, allow_nan=False))
        except Exception as exc:
            self.get_logger().warn('MOVEIT PLAN SHADOW REJECTED — NO ACTUATION: ' + str(exc))


def main():
    rclpy.init()
    node = MoveItShadowMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
