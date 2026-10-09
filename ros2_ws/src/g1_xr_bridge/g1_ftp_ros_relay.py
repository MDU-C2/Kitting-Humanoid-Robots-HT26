#!/usr/bin/env python3
"""Publish measured FTP actuator fractions for diagnostics, never /joint_states.

A value 0..1 here is a normalized Inspire actuator reading, NOT a URDF joint
angle in radians. Physical URDF joint mapping remains intentionally unverified.
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

from g1_ftp_preflight import fetch_snapshot
from g1_ftp_state_contract import FTP_NAMES, FTP_SOCKET, FtpFeedbackError, validate_ftp_snapshot


TOPIC = '/g1_ftp/angle_act_normalized'


class FtpRelay(Node):
    def __init__(self, socket_path):
        super().__init__('g1_ftp_readonly_relay')
        self.socket_path = socket_path
        self.publisher = self.create_publisher(Float64MultiArray, TOPIC, 10)
        self.timer = self.create_timer(0.05, self.tick)
        self._last_warning = None
        self.get_logger().warn('FTP raw normalized actuator diagnostics ONLY; not URDF joint angles')

    def tick(self):
        try:
            state = validate_ftp_snapshot(fetch_snapshot(self.socket_path),
                                          now=time.monotonic())
        except (OSError, ValueError, FtpFeedbackError) as exc:
            warning = str(exc)
            if warning != self._last_warning:
                self.get_logger().warn('No fresh FTP hand feedback: ' + warning)
                self._last_warning = warning
            return  # Never give stale hand measurements a new publication
        self._last_warning = None
        msg = Float64MultiArray()
        dim = MultiArrayDimension()
        dim.label = 'left[0:6],right[6:12]; pinky,ring,middle,index,thumb_bend,thumb_rotation'
        dim.size = len(FTP_NAMES)
        dim.stride = len(FTP_NAMES)
        msg.layout.dim = [dim]
        msg.layout.data_offset = 0
        msg.data = list(state.normalized)
        self.publisher.publish(msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--socket', default=FTP_SOCKET)
    opts = parser.parse_args()
    rclpy.init()
    node = FtpRelay(opts.socket)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
