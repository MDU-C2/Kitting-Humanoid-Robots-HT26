"""Publish ROS joint state from local simulated XR or read-only G1 observer.

No commands can be sent through this process. In simulation the upstream
measurement is explicitly SYNTHETIC. In observation it is rt/lowstate.
"""
import argparse
import json
import math
import socket
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from g1_state_contract import JOINT_NAMES_29
from xr_inprocess_bridge import SIM_SOCKET
from g1_readonly_observer import SOCKET as OBSERVER_SOCKET


def read_state(socket_path, mode):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.settimeout(0.15)
        sock.connect(socket_path)
        sock.sendall(b'{"kind":"status"}\n' if mode == 'observe' else
                     b'{"protocol":1,"kind":"status"}\n')
        file = sock.makefile('rb')
        data = file.readline(16384)
        if not data or len(data) == 16384:
            raise ValueError('No complete state reply')
    answer = json.loads(data)
    if mode == 'observe':
        if (answer.get('kind') != 'state' or answer.get('read_only') is not True
                or not answer.get('fresh')):
            raise ValueError('Read-only G1 state unavailable or stale')
        names = answer['joint_names']
        pos, vel = answer['positions'], answer['velocities']
    else:
        if (answer.get('protocol') != 1 or answer.get('ok') is not True
                or answer.get('kind') != 'status_ack'
                or answer.get('hardware_connected') is not False
                or not answer.get('xr_loop_fresh')
                or not answer.get('feedback_fresh')):
            raise ValueError('Synthetic XR state unavailable or stale')
        names = JOINT_NAMES_29
        pos = answer.get('synthetic_joint_positions' if mode == 'offline'
                         else 'sim_joint_positions')
        vel = [0.0] * len(names)
    if (list(names) != list(JOINT_NAMES_29) or not isinstance(pos, list)
            or len(pos) != 29 or len(vel) != 29
            or not all(isinstance(x, (float, int)) and not isinstance(x, bool)
                       and math.isfinite(x) for x in pos + vel)):
        raise ValueError('Invalid/mismatched 29-joint DDS state')
    return list(pos), list(vel)


class Relay(Node):
    def __init__(self, mode):
        super().__init__('g1_xr_joint_state_relay')
        self.mode = mode
        self.path = OBSERVER_SOCKET if mode == 'observe' else SIM_SOCKET
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.025 if mode == 'offline' else 0.05, self.tick)
        self._last_error = None
        self.get_logger().warn('Joint states: ' + ('READ-ONLY real feedback' if mode == 'observe'
                                                   else ('SYNTHETIC instant tracking; no hardware'
                                                         if mode == 'offline' else 'ISOLATED simulator LowState')))

    def tick(self):
        try:
            pos, vel = read_state(self.path, self.mode)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            problem = str(exc)
            if problem != self._last_error:
                self.get_logger().warn(f'No fresh joint states: {problem}')
                self._last_error = problem
            return  # Do not republish stale data with a fresh ROS timestamp
        self._last_error = None
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES_29)
        msg.position = pos
        msg.velocity = vel

        # Synthetic Inspire hand state for offline MoveIt testing only.
        # Never substitute these values for physical hand feedback.
        if self.mode == 'offline':
            hand_joints = [
                f"{side}_{joint}_joint"
                for side in ("left", "right")
                for joint in (
                    "index_1", "little_1", "middle_1",
                    "ring_1", "thumb_1", "thumb_2"
                )
            ]
            msg.name.extend(hand_joints)
            msg.position.extend([0.0] * len(hand_joints))
            msg.velocity.extend([0.0] * len(hand_joints))

        self.publisher.publish(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['offline', 'observe', 'xr-sim'], required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Relay(args.mode)
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
