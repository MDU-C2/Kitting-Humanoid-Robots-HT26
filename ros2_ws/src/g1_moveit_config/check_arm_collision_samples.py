#!/usr/bin/env python3
"""Read-only MoveIt collision-state survey for isolated G1 offline simulation.

Calls /check_state_validity only. Does not execute plans or publish joint commands.
Use --samples/--seed for reproducible, bounded arm-state sampling.
"""
import argparse
import random
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ARMS = [
    f'{side}_{joint}_joint'
    for side in ('left', 'right')
    for joint in ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw',
                  'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw')
]


def get_limits(urdf_path):
    root = ET.parse(urdf_path).getroot()
    joints = {node.get('name'): node for node in root.findall('joint')}
    limits = []
    for name in ARMS:
        joint = joints.get(name)
        if joint is None or joint.find('limit') is None:
            raise ValueError(f'Missing URDF limits for {name}')
        tag = joint.find('limit')
        lo, hi = float(tag.get('lower')), float(tag.get('upper'))
        if not lo < hi:
            raise ValueError(f'Invalid limits for {name}')
        limits.append((lo, hi))
    return limits


def verify_collision_links(urdf_path):
    root = ET.parse(urdf_path).getroot()
    links = {node.get('name'): node for node in root.findall('link')}
    for name in ('pelvis', 'waist_yaw_link', 'waist_roll_link'):
        if name not in links or links[name].find('collision/geometry') is None:
            raise ValueError(f'Missing collision geometry: {name}')
    print('PASS: pelvis and waist collision elements exist in source URDF', flush=True)


def sample_positions(limits, total, seed):
    rng = random.Random(seed)
    yield 'all_zero_clamped', [min(hi, max(lo, 0.0)) for lo, hi in limits]
    for i in range(total):
        # One conservative interior set per joint (avoids direct URDF endpoints).
        values = [rng.uniform(lo + 0.05 * (hi - lo),
                              hi - 0.05 * (hi - lo)) for lo, hi in limits]
        yield f'random_{i + 1:03}', values


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples', type=int, default=100)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--timeout', type=float, default=10.0)
    parser.add_argument('--urdf', type=Path, default=Path(
        '/workspace/src/g1_description/urdf/g1_29dof_with_inspire_hand_ftp.urdf'))
    parser.add_argument('--offline-ack', action='store_true', required=True,
                        help='Required: run only with isolated --mode offline pipeline')
    args = parser.parse_args()
    if args.samples < 1 or args.samples > 2000:
        parser.error('--samples must be 1..2000')
    verify_collision_links(args.urdf)
    limits = get_limits(args.urdf)

    import rclpy
    from moveit_msgs.srv import GetStateValidity
    from moveit_msgs.msg import RobotState
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = rclpy.create_node('g1_offline_collision_survey')
    try:
        discovered = {name for name, _ in node.get_node_names_and_namespaces()}
        if 'g1_xr_bridge_offline_ipc' not in discovered:
            raise RuntimeError('Offline XR action node not found; refusing survey outside offline pipeline')
        client = node.create_client(GetStateValidity, '/check_state_validity')
        if not client.wait_for_service(timeout_sec=args.timeout):
            raise RuntimeError('/check_state_validity unavailable')

        def call(names, positions, diff=True):
            request = GetStateValidity.Request()
            request.group_name = 'both_arms'
            request.robot_state = RobotState()
            request.robot_state.is_diff = diff
            request.robot_state.joint_state = JointState()
            request.robot_state.joint_state.name = list(names)
            request.robot_state.joint_state.position = list(positions)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
            if not future.done():
                raise RuntimeError('Validity service timed out')
            result = future.result()
            if result is None:
                raise RuntimeError(f'Validity service failed: {future.exception()}')
            return result

        baseline = call([], [])
        print(f'Current planning scene: valid={baseline.valid}, contacts={len(baseline.contacts)}', flush=True)
        if not baseline.valid:
            print('WARNING: current scene is already invalid; review state before interpreting samples', flush=True)
        valid_count = 0
        invalid_count = 0
        contact_pairs = {}
        for label, positions in sample_positions(limits, args.samples, args.seed):
            response = call(ARMS, positions)
            if response.valid:
                valid_count += 1
                continue
            invalid_count += 1
            pairs = sorted({tuple(sorted((contact.contact_body_1, contact.contact_body_2)))
                            for contact in response.contacts})
            print(f'INVALID {label}: contact_pairs={pairs[:8]}', flush=True)
            for pair in pairs:
                contact_pairs[pair] = contact_pairs.get(pair, 0) + 1
        print(f'SURVEY: {valid_count} valid, {invalid_count} invalid, '
              f'{args.samples + 1} sampled configurations; seed={args.seed}', flush=True)
        for pair, count in sorted(contact_pairs.items(), key=lambda item: -item[1])[:20]:
            print(f'  contact_pair {pair}: {count} sample(s)')
        if invalid_count == 0:
            print('NOTE: No invalid samples found; this does NOT prove collision detection works.')
        if invalid_count and not contact_pairs:
            print('NOTE: Invalid samples without reported contacts: inspect bounds, scene and constraints.')
        print('READ-ONLY survey complete; NO joint commands or trajectory execution.', flush=True)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (RuntimeError, ValueError, OSError) as exc:
        print(f'FAIL: {exc}', file=sys.stderr)
        sys.exit(2)
