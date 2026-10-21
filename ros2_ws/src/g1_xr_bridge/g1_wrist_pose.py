#!/usr/bin/env python3
"""Read-only torso-relative G1 wrist FK from the MoveIt Inspire FTP URDF.

Independent of XR/Pinocchio: ROS action server uses system Python (numpy).
Inputs MUST be 14-arm vectors in xr_arm_trajectory.ARM_JOINT_NAMES order.
Frames are left/right_wrist_yaw_link origins relative to torso_link, NOT TCPs.
"""
import math
import xml.etree.ElementTree as ET
import numpy as np

WRIST_LINKS = ('left_wrist_yaw_link', 'right_wrist_yaw_link')


def _rpy_matrix(angles):
    r, p, y = angles
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([
        [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
        [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
        [-sp, cp*sr, cp*cr],
    ], dtype=float)


def _joint_rotation(axis, theta):
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError('Invalid URDF joint axis')
    x, y, z = axis / norm
    c, s = math.cos(theta), math.sin(theta)
    v = 1.0 - c
    return np.array([
        [c+x*x*v, x*y*v-z*s, x*z*v+y*s],
        [y*x*v+z*s, c+y*y*v, y*z*v-x*s],
        [z*x*v-y*s, z*y*v+x*s, c+z*z*v],
    ], dtype=float)


class WristFK:
    """URDF FK validated against the 7-DOF torso-to-wrist chains."""
    def __init__(self, urdf_path, arm_joint_names):
        self.names = tuple(arm_joint_names)
        if len(self.names) != 14 or len(set(self.names)) != 14:
            raise ValueError('Expected fourteen distinct G1 arm joint names')
        self.idx = {name: i for i, name in enumerate(self.names)}
        root = ET.parse(urdf_path).getroot()
        child_joints = {}
        for joint in root.findall('joint'):
            parent = joint.find('parent')
            child = joint.find('child')
            if parent is not None and child is not None:
                child_link = child.get('link')
                if child_link in child_joints:
                    raise ValueError(f'Duplicate parent joint for {child_link}')
                child_joints[child_link] = (joint, parent.get('link'))
        self.chains = {}
        for wrist in WRIST_LINKS:
            chain = []
            current = wrist
            seen = set()
            while current != 'torso_link':
                if current in seen or current not in child_joints:
                    raise ValueError(f'Cannot find torso_link -> {wrist} chain')
                seen.add(current)
                joint, parent_link = child_joints[current]
                name = joint.get('name')
                kind = joint.get('type')
                if kind not in ('revolute', 'continuous', 'fixed'):
                    raise ValueError(f'Unsupported joint type {kind}: {name}')
                origin = joint.find('origin')
                xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
                rpy = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
                if (xyz.shape != (3,) or rpy.shape != (3,) or
                        not np.isfinite(xyz).all() or not np.isfinite(rpy).all()):
                    raise ValueError(f'Invalid URDF origin for {name}')
                offset = np.eye(4)
                offset[:3, :3] = _rpy_matrix(rpy)
                offset[:3, 3] = xyz
                if kind == 'fixed':
                    joint_index = None
                    axis = None
                else:
                    if name not in self.idx:
                        raise ValueError(f'Unexpected active wrist-chain joint: {name}')
                    joint_index = self.idx[name]
                    axis_tag = joint.find('axis')
                    axis = np.fromstring(axis_tag.get('xyz', '1 0 0'), sep=' ') if axis_tag is not None else np.array([1., 0., 0.])
                    if axis.shape != (3,) or not np.isfinite(axis).all():
                        raise ValueError(f'Invalid URDF axis for {name}')
                chain.append((name, kind, offset, joint_index, axis))
                current = parent_link
            chain.reverse()
            expected = set(self.names[:7] if wrist.startswith('left') else self.names[7:])
            actual = {item[0] for item in chain if item[3] is not None}
            if actual != expected or len([item for item in chain if item[3] is not None]) != 7:
                raise ValueError(f'Incorrect wrist-chain joint mapping for {wrist}: {actual}')
            self.chains[wrist] = chain

    def poses(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (14,) or not np.isfinite(q).all():
            raise ValueError('Wrist FK requires 14 finite measured or desired joint positions')
        poses = {}
        for wrist, chain in self.chains.items():
            transform = np.eye(4)
            for _, kind, offset, index, axis in chain:
                transform = transform @ offset
                if kind != 'fixed':
                    moving = np.eye(4)
                    moving[:3, :3] = _joint_rotation(axis, float(q[index]))
                    transform = transform @ moving
            poses[wrist] = transform
        return poses

    def errors(self, target, measured):
        desired_poses = self.poses(target)
        actual_poses = self.poses(measured)
        errors = {}
        for name in WRIST_LINKS:
            desired, actual = desired_poses[name], actual_poses[name]
            delta_p = desired[:3, 3] - actual[:3, 3]
            relative_r = desired[:3, :3].T @ actual[:3, :3]
            rotation_angle = math.acos(float(np.clip((np.trace(relative_r) - 1.0) / 2.0, -1.0, 1.0)))
            errors[name] = (float(np.linalg.norm(delta_p)), float(rotation_angle))
        return errors
