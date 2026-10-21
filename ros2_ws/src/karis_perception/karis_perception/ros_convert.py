"""Conversions between ROS 2 image/pose messages and numpy (without cv_bridge)."""
import array
import re
import struct

import cv2
import numpy as np
from geometry_msgs.msg import Pose, Transform, Vector3
from sensor_msgs.msg import CompressedImage, Image

PNG_MAGIC = b"\x89PNG"
DEPTH_HEADER = struct.pack("<iff", 0, 0.0, 0.0)   # compressed_depth_image_transport ConfigHeader


# ---------------------------------------------------------------- images
def image_msg_type(topic):
    """CompressedImage for '.../compressed' and '.../compressedDepth' topics, else Image."""
    return CompressedImage if topic.rstrip("/").split("/")[-1] in ("compressed", "compressedDepth") \
        else Image


def color_to_bgr(msg):
    if isinstance(msg, CompressedImage):
        img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            raise ValueError(f"cannot decode colour image (format '{msg.format}')")
        return img
    arr = _raw_array(msg)
    enc = msg.encoding.lower()
    if enc == "bgr8":
        return arr
    if enc == "rgb8":
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    if enc == "bgra8":
        return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
    if enc == "rgba8":
        return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
    if enc in ("mono8", "8uc1"):
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    raise ValueError(f"unsupported colour encoding '{msg.encoding}'")


def depth_to_metres(msg, scale_16u=0.001):
    """Depth image (16UC1 in mm or 32FC1 in m, raw or compressedDepth PNG) -> float32 metres."""
    if isinstance(msg, CompressedImage):
        data = bytes(msg.data)
        i = data.find(PNG_MAGIC)
        if i < 0:
            raise ValueError(f"depth is not PNG compressed (format '{msg.format}'); "
                             "RVL compressedDepth is not supported, use png")
        raw = cv2.imdecode(np.frombuffer(data[i:], np.uint8), cv2.IMREAD_UNCHANGED)
        if raw is None:
            raise ValueError("cannot decode depth PNG")
        if raw.dtype == np.uint16:
            return raw.astype(np.float32) * scale_16u, raw
        raise ValueError(f"unexpected depth PNG type {raw.dtype}")
    arr = _raw_array(msg)
    enc = msg.encoding.upper()
    if enc in ("16UC1", "MONO16"):
        return arr.astype(np.float32) * scale_16u, arr
    if enc == "32FC1":
        m = np.nan_to_num(arr.astype(np.float32), nan=0.0, posinf=0.0, neginf=0.0)
        return m, (m * 1000).astype(np.uint16)
    raise ValueError(f"unsupported depth encoding '{msg.encoding}'")


_DTYPES = {"bgr8": (np.uint8, 3), "rgb8": (np.uint8, 3), "bgra8": (np.uint8, 4),
           "rgba8": (np.uint8, 4), "mono8": (np.uint8, 1), "8uc1": (np.uint8, 1),
           "mono16": (np.uint16, 1), "16uc1": (np.uint16, 1), "32fc1": (np.float32, 1)}


def _raw_array(msg):
    dt, ch = _DTYPES[msg.encoding.lower()]
    dt = np.dtype(dt).newbyteorder(">" if msg.is_bigendian else "<")
    row = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)
    arr = row[:, :msg.width * ch * dt.itemsize].copy().view(dt)
    arr = arr.reshape(msg.height, msg.width, ch) if ch > 1 else arr.reshape(msg.height, msg.width)
    return arr.astype(dt.newbyteorder("="))


def bgr_to_compressed(bgr, stamp, frame_id, quality=85):
    ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    m = CompressedImage()
    m.header.stamp, m.header.frame_id = stamp, frame_id
    m.format = "bgr8; jpeg compressed bgr8"
    m.data = array.array("B", buf.tobytes())
    return m


def depth_to_compressed(depth_u16, stamp, frame_id, level=1):
    ok, buf = cv2.imencode(".png", depth_u16, [cv2.IMWRITE_PNG_COMPRESSION, int(level)])
    m = CompressedImage()
    m.header.stamp, m.header.frame_id = stamp, frame_id
    m.format = "16UC1; compressedDepth png"
    m.data = array.array("B", DEPTH_HEADER + buf.tobytes())
    return m


def camera_info_K(msg):
    return np.array(msg.k, dtype=np.float64).reshape(3, 3)


# ---------------------------------------------------------------- poses
def quat_xyzw_to_rot(q):
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n if n > 0 else 0.0
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)]])


def rot_to_quat_xyzw(R):
    """Rotation matrix -> unit quaternion (x, y, z, w), w >= 0."""
    R = np.asarray(R, dtype=np.float64)
    tr = np.trace(R)
    if tr > 0:
        s = 2.0 * np.sqrt(tr + 1.0)
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([x, y, z, w])
    q /= np.linalg.norm(q)
    return q if q[3] >= 0 else -q


def matrix_to_pose(T):
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in T[:3, 3])
    q = rot_to_quat_xyzw(T[:3, :3])
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (float(v) for v in q)
    return p


def pose_to_matrix(p):
    T = np.eye(4)
    T[:3, :3] = quat_xyzw_to_rot((p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))
    T[:3, 3] = (p.position.x, p.position.y, p.position.z)
    return T


def transform_to_matrix(t):
    """geometry_msgs/Transform -> 4x4."""
    T = np.eye(4)
    r = t.rotation
    T[:3, :3] = quat_xyzw_to_rot((r.x, r.y, r.z, r.w))
    T[:3, 3] = (t.translation.x, t.translation.y, t.translation.z)
    return T


def matrix_to_transform(T):
    t = Transform()
    t.translation.x, t.translation.y, t.translation.z = (float(v) for v in T[:3, 3])
    q = rot_to_quat_xyzw(T[:3, :3])
    t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w = (float(v) for v in q)
    return t


def vec3(v):
    m = Vector3()
    m.x, m.y, m.z = (float(a) for a in v)
    return m


def slug(name):
    """'Rund mutter_1' -> 'rund_mutter_1' (safe as a TF frame name)."""
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")
