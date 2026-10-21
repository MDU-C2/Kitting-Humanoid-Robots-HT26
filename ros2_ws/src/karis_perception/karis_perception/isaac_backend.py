"""Client side of the Isaac ROS FoundationPose helper (/karis/fp/estimate_pose).

IsaacEstimator has the register() / track_one() / pose_last interface the tracker uses.
The drift check (mesh silhouette vs. detector mask) and the 3D box drawing run on the CPU.
"""
from __future__ import annotations

import array
import itertools
import threading
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from karis_interfaces.srv import EstimatePose
from karis_perception import ros_convert as rc

_ids = itertools.count(1)


class UnknownClass(RuntimeError):
    """The Isaac helper does not serve this class."""


class IsaacState:
    """Per-part tracking state: the helper's track id and the last pose."""

    def __init__(self, track_id, pose):
        self.track_id, self.pose = track_id, np.array(pose)

    def clone(self):
        return IsaacState(self.track_id, self.pose)


class IsaacClient:
    """Calls /karis/fp/estimate_pose from its own node and executor thread, so the caller can block."""

    def __init__(self, context_node, service="/karis/fp/estimate_pose", timeout_s=12.0):
        self.log = context_node.get_logger()
        # ignore launch remaps such as __node:=
        self.node = Node("karis_fp_client", use_global_arguments=False)
        self.cli = self.node.create_client(EstimatePose, service)
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self.node)
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()
        self.timeout = timeout_s
        self.service = service
        self.frame = None            # (color CompressedImage, depth CompressedImage, CameraInfo)
        self.sent = None             # stamp of the frame whose images the helper has
        self.times = []

    def _spin(self):
        while rclpy.ok():
            try:
                self.ex.spin_once(timeout_sec=0.05)
            except Exception:          # shutting down
                return

    def wait_for_helper(self, timeout_s=0.0):
        return self.cli.wait_for_service(timeout_sec=timeout_s)

    def set_frame(self, color, depth, info):
        self.frame = (color, depth, info)

    def call(self, track_id, class_name, mode, mask=None):
        color, depth, info = self.frame
        stamp = (color.header.stamp.sec, color.header.stamp.nanosec)
        for attempt in (0, 1):
            req = EstimatePose.Request()
            req.track_id, req.class_name, req.mode = track_id, class_name, mode
            req.camera_info = info
            if self.sent == stamp and attempt == 0:          # helper has this frame already
                req.color = self._header_only(color)
                req.depth = self._header_only(depth)
            else:
                req.color, req.depth = color, depth
            if mask is not None:
                req.mask = mask
            res = self._call(req)
            if res.success:
                self.sent = stamp
                self.times.append(res.processing_ms)
                return res
            if res.message != "frame not cached":
                break
        if res.message.startswith("unknown class"):
            raise UnknownClass(f"{class_name}: {res.message}")
        raise RuntimeError(f"Isaac FoundationPose {mode} of {class_name}: {res.message}")

    @staticmethod
    def _header_only(msg):
        m = CompressedImage()
        m.header, m.format = msg.header, msg.format
        return m

    def _call(self, req):
        if not self.cli.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f"{self.service} is not available: is the Isaac helper running? "
                               "(ros2 launch karis_isaac_fp isaac_fp.launch.py)")
        fut = self.cli.call_async(req)
        done = threading.Event()
        fut.add_done_callback(lambda f: done.set())
        if not done.wait(self.timeout):
            raise RuntimeError(f"no answer from {self.service} in {self.timeout:.0f} s")
        return fut.result()

    def shutdown(self):
        self.ex.remove_node(self.node)
        self.node.destroy_node()


class IsaacEstimator:
    """FoundationPose estimator for one class, backed by the Isaac helper."""

    def __init__(self, client, class_name, verts, faces, ests=None):
        self.client, self.class_name = client, class_name
        self.ests = ests                          # the class is removed from it if the helper lacks it
        self.verts = np.asarray(verts, dtype=np.float64)   # CAD frame [m]
        self.faces = np.asarray(faces, dtype=np.int64)
        self.pose_last = None

    def register(self, K, rgb, depth, ob_mask, iteration=None, **kw):
        ok, png = cv2.imencode(".png", (np.asarray(ob_mask) > 0).astype(np.uint8) * 255)
        mask = CompressedImage()
        mask.header = self.client.frame[0].header
        mask.format = "mono8; png compressed"
        mask.data = array.array("B", png.tobytes())
        tid = f"{rc.slug(self.class_name)}#{next(_ids)}"
        try:
            res = self.client.call(tid, self.class_name, "register", mask)
        except UnknownClass as e:
            # fall back to centre-only for this class
            if self.ests is not None and self.ests.pop(self.class_name, None) is not None:
                self.client.log.warn(f"{e} -> '{self.class_name}' gets position only from now on. "
                                     "Start the helper with this class to get its full pose.")
            raise RuntimeError(str(e)) from None
        pose = np.array(res.pose, dtype=np.float64).reshape(4, 4)
        self.pose_last = IsaacState(tid, pose)
        return pose

    def track_one(self, rgb, depth, K, iteration=None, **kw):
        st = self.pose_last
        try:
            res = self.client.call(st.track_id, self.class_name, "track")
        except RuntimeError as e:
            self.client.log.warn(str(e), throttle_duration_sec=5.0)
            return st.pose
        pose = np.array(res.pose, dtype=np.float64).reshape(4, 4)
        self.pose_last = IsaacState(st.track_id, pose)
        return pose


def build_isaac_estimators(classes, class_names, client, log):
    """One IsaacEstimator per mesh class known to the detector. Returns (ests, Draw(), None)."""
    import trimesh
    ests = {}
    for cls_name, cfg in classes.items():
        if cls_name not in class_names:
            log.warn(f"mesh class '{cls_name}' is not a class of this model")
            continue
        mesh = trimesh.load(str(cfg["mesh"]), force="mesh")
        mesh.apply_scale(cfg.get("scale", 1.0))
        to_origin, extents = trimesh.bounds.oriented_bounds(mesh)
        verts, faces = simplify_for_silhouette(mesh.vertices, mesh.faces)
        est = IsaacEstimator(client, cls_name, verts, faces, ests)
        ests[cls_name] = {"est": est, "to_origin": to_origin,
                          "bbox": np.stack([-extents / 2, extents / 2], axis=0),
                          "extent": float(extents.max())}
        log.info(f"  {cls_name}: {Path(cfg['mesh']).name} -> Isaac ROS FoundationPose "
                 f"(drift check: {len(faces)} of {len(mesh.faces)} triangles)")
    return ests, Draw(), None


def simplify_for_silhouette(verts, faces, cell_m=0.001, max_faces=6000):
    """Merge vertices on a grid (growing from cell_m) until at most max_faces triangles remain."""
    verts = np.asarray(verts, dtype=np.float64)
    faces = np.asarray(faces, dtype=np.int64)
    cell = cell_m
    for _ in range(8):
        if len(faces) <= max_faces:
            return verts, faces
        keys = np.floor(verts / cell).astype(np.int64)
        _, inv = np.unique(keys, axis=0, return_inverse=True)
        inv = inv.ravel()
        n = inv.max() + 1
        new_v = np.zeros((n, 3))
        np.add.at(new_v, inv, verts)
        new_v /= np.bincount(inv, minlength=n)[:, None]
        f = inv[faces]
        f = f[(f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])]
        f = np.unique(np.sort(f, axis=1), axis=0)
        if len(f) <= max_faces:
            return new_v, f
        cell *= 1.5
    return new_v, f


def silhouette_check(entry, pose, K, depth, mask, fpu=None, glctx=None):
    """IoU between the mesh projected at `pose` and the detector mask -> (iou, None, rendered mask)."""
    try:
        est = entry["est"]
        h, w = mask.shape[:2]
        rmask = render_silhouette(est.verts, est.faces, pose, K, (h, w))
        m = np.asarray(mask, dtype=bool)
        union = np.count_nonzero(rmask | m)
        iou = np.count_nonzero(rmask & m) / union if union else 0.0
        return float(iou), None, rmask
    except Exception as e:
        print(f"    silhouette check failed: {e}")
        return None, None, None


def render_silhouette(verts, faces, pose, K, shape):
    """Boolean mask of the mesh projected with pinhole K at `pose` (4x4, mesh -> camera)."""
    h, w = shape
    P = verts @ pose[:3, :3].T + pose[:3, 3]
    z = P[:, 2]
    ok = z > 1e-4
    uv = np.zeros((len(P), 2))
    uv[ok, 0] = K[0, 0] * P[ok, 0] / z[ok] + K[0, 2]
    uv[ok, 1] = K[1, 1] * P[ok, 1] / z[ok] + K[1, 2]
    tri = faces[ok[faces].all(axis=1)]
    out = np.zeros((h, w), dtype=np.uint8)
    if len(tri):
        pts = np.round(uv[tri] * 16).astype(np.int32)  # shift=4: sub-pixel precision
        cv2.fillPoly(out, list(pts), 1, lineType=cv2.LINE_8, shift=4)
    return out.astype(bool)


def _project(K, p):
    return (int(round(K[0, 0] * p[0] / p[2] + K[0, 2])), int(round(K[1, 1] * p[1] / p[2] + K[1, 2])))


class Draw:
    """Draws a 3D box and axes (same signatures as FoundationPose's Utils)."""

    @staticmethod
    def draw_posed_3d_box(K, img, ob_in_cam, bbox, line_color=(0, 255, 0), linewidth=2, **kw):
        lo, hi = np.asarray(bbox[0]), np.asarray(bbox[1])
        corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        P = corners @ ob_in_cam[:3, :3].T + ob_in_cam[:3, 3]
        if np.any(P[:, 2] <= 1e-4):
            return img
        px = [_project(K, p) for p in P]
        for i in range(8):
            for j in range(i + 1, 8):
                if bin(i ^ j).count("1") == 1:        # edge
                    cv2.line(img, px[i], px[j], line_color, linewidth, cv2.LINE_AA)
        return img

    @staticmethod
    def draw_xyz_axis(img, ob_in_cam, scale, K, thickness=2, is_input_rgb=True, **kw):
        o = ob_in_cam[:3, 3]
        if o[2] <= 1e-4:
            return img
        cols = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]          # x red, y green, z blue
        if not is_input_rgb:
            cols = [c[::-1] for c in cols]
        for i, col in enumerate(cols):
            e = o + ob_in_cam[:3, i] * scale
            if e[2] > 1e-4:
                cv2.line(img, _project(K, o), _project(K, e), col, thickness, cv2.LINE_AA)
        return img


def timing_summary(client):
    t = client.times
    if not t:
        return ""
    return (f"Isaac helper per call: median {np.median(t):.1f} ms, max {np.max(t):.1f} ms "
            f"({len(t)} calls)")
