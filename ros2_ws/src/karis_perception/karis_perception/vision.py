"""Detection post-processing (mask cleanup, ids), mask centres, rotation helpers and part classes.

The module-level settings are defaults; the pose estimator sets them from perception.yaml.
"""
from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from karis_perception import detectors

# ------------------------------------------------------------ settings
CONFIDENCE = 0.6          # detector confidence threshold
IOU = 0.6                 # detector NMS IoU
DEVICE = 0                # GPU index for YOLO
MIN_MASK_AREA_PX = 300    # smaller masks are ignored
MIN_VALID_DEPTH_PX = 50   # a part needs this many depth pixels in its mask
ROI = None                # (x1, y1, x2, y2) px: ignore parts whose centre is outside
MIN_DEPTH_M, MAX_DEPTH_M = 0.1, 3.0
REG_ITERATIONS = 3        # refinement steps per registration


# ------------------------------------------------------------ classes
def package_share() -> Path:
    """share/karis_perception of the installed package (config/, meshes/)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("karis_perception"))
    except Exception:                    # not installed (tests from the source tree)
        return Path(__file__).resolve().parent.parent


def load_classes(path=None):
    """{class name: {"mesh": Path, "scale": float}} from classes.yaml. Mesh paths are relative
    to the yaml's meshes/ folder (the package's), or absolute."""
    import yaml
    share = package_share()
    path = Path(path) if path else share / "config" / "classes.yaml"
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    mesh_dir = Path(data.get("mesh_dir") or (share / "meshes"))
    if not mesh_dir.is_absolute():
        mesh_dir = path.parent / mesh_dir
    out = {}
    for name, cfg in (data.get("classes") or {}).items():
        mesh = Path(cfg["mesh"])
        out[str(name)] = {"mesh": mesh if mesh.is_absolute() else mesh_dir / mesh,
                          "scale": float(cfg.get("scale", 1.0))}
    return out


# ------------------------------------------------------------ detection
def clean_mask(mask: np.ndarray, k: int = 3) -> np.ndarray:
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    m = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
    if n <= 1:
        return np.zeros_like(mask)
    largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    m = (labels == largest).astype(np.uint8) * 255
    contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filled = np.zeros_like(m)
    cv2.drawContours(filled, contours, -1, 255, thickness=cv2.FILLED)
    return filled


def cuda_sync():
    """Wait for queued GPU work so wall-clock timings are real."""
    try:
        import torch
        if torch.cuda.is_available():
            torch.cuda.synchronize()
    except Exception:
        pass


def detect(model, frame_bgr, timing=None, depth=None):
    """Detector + mask cleanup. `model` is a detectors.YoloDetector / RFDETRDetector.
    Boxes without a mask get one from `depth` (see detectors.box_mask).
    If `timing` is a dict, the detector's own inference time (ms) is stored in it."""
    raw = model.predict(frame_bgr, CONFIDENCE, timing)
    if not raw:
        return []
    h, w = frame_bgr.shape[:2]
    dets = []
    for cls_id, conf, box, mask in raw:
        if mask is None:
            mask = detectors.box_mask(box, (h, w), depth)
        m = mask.astype(np.uint8) * 255
        if m.shape != (h, w):
            m = cv2.resize(m, (w, h), interpolation=cv2.INTER_NEAREST)
        m = clean_mask(m)
        if int(np.count_nonzero(m)) < MIN_MASK_AREA_PX:
            continue
        ys, xs = np.nonzero(m)
        cu, cv_ = float(xs.mean()), float(ys.mean())
        if ROI is not None:
            x1, y1, x2, y2 = ROI
            if not (x1 <= cu <= x2 and y1 <= cv_ <= y2):
                continue
        dets.append({"cls_id": int(cls_id), "cls_name": model.names[int(cls_id)],
                     "conf": float(conf), "box": box, "mask": m > 0,
                     "center_px": (cu, cv_)})
    # stable ids: class, then left to right
    dets.sort(key=lambda d: (d["cls_name"], d["center_px"][0]))
    counters = {}
    for d in dets:
        counters[d["cls_name"]] = counters.get(d["cls_name"], 0) + 1
        d["object_id"] = f"{d['cls_name']}_{counters[d['cls_name']]}"
    return dets


def centre_from_depth(mask, depth, K):
    valid = mask & (depth > 0)
    if np.count_nonzero(valid) < MIN_VALID_DEPTH_PX:
        return None
    ys, xs = np.nonzero(mask)
    u, v = xs.mean(), ys.mean()
    z = float(np.median(depth[valid]))
    return np.array([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z])


def rot_to_rpy_deg(R):
    """R = Rz(yaw) Ry(pitch) Rx(roll), same convention as ROS / tf."""
    pitch = np.arcsin(np.clip(-R[2, 0], -1.0, 1.0))
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return np.degrees([roll, pitch, yaw])


def rot_to_quat_xyzw(R):
    tr = np.trace(R)
    if tr > 0:
        s = 2.0 * np.sqrt(tr + 1.0)
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q = [0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s]
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        q = [(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s]
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        q = [(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s]
    q = np.array(q)
    return q / np.linalg.norm(q)


def class_color(cls_id):
    return tuple(int(x) for x in np.random.RandomState(cls_id).randint(60, 255, 3))
