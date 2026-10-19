"""
KARIS detectors: model picker + YOLO and RF-DETR backends with one interface.

    from karis_perception.detectors import choose_model, load_detector
    det = load_detector(choose_model(models_dir))      # asks which model to use
    det.names                                           # {class_id: class_name}
    raw = det.predict(bgr, conf, timing)                # list of (cls_id, conf, box, mask or None)

YOLO (.pt, Ultralytics) runs in this process.

RF-DETR (.pth) needs newer PyTorch/transformers than the ROS Python, so it runs in a
helper process, rfdetr_server.py (next to this file), in its own Python (the image's
/opt/rfdetr-venv, KARIS_RFDETR_PYTHON), and the images are sent to it over a local socket.
The helper is started automatically; a helper already running on the port is used instead:
    /opt/rfdetr-venv/bin/python <this folder>/rfdetr_server.py --model <models>/RF_DETR_m.pth

Models that only give boxes (RF-DETR detection) get a mask from the depth image:
inside the box, the pixels that stand out from the surface around the box.
"""
from __future__ import annotations

import atexit
import json
import os
import socket
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

MODEL_EXTS = {".pt", ".pth", ".ckpt"}
RFDETR_PORT = 5601
RFDETR_PYTHONS = [os.environ.get("KARIS_RFDETR_PYTHON", ""),
                  "/opt/conda/envs/rfdetr/bin/python"]
RFDETR_CLASS_OFFSET = "auto"   # "auto", 0 or -1: RF-DETR class id -> index in its name list
BOX_MASK_MODE = "depth"        # boxes without a mask: "depth" (object pixels) or "box" (whole box)
OBJECT_MIN_HEIGHT_M = 0.004    # "depth" mode: pixels this much above the surrounding surface


# ============================================================
# Model picker
# ============================================================
def model_kind(path: Path) -> str:
    return "YOLO" if path.suffix.lower() == ".pt" else "RF-DETR"


def pop_model_arg(argv):
    """Removes '--model X' from argv (so scripts can use their other arguments) and returns X."""
    if "--model" in argv:
        i = argv.index("--model")
        val = argv[i + 1] if i + 1 < len(argv) else None
        del argv[i:i + 2]
        return val
    return None


def choose_model(models_dir: Path, choice: str | None = None) -> Path:
    """Lists the models in models_dir and asks which one to use.
    `choice` (a number or a file name, e.g. from --model) skips the question.
    Enter picks the model used last time."""
    models = sorted(p for p in models_dir.iterdir()
                    if p.is_file() and p.suffix.lower() in MODEL_EXTS) if models_dir.exists() else []
    if not models:
        raise FileNotFoundError(f"No models (.pt / .pth) in {models_dir}")
    last_file = models_dir / ".last_model"
    last = last_file.read_text().strip() if last_file.exists() else ""
    default = next((i for i, m in enumerate(models) if m.name == last), 0)

    if choice is None and len(models) > 1:
        print(f"\nModels in {models_dir}:")
        for i, m in enumerate(models):
            mark = "  (last used)" if i == default and last else ""
            print(f"  {i + 1}) {m.name:<32s} {model_kind(m):<8s} {m.stat().st_size / 1e6:7.1f} MB{mark}")
        if sys.stdin.isatty():
            choice = input(f"Choose a model [1-{len(models)}] (Enter = {default + 1}): ").strip()
        else:
            choice = ""
    if choice in (None, ""):
        picked = models[default]
    elif choice.isdigit() and 1 <= int(choice) <= len(models):
        picked = models[int(choice) - 1]
    else:
        match = [m for m in models if m.name == choice or m.stem == choice]
        if not match:
            raise ValueError(f"No model '{choice}' in {models_dir}")
        picked = match[0]
    try:
        last_file.write_text(picked.name)
    except OSError:
        pass
    print(f"Using model: {picked.name} ({model_kind(picked)})")
    return picked


def load_detector(path: Path, image_size=0, device=0, iou=0.6, fallback_size=640, log=print):
    """YOLO (.pt) or RF-DETR (.pth). image_size: YOLO input size in px; 0 = the size the
    model was trained at (fallback_size if the checkpoint does not say). RF-DETR sets its
    own input size."""
    path = Path(path)
    if model_kind(path) == "YOLO":
        det = YoloDetector(path, fallback_size, device, iou)
        trained = yolo_train_imgsz(det.model)
        if image_size and image_size > 0:
            det.image_size, src = int(image_size), "parameter image_size"
        elif trained:
            det.image_size, src = trained, "the model's training size"
        else:
            det.image_size, src = fallback_size, "the fallback size (not stored in the model)"
        note = f" (trained at {trained})" if trained and trained != det.image_size else ""
        log(f"YOLO input size {det.image_size} px from {src}{note}")
    else:
        det = RFDETRDetector(path)
        log("Detector input size: set by the RF-DETR model itself")
    det.model_file = path.name
    return det


def yolo_train_imgsz(yolo):
    """Image size an Ultralytics YOLO .pt was trained at, or None. Ultralytics keeps the
    training arguments in the checkpoint ('train_args') and copies imgsz into overrides."""
    ck = getattr(yolo, "ckpt", None)
    sources = [getattr(yolo, "overrides", None),
               ck.get("train_args") if isinstance(ck, dict) else None,
               getattr(getattr(yolo, "model", None), "args", None)]
    for src in sources:
        v = src.get("imgsz") if isinstance(src, dict) else getattr(src, "imgsz", None)
        if isinstance(v, (list, tuple)) and v:
            v = max(v)
        try:
            if v and int(v) > 0:
                return int(v)
        except (TypeError, ValueError):
            pass
    return None


class _Tee:
    """Sends everything printed to the terminal AND to a file."""

    def __init__(self, stream, path):
        self.stream, self.file = stream, open(path, "w", encoding="utf-8")

    def write(self, text):
        self.stream.write(text)
        self.file.write(text)
        self.file.flush()

    def flush(self):
        self.stream.flush()
        self.file.flush()

    def __getattr__(self, name):          # isatty, encoding, ... from the terminal
        return getattr(self.stream, name)


def save_console(out_dir: Path, name="summary.txt"):
    """From now on, everything the script prints is also saved in out_dir/summary.txt
    (FoundationPose's own debug log is not included)."""
    import datetime
    if not isinstance(sys.stdout, _Tee):
        sys.stdout = _Tee(sys.stdout, Path(out_dir) / name)
        print(f"Run started {datetime.datetime.now():%Y-%m-%d %H:%M:%S}, "
              f"command: {' '.join(sys.argv)}")


def write_model_info(out_dir: Path, det):
    """Records which model produced the results in out_dir (read by copy_results.sh)."""
    try:
        (Path(out_dir) / "model.txt").write_text(f"{det.model_file}\n")
    except OSError:
        pass


# ============================================================
# YOLO
# ============================================================
class YoloDetector:
    kind = "YOLO"

    def __init__(self, path, image_size, device, iou):
        from ultralytics import YOLO
        self.model = YOLO(str(path))
        self.names = dict(self.model.names)
        self.image_size, self.device, self.iou = image_size, device, iou
        self.has_masks = True

    def warmup(self):
        self.model.predict(np.zeros((480, 640, 3), np.uint8), imgsz=self.image_size,
                           device=self.device, verbose=False)

    def predict(self, bgr, conf, timing=None):
        r = self.model.predict(bgr, imgsz=self.image_size, conf=conf, iou=self.iou,
                               device=self.device, agnostic_nms=True, retina_masks=True,
                               verbose=False)[0]
        if timing is not None:
            sp = getattr(r, "speed", None) or {}
            timing.update(det_pre_ms=sp.get("preprocess"), det_inf_ms=sp.get("inference"),
                          det_post_ms=sp.get("postprocess"))
        if r.boxes is None or len(r.boxes) == 0:
            return []
        cls = r.boxes.cls.int().cpu().numpy()
        cf = r.boxes.conf.cpu().numpy()
        boxes = r.boxes.xyxy.cpu().numpy()
        masks = r.masks.data.cpu().numpy() if r.masks is not None else None
        return [(int(cls[i]), float(cf[i]), boxes[i], None if masks is None else masks[i] > 0.5)
                for i in range(len(cls))]


# ============================================================
# RF-DETR (client of rfdetr_server.py)
# ============================================================
def _send(sock, obj: dict, payload: bytes = b""):
    head = json.dumps(obj).encode()
    sock.sendall(struct.pack("!II", len(head), len(payload)) + head + payload)


def _recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(min(n - len(buf), 1 << 20))
        if not chunk:
            raise ConnectionError("RF-DETR helper closed the connection")
        buf += chunk
    return bytes(buf)


def _recv(sock):
    lh, lp = struct.unpack("!II", _recv_exact(sock, 8))
    head = json.loads(_recv_exact(sock, lh))
    return head, _recv_exact(sock, lp) if lp else b""


class RFDETRDetector:
    kind = "RF-DETR"

    def __init__(self, path):
        self.path = Path(path).resolve()
        self.proc = None
        self.sock = self._connect_or_start()
        _send(self.sock, {"cmd": "info"})
        info, _ = _recv(self.sock)
        if info.get("model") and Path(info["model"]).name != self.path.name:
            print(f"  NOTE: the running RF-DETR helper has {Path(info['model']).name} loaded, "
                  f"not {self.path.name}. Stop it and start it with --model {self.path.name}.")
        names = info["names"]
        self.name_list = names
        self.has_masks = bool(info.get("segmentation"))
        self.offset = None if RFDETR_CLASS_OFFSET == "auto" else int(RFDETR_CLASS_OFFSET)
        self.names = {i: n for i, n in enumerate(names)}
        print(f"  RF-DETR ({info.get('model_class', '?')}, rfdetr {info.get('version', '?')}): "
              f"{len(names)} classes, {'masks' if self.has_masks else 'boxes only'}")
        print(f"  Class names: {names}")
        if not self.has_masks:
            print(f"  Boxes only: masks are made from depth inside each box ({BOX_MASK_MODE} mode).")

    # --- helper process ---
    def _try_connect(self):
        try:
            s = socket.create_connection(("127.0.0.1", RFDETR_PORT), timeout=2)
            s.settimeout(120)
            return s
        except OSError:
            return None

    def _connect_or_start(self):
        s = self._try_connect()
        if s:
            print(f"  Using the RF-DETR helper already running on port {RFDETR_PORT}")
            return s
        py = next((p for p in RFDETR_PYTHONS if p and Path(p).exists()), None)
        if py is None:
            raise RuntimeError(
                "RF-DETR needs its own Python (>= 3.9) with the rfdetr package, and none was found.\n"
                "Either create it once inside the container:\n"
                "    conda create -y -n rfdetr python=3.11 && /opt/conda/envs/rfdetr/bin/pip install rfdetr\n"
                "or start the helper yourself in another terminal (any Python with rfdetr + GPU):\n"
                f"    python rfdetr_server.py --model {self.path}")
        server = Path(__file__).resolve().parent / "rfdetr_server.py"
        log_path = Path(tempfile.gettempdir()) / "karis_rfdetr_server.log"
        log = open(log_path, "w")
        print(f"  Starting the RF-DETR helper with {py} (log: {log_path})...")
        self.proc = subprocess.Popen([py, str(server), "--model", str(self.path),
                                      "--port", str(RFDETR_PORT)], stdout=log, stderr=subprocess.STDOUT)
        atexit.register(self.close)
        t0 = time.time()
        while time.time() - t0 < 300:
            if self.proc.poll() is not None:
                raise RuntimeError(f"The RF-DETR helper stopped; see {log_path}")
            s = self._try_connect()
            if s:
                print(f"  RF-DETR helper ready after {time.time() - t0:.0f} s")
                return s
            time.sleep(1)
        raise RuntimeError(f"The RF-DETR helper did not start within 5 min; see {log_path}")

    def close(self):
        try:
            _send(self.sock, {"cmd": "bye"})
            self.sock.close()
        except Exception:
            pass
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()

    # --- inference ---
    def warmup(self):
        self.predict(np.zeros((480, 640, 3), np.uint8), 0.5)

    def _name_index(self, cid):
        if self.offset is None:            # auto: ids index the name list unless they run past it
            return cid
        return cid + self.offset

    def predict(self, bgr, conf, timing=None):
        t0 = time.perf_counter()
        h, w = bgr.shape[:2]
        _send(self.sock, {"cmd": "predict", "h": h, "w": w, "threshold": conf},
              np.ascontiguousarray(bgr).tobytes())
        head, payload = _recv(self.sock)
        total = (time.perf_counter() - t0) * 1000
        if timing is not None:
            inf = head.get("infer_ms")
            timing.update(det_pre_ms=None, det_inf_ms=inf,
                          det_post_ms=None if inf is None else total - inf)
        n = head["n"]
        if n == 0:
            return []
        masks = None
        if head.get("has_mask"):
            masks = np.unpackbits(np.frombuffer(payload, np.uint8), count=n * h * w).reshape(n, h, w) > 0
        ids = [int(c) for c in head["class_ids"]]
        if self.offset is None and max(ids) >= len(self.name_list):
            self.offset = -1
            print("  NOTE: RF-DETR class ids run past its name list; using id - 1 for the names.")
        out = []
        for i in range(n):
            idx = self._name_index(ids[i])
            if not 0 <= idx < len(self.name_list):
                continue
            out.append((idx, float(head["scores"][i]), np.array(head["boxes"][i], np.float32),
                        None if masks is None else masks[i]))
        return out


# ============================================================
# Box -> mask from depth (for detectors without masks)
# ============================================================
def box_mask(box, shape, depth=None, mode=None):
    """Mask for a detection that only has a box. In "depth" mode the surface around
    the box (a ring just outside it) is fitted with a plane, and the pixels inside
    the box that stand OBJECT_MIN_HEIGHT_M above that plane are the object.
    Falls back to the whole box if that does not work."""
    mode = mode or BOX_MASK_MODE
    h, w = shape[:2]
    x1, y1, x2, y2 = [int(round(v)) for v in box]
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(w, x2), min(h, y2)
    full = np.zeros((h, w), bool)
    full[y1:y2, x1:x2] = True
    if mode != "depth" or depth is None or x2 - x1 < 4 or y2 - y1 < 4:
        return full

    pad = max(6, int(0.15 * max(x2 - x1, y2 - y1)))
    X1, Y1, X2, Y2 = max(0, x1 - pad), max(0, y1 - pad), min(w, x2 + pad), min(h, y2 + pad)
    ring = np.zeros((h, w), bool)
    ring[Y1:Y2, X1:X2] = True
    ring &= ~full
    ring &= depth > 0
    if np.count_nonzero(ring) < 30:
        return full
    vs, us = np.nonzero(ring)
    inv = 1.0 / depth[vs, us]                    # 1/z of a plane is linear in (u, v)
    A = np.stack([us, vs, np.ones_like(us)], 1).astype(np.float64)
    coef = None
    for _ in range(3):                            # least squares with outlier rejection
        coef, *_ = np.linalg.lstsq(A, inv, rcond=None)
        res = np.abs(A @ coef - inv)
        keep = res < max(3 * np.median(res), 1e-6)
        if keep.sum() < 30:
            break
        A, inv = A[keep], inv[keep]
    yy, xx = np.mgrid[y1:y2, x1:x2]
    plane_inv = coef[0] * xx + coef[1] * yy + coef[2]
    with np.errstate(divide="ignore", invalid="ignore"):
        plane_z = np.where(plane_inv > 0, 1.0 / plane_inv, 0)
    d = depth[y1:y2, x1:x2]
    obj = (d > 0) & (plane_z > 0) & (d < plane_z - OBJECT_MIN_HEIGHT_M)
    if np.count_nonzero(obj) < 0.08 * obj.size:  # nothing stands out: keep the box
        return full
    m = np.zeros((h, w), np.uint8)
    m[y1:y2, x1:x2] = obj.astype(np.uint8) * 255
    return m > 0
