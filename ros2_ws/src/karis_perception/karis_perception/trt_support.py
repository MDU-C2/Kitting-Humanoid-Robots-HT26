"""
TensorRT detectors for the KARIS perception, next to the PyTorch ones in detectors.py:

    best8s.pt       YOLO, PyTorch (Ultralytics)              -> detectors.YoloDetector
    best8s.engine   YOLO, TensorRT (Ultralytics export)      -> detectors.YoloDetector (Ultralytics runs it)
    RF-DETR_m.pth   RF-DETR, PyTorch (helper rfdetr_server.py, port 5601)
    RF-DETR_m.trt   RF-DETR, TensorRT (helper rfdetr_trt_server.py, port 5602)

Make the TensorRT files with `ros2 run karis_perception export_trt <model>`; compare them with
`ros2 run karis_perception bench_detector`.

    from karis_perception import trt_support
    trt_support.enable(detectors)                     # .engine / .trt in the model picker
    det = trt_support.load_detector(detectors, path, image_size, device, iou)
"""
from __future__ import annotations

import json
import socket
import subprocess
import time
from pathlib import Path

RFDETR_TRT_PORT = 5602
HERE = Path(__file__).resolve().parent


# ------------------------------------------------------------------ model files
def enable(detectors):
    """Lets detectors.choose_model list .engine and .trt files and name their kind."""
    detectors.MODEL_EXTS.update({".engine", ".trt"})
    if not getattr(detectors.model_kind, "_karis_trt", False):
        def model_kind(path: Path) -> str:
            ext = Path(path).suffix.lower()
            return {".pt": "YOLO", ".engine": "YOLO-TRT", ".trt": "RF-DETR-TRT"}.get(ext, "RF-DETR")
        model_kind._karis_trt = True
        detectors.model_kind = model_kind


def load_detector(detectors, path, image_size=0, device=0, iou=0.6, fallback_size=1280, log=print):
    """Loads any of the four kinds. image_size: YOLO input size, 0 = what the model was
    trained / exported at. Returns the detector (with .kind, .names, .predict, .warmup)."""
    enable(detectors)
    path = Path(path)
    ext = path.suffix.lower()
    if ext in (".pt", ".engine"):
        det = detectors.YoloDetector(path, fallback_size, device, iou)
        det.kind = "YOLO" if ext == ".pt" else "YOLO-TRT"
        set_yolo_size(det, path, image_size, fallback_size, log)
    elif ext == ".trt":
        det = RFDETRTRTDetector.create(detectors, path)
        log("Detector input size: fixed in the RF-DETR TensorRT engine")
    else:
        det = detectors.RFDETRDetector(path)
        log("Detector input size: set by the RF-DETR model itself")
    det.model_file = path.name
    return det


# ------------------------------------------------------------------ YOLO input size
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


def engine_imgsz(path):
    """[h, w] a YOLO TensorRT engine was exported for (Ultralytics writes its metadata as
    4-byte length + JSON in front of the engine), or None."""
    try:
        with open(path, "rb") as f:
            n = int.from_bytes(f.read(4), byteorder="little")
            if not 0 < n < 1_000_000:
                return None
            meta = json.loads(f.read(n).decode("utf-8"))
        sz = meta.get("imgsz")
        if isinstance(sz, int):
            return [sz, sz]
        return [int(sz[0]), int(sz[1])] if sz else None
    except (OSError, ValueError, UnicodeDecodeError, TypeError, IndexError):
        return None


def set_yolo_size(det, path, override=0, fallback=1280, log=print):
    """.engine: always the exported size (a TensorRT engine has fixed input dimensions).
    .pt: parameter image_size if > 0, else the training size, else the fallback size."""
    path = Path(path)
    if path.suffix.lower() == ".engine":
        sz = engine_imgsz(path)
        if sz is None:
            det.image_size = override or fallback
            log(f"YOLO TensorRT engine without size metadata: using {det.image_size} px "
                "(must equal the export size)")
        else:
            det.image_size = sz
            if override and override != max(sz):
                log(f"image_size {override} ignored: the engine was exported for {sz[1]}x{sz[0]}")
            log(f"YOLO input size {sz[1]}x{sz[0]} px (fixed in the TensorRT engine)")
        return
    trained = yolo_train_imgsz(det.model)
    if override and override > 0:
        det.image_size, src = override, "parameter image_size"
    elif trained:
        det.image_size, src = trained, "the model's training size"
    else:
        det.image_size, src = fallback, "the fallback size (not stored in the model)"
    note = f" (trained at {trained})" if trained and trained != det.image_size else ""
    log(f"YOLO input size {det.image_size} px from {src}{note}")


# ------------------------------------------------------------------ RF-DETR TensorRT
class RFDETRTRTDetector:
    """Same as detectors.RFDETRDetector (same socket protocol), but the helper runs the
    TensorRT engine (rfdetr_trt_server.py, from this package) on its own port, so a running
    PyTorch RF-DETR helper is never mistaken for it."""

    @staticmethod
    def create(detectors, path):
        base = detectors.RFDETRDetector

        class _Det(base):
            kind = "RF-DETR-TRT"

            def _try_connect(self):
                try:
                    s = socket.create_connection(("127.0.0.1", RFDETR_TRT_PORT), timeout=2)
                    s.settimeout(120)
                    return s
                except OSError:
                    return None

            def _connect_or_start(self):
                s = self._try_connect()
                if s:
                    print(f"  Using the RF-DETR TensorRT helper already running on port {RFDETR_TRT_PORT}")
                    return s
                py = next((p for p in detectors.RFDETR_PYTHONS if p and Path(p).exists()), None)
                if py is None:
                    raise RuntimeError("RF-DETR TensorRT needs the RF-DETR Python (KARIS_RFDETR_PYTHON); none found")
                log_path = Path(path).with_suffix(".server.log")
                log = open(log_path, "w")
                print(f"  Starting the RF-DETR TensorRT helper with {py} (log: {log_path})...")
                self.proc = subprocess.Popen(
                    [py, str(HERE / "rfdetr_trt_server.py"), "--engine", str(self.path),
                     "--port", str(RFDETR_TRT_PORT)], stdout=log, stderr=subprocess.STDOUT)
                import atexit
                atexit.register(self.close)
                t0 = time.time()
                while time.time() - t0 < 300:
                    if self.proc.poll() is not None:
                        raise RuntimeError(f"The RF-DETR TensorRT helper stopped; see {log_path}")
                    s = self._try_connect()
                    if s:
                        print(f"  RF-DETR TensorRT helper ready after {time.time() - t0:.0f} s")
                        return s
                    time.sleep(1)
                raise RuntimeError(f"The RF-DETR TensorRT helper did not start within 5 min; see {log_path}")

        return _Det(path)
