"""
Exports trained detectors to TensorRT FP16, into the same models/ folder:

    ros2 run karis_perception export_trt best8s.pt              # -> models/best8s.engine
    ros2 run karis_perception export_trt RF-DETR_m.pth          # -> models/RF-DETR_m.trt
    ros2 run karis_perception export_trt best8s.pt RF-DETR_m.pth

YOLO (.pt): exported at its training size, in the camera's 4:3 shape (training size 640 ->
640x480, 1280 -> 1280x960), the same shape PyTorch uses on a 640x480 frame. --square for a
square engine. RF-DETR (.pth): at the model's own resolution, built in the RF-DETR Python.

A TensorRT engine only runs on the GPU and TensorRT version it was built with: build it on the
computer that runs the perception. Stop the Isaac helper first (building needs GPU memory).
Then use it like any model: perception.launch.py model:=best8s.engine
"""
from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
from pathlib import Path

from karis_perception import trt_support

HERE = Path(__file__).resolve().parent


def find_models_dir(karis_dir):
    from karis_perception.pose_estimator_node import find_karis_dir
    kd = find_karis_dir(karis_dir or "")
    return kd, kd / "models"


def resolve(models_dir: Path, name: str) -> Path:
    p = Path(name)
    if p.exists():
        return p.resolve()
    if (models_dir / name).exists():
        return (models_dir / name).resolve()
    raise FileNotFoundError(f"No model '{name}' (looked in {models_dir})")


def export_yolo(path: Path, square: bool, fp32: bool, aspect: float):
    from ultralytics import YOLO
    m = YOLO(str(path))
    size = trt_support.yolo_train_imgsz(m) or 640
    imgsz = [size, size] if square else [int(math.ceil(size * aspect / 32) * 32), size]
    print(f"\n=== {path.name}: YOLO -> TensorRT {'FP32' if fp32 else 'FP16'}, input {imgsz[1]}x{imgsz[0]} "
          f"(trained at {size}) ===", flush=True)
    out = Path(m.export(format="engine", imgsz=imgsz, half=not fp32, device=0, batch=1,
                        simplify=True, verbose=False))
    print(f"Wrote {out} ({out.stat().st_size / 1e6:.0f} MB)")
    return out


def export_rfdetr(path: Path, karis_dir: Path, fp32: bool):
    py = next((p for p in [os.environ.get("KARIS_RFDETR_PYTHON", ""), "/opt/rfdetr-venv/bin/python",
                           "/opt/conda/envs/rfdetr/bin/python"] if p and Path(p).exists()), None)
    if py is None:
        raise RuntimeError("No RF-DETR Python found (KARIS_RFDETR_PYTHON)")
    out = path.with_suffix(".trt")
    print(f"\n=== {path.name}: RF-DETR -> TensorRT {'FP32' if fp32 else 'FP16'} ({py}) ===", flush=True)
    cmd = [py, str(HERE / "rfdetr_trt_export.py"), "--model", str(path), "--out", str(out)]
    cmd += ["--fp32"] if fp32 else []
    if subprocess.call(cmd) != 0:
        raise RuntimeError(f"RF-DETR export of {path.name} failed (output above)")
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description="Export YOLO (.pt) / RF-DETR (.pth) models to TensorRT")
    ap.add_argument("models", nargs="+", help="file names in models/ (or paths)")
    ap.add_argument("--karis-dir", default="", help="data folder with models/ ('' = $KARIS_DIR)")
    ap.add_argument("--square", action="store_true", help="YOLO: square input instead of 4:3")
    ap.add_argument("--aspect", type=float, default=0.75, help="YOLO: height/width of the camera image")
    ap.add_argument("--fp32", action="store_true", help="FP32 instead of FP16")
    a = ap.parse_args(argv if argv is not None else sys.argv[1:])

    karis_dir, models_dir = find_models_dir(a.karis_dir)
    done, failed = [], []
    for name in a.models:
        try:
            path = resolve(models_dir, name)
            ext = path.suffix.lower()
            if ext == ".pt":
                done.append(export_yolo(path, a.square, a.fp32, a.aspect))
            elif ext in (".pth", ".ckpt"):
                done.append(export_rfdetr(path, karis_dir, a.fp32))
            else:
                raise ValueError(f"{path.name}: only .pt (YOLO) and .pth (RF-DETR) can be exported")
        except Exception as e:                                         # noqa: BLE001
            print(f"FAILED {name}: {e}", flush=True)
            failed.append(name)
    print("\nDone:", ", ".join(p.name for p in done) or "nothing")
    if done:
        print("Compare:  ros2 run karis_perception bench_detector "
              + " ".join(f"{p.stem}{'.pt' if p.suffix == '.engine' else '.pth'} {p.name}" for p in done))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
