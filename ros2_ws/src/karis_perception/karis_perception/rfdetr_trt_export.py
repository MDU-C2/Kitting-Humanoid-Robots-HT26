"""
Builds a TensorRT FP16 engine from an RF-DETR checkpoint (.pth). Run by `export_trt` in the
RF-DETR Python (KARIS_RFDETR_PYTHON); standalone (no ROS).

    /opt/rfdetr-venv/bin/python rfdetr_trt_export.py --model models/RF-DETR_m.pth \
        --karis-dir /home/kimsv/Kitting-Humanoid-Robots-HT26/local_data --out models/RF-DETR_m.trt

Writes RF-DETR_m.trt and RF-DETR_m.karis.json (class names, model type) next to it.
Needs: pip install -U "rfdetr[onnx]>=1.9" "tensorrt-cu12>=10.8,<11" polygraphy onnxconverter-common
      in that Python, and a free GPU (a few GB while building).
"""
import argparse
import json
import shutil
import sys
import time
from pathlib import Path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--karis-dir", default="", help="(unused, kept for older commands)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--fp32", action="store_true", help="build in FP32 instead of FP16")
    a = ap.parse_args()

    sys.path.insert(0, str(Path(__file__).resolve().parent))   # rfdetr_server.py next to this file
    from rfdetr_server import load_model, class_name_list
    import rfdetr

    src, out = Path(a.model).resolve(), Path(a.out).resolve()
    print(f"Loading {src.name} (rfdetr {getattr(rfdetr, '__version__', '?')})...", flush=True)
    model = load_model(src)
    names = class_name_list(model)
    build = out.parent / f".{out.stem}_trt_build"
    shutil.rmtree(build, ignore_errors=True)
    t0 = time.time()
    print(f"Building the TensorRT {'FP32' if a.fp32 else 'FP16'} engine (a few minutes)...", flush=True)
    engine = Path(model.export(format="tensorrt", output_dir=str(build), fp16=not a.fp32,
                               output_name=out.stem, verbose=False))
    shutil.move(str(engine), str(out))
    meta = {"source": src.name, "model_class": type(model).__name__, "names": names,
            "rfdetr": getattr(rfdetr, "__version__", "?"), "precision": "fp32" if a.fp32 else "fp16"}
    out.with_suffix(".karis.json").write_text(json.dumps(meta, indent=2))
    shutil.rmtree(build, ignore_errors=True)
    print(f"Wrote {out} ({out.stat().st_size / 1e6:.0f} MB) in {time.time() - t0:.0f} s; "
          f"{len(names)} classes {names}", flush=True)


if __name__ == "__main__":
    main()
