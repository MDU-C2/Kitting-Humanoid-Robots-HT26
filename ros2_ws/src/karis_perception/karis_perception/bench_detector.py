"""
Compares detectors on the same images: time per detection (what the pose estimator waits for)
and whether they find the same parts (against the first model given).

    ros2 run karis_perception bench_detector best8s.pt best8s.engine
    ros2 run karis_perception bench_detector RF-DETR_m.pth RF-DETR_m.trt --runs 100
    ros2 run karis_perception bench_detector best8s.pt best8s.engine --images $KARIS_DIR/data/rgb

Images: the photos in $KARIS_DIR/data (default), resized to the camera size
(640x480) like the live frames. Stop the Isaac helper first for clean numbers (same GPU).
"""
from __future__ import annotations

import argparse
import gc
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from karis_perception import trt_support

IMG_EXTS = {".png", ".jpg", ".jpeg", ".bmp"}


def load_images(folder: Path, size, limit):
    files = sorted(p for p in folder.rglob("*") if p.suffix.lower() in IMG_EXTS
                   and "depth" not in p.parts and "mask" not in p.name.lower())
    if not files:
        raise FileNotFoundError(f"No images in {folder}")
    step = max(1, len(files) // limit)
    imgs = []
    for p in files[::step][:limit]:
        im = cv2.imread(str(p), cv2.IMREAD_COLOR)
        if im is None:
            continue
        if size and (im.shape[1], im.shape[0]) != tuple(size):
            im = cv2.resize(im, tuple(size), interpolation=cv2.INTER_AREA)
        imgs.append(im)
    return imgs


def box_iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def compare(ref, dets):
    """Greedy match by class and box IoU >= 0.5. Returns (matched, ref count, extra, box IoUs, mask IoUs)."""
    used, biou, miou = set(), [], []
    for c, _, rb, rm in sorted(ref, key=lambda d: -d[1]):
        best, bi = None, 0.5
        for j, (c2, _, b2, _) in enumerate(dets):
            if j in used or c2 != c:
                continue
            v = box_iou(rb, b2)
            if v >= bi:
                best, bi = j, v
        if best is None:
            continue
        used.add(best)
        biou.append(bi)
        m2 = dets[best][3]
        if rm is not None and m2 is not None and rm.shape == m2.shape:
            u = np.logical_or(rm, m2).sum()
            miou.append(np.logical_and(rm, m2).sum() / u if u else 1.0)
    return len(used), len(ref), len(dets) - len(used), biou, miou


def free(det):
    if hasattr(det, "close"):
        det.close()
    if hasattr(det, "model"):
        det.model = None
    gc.collect()
    try:
        import torch
        torch.cuda.empty_cache()
    except Exception:                                                 # noqa: BLE001
        pass


def main(argv=None):
    ap = argparse.ArgumentParser(description="Time and compare detectors (PyTorch vs TensorRT)")
    ap.add_argument("models", nargs="+", help="file names in models/ (.pt .engine .pth .trt)")
    ap.add_argument("--images", default="", help="image folder ('' = $KARIS_DIR/data)")
    ap.add_argument("--runs", type=int, default=60, help="timed detections per model")
    ap.add_argument("--max-images", type=int, default=30)
    ap.add_argument("--conf", type=float, default=0.6)
    ap.add_argument("--image-size", type=int, default=0, help="YOLO .pt input size (0 = as trained)")
    ap.add_argument("--camera-size", default="640x480", help="resize images to this ('' = keep)")
    ap.add_argument("--karis-dir", default="")
    a = ap.parse_args(argv if argv is not None else sys.argv[1:])

    from karis_perception.pose_estimator_node import find_karis_dir
    karis_dir = find_karis_dir(a.karis_dir)
    from karis_perception import detectors
    trt_support.enable(detectors)
    fallback, iou = 640, 0.6
    size = tuple(int(v) for v in a.camera_size.lower().split("x")) if a.camera_size else None
    imgs = load_images(Path(a.images) if a.images else karis_dir / "data", size, a.max_images)
    print(f"{len(imgs)} images ({imgs[0].shape[1]}x{imgs[0].shape[0]}), {a.runs} timed detections per model, "
          f"confidence {a.conf}\n")

    results, ref = [], None
    for name in a.models:
        path = Path(name) if Path(name).exists() else karis_dir / "models" / name
        print(f"--- {path.name}")
        try:
            det = trt_support.load_detector(detectors, path, a.image_size, 0, iou, fallback_size=fallback,
                                            log=lambda s: print("  " + s))
        except Exception as e:                                         # noqa: BLE001
            print(f"  could not load: {e}\n")
            continue
        for i in range(5):                                             # warm-up (CUDA, TensorRT, cuDNN)
            det.predict(imgs[i % len(imgs)], a.conf)
        outs = [det.predict(im, a.conf) for im in imgs]               # for the comparison
        ms, inf = [], []
        for i in range(a.runs):
            tm = {}
            t0 = time.perf_counter()
            det.predict(imgs[i % len(imgs)], a.conf, tm)
            ms.append((time.perf_counter() - t0) * 1000)
            if tm.get("det_inf_ms") is not None:
                inf.append(tm["det_inf_ms"])
        r = {"name": path.name, "kind": det.kind, "ms": np.median(ms), "p90": np.percentile(ms, 90),
             "inf": np.median(inf) if inf else float("nan"),
             "dets": np.mean([len(o) for o in outs]), "outs": outs}
        if ref is None:
            ref = r
        else:
            m = [compare(x, y) for x, y in zip(ref["outs"], outs)]
            matched, total, extra = sum(v[0] for v in m), sum(v[1] for v in m), sum(v[2] for v in m)
            biou = [x for v in m for x in v[3]]
            miou = [x for v in m for x in v[4]]
            r["agree"] = (f"{matched}/{total} of {ref['name']}'s parts found, {extra} extra, "
                          f"box IoU {np.mean(biou) if biou else float('nan'):.3f}, "
                          f"mask IoU {np.mean(miou) if miou else float('nan'):.3f}")
        print(f"  {r['ms']:.1f} ms per detection (median), p90 {r['p90']:.1f} ms; "
              f"model inference {r['inf']:.1f} ms; {r['dets']:.1f} parts per image\n")
        results.append(r)
        free(det)

    if not results:
        return 1
    base = results[0]["ms"]
    print(f"{'model':<28s} {'kind':<12s} {'median ms':>9s} {'p90 ms':>7s} {'infer ms':>8s} {'speed-up':>8s}  parts/img")
    for r in results:
        print(f"{r['name']:<28s} {r['kind']:<12s} {r['ms']:9.1f} {r['p90']:7.1f} {r['inf']:8.1f} "
              f"{base / r['ms']:7.2f}x  {r['dets']:.1f}")
    for r in results[1:]:
        print(f"{r['name']}: {r['agree']}")
    print("\nmedian ms = what the pose estimator waits for one detection (incl. pre/post-processing; "
          "RF-DETR incl. the transfer to its helper). Agreement close to all parts, box IoU > 0.9 and "
          "mask IoU > 0.85 means FP16 did not change the results noticeably.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
