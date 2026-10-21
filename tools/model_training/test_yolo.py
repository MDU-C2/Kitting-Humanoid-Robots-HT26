"""
KARIS test script.

Offline evaluation of a trained model on the labelled test split:
1. Quantitative evaluation (box/mask mAP, per-class mask mAP, best F1 threshold)
2. Prediction images on the test set (visual check)
3. Batch-1 timing at the chosen confidence (closer to use on the robot)
Everything is saved to OUTPUT_DIR, including a summary JSON for the report.
"""
import json
import time
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

# ============================================================
# CONFIGURATION
# ============================================================
PROJECT_ROOT = Path(__file__).resolve().parents[3]
DATASET_ROOT = PROJECT_ROOT / "VisionAspect" / "Data" / "KARIS.v8i.yolo-from-coco"
DATA_YAML = DATASET_ROOT / "data.yaml"
TEST_IMAGES = DATASET_ROOT / "images" / "test"

RUN_NAME = "karis_yolo26s"                         # training run to test
RUN_DIR = PROJECT_ROOT / "VisionAspect" / "Models" / "YOLO" / "runs" / RUN_NAME
MODEL_PATH = RUN_DIR / "weights" / "best.pt"
OUTPUT_DIR = PROJECT_ROOT / "VisionAspect" / "Models" / "YOLO" / "runs" / "test" / RUN_NAME

IMAGE_SIZE = 640         # must match IMAGE_SIZE used in training
BATCH_SIZE = 16          # evaluation batch; val() needs a positive number (not -1)
DEVICE = None            # None = first GPU if available, else CPU

# None = use the confidence that maximizes mask F1 on the test set; or set a number
CONFIDENCE = None
FALLBACK_CONFIDENCE = 0.4
IOU = 0.6                # NMS IoU; ignored by NMS-free YOLO26 models

WARMUP_IMAGES = 5        # images excluded from the timing average (GPU warmup)
IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}


# ============================================================
# Helpers
# ============================================================
def imread_unicode(path: Path):
    """cv2.imread fails on Windows paths with non-ASCII characters (e.g. 'ä')."""
    data = np.fromfile(str(path), dtype=np.uint8)
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def best_f1_confidence(metrics):
    try:
        f1 = metrics.seg.f1_curve.mean(0)  # mean over classes, shape (1000,)
        i = int(f1.argmax())
        return float(metrics.seg.px[i]), float(f1[i])
    except Exception:  # attribute names differ between versions
        return None, None


# ============================================================
# TESTING
# ============================================================
def test():
    for p in (DATA_YAML, MODEL_PATH, TEST_IMAGES):
        if not p.exists():
            raise FileNotFoundError(f"Could not find:\n{p}")
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    print(f"Loading model: {MODEL_PATH}")
    model = YOLO(str(MODEL_PATH))

    # 1. Quantitative evaluation (conf=0.001 so mAP sees the whole PR curve)
    print("\nEvaluating model on test set...")
    metrics = model.val(
        data=str(DATA_YAML), split="test", imgsz=IMAGE_SIZE, batch=BATCH_SIZE,
        device=DEVICE, conf=0.001, project=str(OUTPUT_DIR), name="metrics",
        exist_ok=True, plots=True,
    )
    print(f"\nBox  mAP50: {metrics.box.map50:.3f}   mAP50-95: {metrics.box.map:.3f}")
    print(f"Mask mAP50: {metrics.seg.map50:.3f}   mAP50-95: {metrics.seg.map:.3f}")

    # m.seg.maps is indexed by class id; ap_class_index lists the classes present in the split
    print("\nPer-class mask mAP50-95 (weakest first):")
    per_class = sorted(
        ((int(c), float(metrics.seg.maps[int(c)])) for c in metrics.seg.ap_class_index),
        key=lambda t: t[1],
    )
    for idx, ap in per_class:
        print(f"  {metrics.names[idx]:<25s} {ap:.3f}")

    best_conf, best_f1 = best_f1_confidence(metrics)
    if CONFIDENCE is not None:
        conf = CONFIDENCE
    elif best_conf is not None:
        conf = best_conf
        print(f"\nBest mask-F1 confidence: {best_conf:.3f} (F1 = {best_f1:.3f})")
    else:
        conf = FALLBACK_CONFIDENCE
        print(f"\nCould not compute the best threshold, using {conf}")
    print(f"Using confidence {conf:.3f} for predictions and timing")

    # 2. Prediction images
    print("\nGenerating predictions...")
    model.predict(
        source=str(TEST_IMAGES), imgsz=IMAGE_SIZE, conf=conf, iou=IOU, device=DEVICE,
        show_labels=True, show_conf=True, boxes=True, save=True, verbose=False,
        project=str(OUTPUT_DIR), name="predictions", exist_ok=True,
    )

    # 3. Timing: one image at a time, images preloaded so disk reads are not counted
    print("\nTiming batch-1 inference...")
    images = sorted(p for p in TEST_IMAGES.iterdir() if p.suffix.lower() in IMAGE_EXTS)
    frames = [f for f in (imread_unicode(p) for p in images) if f is not None]

    times_ms, speeds = [], []
    for frame in frames:
        t0 = time.perf_counter()
        r = model.predict(frame, imgsz=IMAGE_SIZE, conf=conf, iou=IOU,
                          device=DEVICE, verbose=False)[0]
        times_ms.append((time.perf_counter() - t0) * 1000.0)
        speeds.append(r.speed)

    n_warm = WARMUP_IMAGES if len(times_ms) > WARMUP_IMAGES else 0
    timed, timed_speeds = times_ms[n_warm:], speeds[n_warm:]
    avg_ms = float(np.mean(timed)) if timed else float("nan")
    avg_parts = {k: float(np.mean([s[k] for s in timed_speeds]))
                 for k in ("preprocess", "inference", "postprocess")} if timed_speeds else {}
    print(f"Speed: {avg_parts.get('preprocess', 0):.1f} ms pre, "
          f"{avg_parts.get('inference', 0):.1f} ms inference, "
          f"{avg_parts.get('postprocess', 0):.1f} ms post")
    print(f"Total: {avg_ms:.1f} ms/image ({1000.0 / avg_ms:.1f} FPS), batch 1, "
          f"averaged over {len(timed)} images after {n_warm} warmup")

    # 4. Save summary for the report
    summary = {
        "model": str(MODEL_PATH), "run": RUN_NAME, "split": "test", "imgsz": IMAGE_SIZE,
        "box_map50": float(metrics.box.map50), "box_map50_95": float(metrics.box.map),
        "mask_map50": float(metrics.seg.map50), "mask_map50_95": float(metrics.seg.map),
        "per_class_mask_map50_95": {metrics.names[i]: ap for i, ap in per_class},
        "best_conf": best_conf, "best_mask_f1": best_f1, "conf_used": conf,
        "val_speed_ms": {k: float(v) for k, v in metrics.speed.items()}, "val_batch": BATCH_SIZE,
        "speed_ms": avg_parts, "total_ms_per_image": avg_ms, "speed_batch": 1,
        "n_timed_images": len(timed),
    }
    json_path = OUTPUT_DIR / "test_summary.json"
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    print("\nTesting complete.")
    print(f"Metrics + curves:  {OUTPUT_DIR / 'metrics'}")
    print(f"Predictions:       {OUTPUT_DIR / 'predictions'}")
    print(f"Summary JSON:      {json_path}")
    return metrics


if __name__ == "__main__":
    test()