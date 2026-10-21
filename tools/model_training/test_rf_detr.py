"""
Offline evaluation of a trained checkpoint on the labelled test split:
1. Quantitative evaluation (box/mask mAP, per-class mask mAP, best F1 threshold)
2. Prediction images on the test set (visual check)
3. Batch-1 timing at the chosen confidence (closer to use on the robot)
Everything is saved to OUTPUT_DIR, including a summary JSON for the report.

Usage:
  python test_rf_detr.py
  python test_rf_detr.py --run karis_rfdetr_seg_m --conf 0.5
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rfdetr
import supervision as sv
import torch
from PIL import Image
from supervision.metrics import F1Score, MeanAveragePrecision, MetricTarget

# ============================================================
# CONFIGURATION
# ============================================================
PROJECT_ROOT = Path(__file__).resolve().parents[3]   # adjust if the script sits elsewhere
DATASET_ROOT = PROJECT_ROOT / "VisionAspect" / "Data" / "KARIS.v8i.coco-segmentation"
SPLIT = "test"           # falls back to "valid" if there is no test split

MODEL = "RFDETRSegMedium"    # must match the class used in training
RUN_NAME = "karis_rfdetr_seg_m"
RUNS_DIR = PROJECT_ROOT / "runs" / "rfdetr_seg_m"
RESOLUTION = 624         # must match RESOLUTION used in training (None = model default)
DEVICE = None            # None = cuda if available, else cpu

# None = use the confidence that maximizes mask F1 on the test set; or set a number
CONFIDENCE = None
FALLBACK_CONFIDENCE = 0.5
EVAL_THRESHOLD = 0.01    # low threshold so mAP sees the whole PR curve (like conf=0.001 in YOLO)

WARMUP_IMAGES = 5        # images excluded from the timing average (GPU warmup)


# ============================================================
# Helpers
# ============================================================
def resolve_device(device: str | None) -> str:
    return device or ("cuda" if torch.cuda.is_available() else "cpu")


def sync(device: str):
    if device.startswith("cuda"):
        torch.cuda.synchronize()


def imwrite_unicode(path: Path, img):
    """cv2.imwrite fails on Windows paths with non-ASCII characters (e.g. 'ä')."""
    ok, buf = cv2.imencode(path.suffix or ".jpg", img)
    if ok:
        buf.tofile(str(path))


def pred_names(model) -> dict[int, str]:
    """Predicted class id -> name. class_names is a dict or a list depending on version."""
    names = model.class_names
    if isinstance(names, dict):
        return {int(k): v for k, v in names.items()}
    return {i: n for i, n in enumerate(names)}


def remap(det: sv.Detections, mapping: dict[int, int], shape) -> sv.Detections:
    """Map predicted class ids onto the dataset's class ids (matched by name)."""
    if len(det):
        det = det[np.array([int(c) in mapping for c in det.class_id], dtype=bool)]
        det.class_id = np.array([mapping[int(c)] for c in det.class_id], dtype=int)
    if det.mask is None:
        det.mask = np.zeros((len(det), shape[0], shape[1]), dtype=bool)
    return det


def best_f1_confidence(preds, targets):
    """Sweep the confidence threshold and return the one with the highest mask F1@0.5."""
    try:
        best_t, best_f1 = None, -1.0
        for t in np.arange(0.05, 0.96, 0.05):
            filt = [p[p.confidence >= t] if len(p) else p for p in preds]
            f1 = F1Score(metric_target=MetricTarget.MASKS).update(filt, targets).compute().f1_50
            if f1 > best_f1:
                best_t, best_f1 = float(t), float(f1)
        return best_t, best_f1
    except Exception as e:  # supervision metric API differs between versions
        print(f"Could not compute the best threshold ({e})")
        return None, None


# ============================================================
# TESTING
# ============================================================
def test(run_name: str, conf_override: float | None, device: str):
    split = SPLIT if (DATASET_ROOT / SPLIT / "_annotations.coco.json").exists() else "valid"
    split_dir = DATASET_ROOT / split
    checkpoint = RUNS_DIR / run_name / "checkpoint_best_total.pth"
    output_dir = RUNS_DIR / "test" / run_name
    for p in (split_dir / "_annotations.coco.json", checkpoint):
        if not p.exists():
            raise FileNotFoundError(f"Could not find:\n{p}")
    if split != SPLIT:
        print(f"Warning: no '{SPLIT}' split, using '{split}'. The checkpoint was "
              f"selected on this same split, so the numbers are optimistic.")
    pred_dir = output_dir / "predictions"
    pred_dir.mkdir(parents=True, exist_ok=True)

    print(f"Loading model: {checkpoint}")
    res = {} if RESOLUTION is None else {"resolution": RESOLUTION}
    model = getattr(rfdetr, MODEL)(pretrain_weights=str(checkpoint), device=device, **res)

    ds = sv.DetectionDataset.from_coco(
        images_directory_path=str(split_dir),
        annotations_path=str(split_dir / "_annotations.coco.json"),
        force_masks=True,
    )
    # Match model classes to dataset classes by name, so id offsets can't mix them up
    gt_index = {name: i for i, name in enumerate(ds.classes)}
    mapping = {pid: gt_index[n] for pid, n in pred_names(model).items() if n in gt_index}
    if not mapping:
        raise RuntimeError("No class names in common between the model and the dataset.")

    # 1. Predict every image once at a low threshold; reused for mAP, F1, images and timing
    print(f"\nPredicting {len(ds)} images on the '{split}' split...")
    paths, images, preds, targets, times_ms = [], [], [], [], []
    for path, image, target in ds:
        pil = Image.open(path).convert("RGB")   # preloaded, so disk reads are not timed
        sync(device)
        t0 = time.perf_counter()
        det = model.predict(pil, threshold=EVAL_THRESHOLD)
        sync(device)
        times_ms.append((time.perf_counter() - t0) * 1000.0)

        paths.append(Path(path))
        images.append(image)
        preds.append(remap(det, mapping, image.shape[:2]))
        targets.append(target)

    # 2. Quantitative evaluation
    summary = {"model": MODEL, "run": run_name, "checkpoint": str(checkpoint), "split": split,
               "resolution": RESOLUTION, "n_images": len(ds)}
    try:
        box = MeanAveragePrecision(metric_target=MetricTarget.BOXES).update(preds, targets).compute()
        seg = MeanAveragePrecision(metric_target=MetricTarget.MASKS).update(preds, targets).compute()
        print(f"\nBox  mAP50: {box.map50:.3f}   mAP50-95: {box.map50_95:.3f}")
        print(f"Mask mAP50: {seg.map50:.3f}   mAP50-95: {seg.map50_95:.3f}")

        print("\nPer-class mask mAP50-95 (weakest first):")
        per_class = sorted(
            ((int(c), float(ap)) for c, ap in zip(seg.matched_classes, seg.ap_per_class.mean(axis=1))),
            key=lambda t: t[1],
        )
        for cid, ap in per_class:
            print(f"  {ds.classes[cid]:<25s} {ap:.3f}")

        summary.update(
            box_map50=float(box.map50), box_map50_95=float(box.map50_95),
            mask_map50=float(seg.map50), mask_map50_95=float(seg.map50_95),
            per_class_mask_map50_95={ds.classes[c]: ap for c, ap in per_class},
        )
    except Exception as e:  # supervision metric API differs between versions
        print(f"Could not compute mAP with this supervision version ({e})")

    best_conf, best_f1 = best_f1_confidence(preds, targets)
    if conf_override is not None:
        conf = conf_override
    elif best_conf is not None:
        conf = best_conf
        print(f"\nBest mask-F1 confidence: {best_conf:.2f} (F1 = {best_f1:.3f})")
    else:
        conf = FALLBACK_CONFIDENCE
        print(f"\nCould not compute the best threshold, using {conf}")
    print(f"Using confidence {conf:.2f} for prediction images")
    summary.update(best_conf=best_conf, best_mask_f1_50=best_f1, conf_used=conf)

    # 3. Prediction images at the chosen confidence
    print("\nSaving prediction images...")
    mask_ann = sv.MaskAnnotator()
    box_ann = sv.BoxAnnotator()
    label_ann = sv.LabelAnnotator()
    for path, image, det in zip(paths, images, preds):
        det = det[det.confidence >= conf] if len(det) else det
        labels = [f"{ds.classes[c]} {s:.2f}" for c, s in zip(det.class_id, det.confidence)]
        vis = mask_ann.annotate(image.copy(), det)
        vis = box_ann.annotate(vis, det)
        vis = label_ann.annotate(vis, det, labels=labels)
        imwrite_unicode(pred_dir / f"{path.stem}.jpg", vis)

    # 4. Timing (whole predict call: preprocessing, model and postprocessing, batch 1).
    #    Measured at EVAL_THRESHOLD; the threshold only filters outputs, so it barely
    #    changes the time for a DETR model.
    n_warm = WARMUP_IMAGES if len(times_ms) > WARMUP_IMAGES else 0
    timed = times_ms[n_warm:]
    avg_ms = float(np.mean(timed)) if timed else float("nan")
    print(f"\nTotal: {avg_ms:.1f} ms/image ({1000.0 / avg_ms:.1f} FPS), batch 1, "
          f"averaged over {len(timed)} images after {n_warm} warmup")
    summary.update(total_ms_per_image=avg_ms, speed_batch=1, n_timed_images=len(timed))

    # 5. Save summary for the report
    json_path = output_dir / "test_summary.json"
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    print("\nTesting complete.")
    print(f"Predictions:   {pred_dir}")
    print(f"Summary JSON:  {json_path}")
    return summary


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Test a trained RF-DETR Seg checkpoint on the KARIS test split.")
    ap.add_argument("--run", default=RUN_NAME, help="run folder under runs/rfdetr_seg_m")
    ap.add_argument("--conf", type=float, default=CONFIDENCE, help="override the F1-optimal threshold")
    ap.add_argument("--device", default=DEVICE, help="cuda, cuda:0 or cpu")
    args = ap.parse_args()

    test(args.run, args.conf, resolve_device(args.device))