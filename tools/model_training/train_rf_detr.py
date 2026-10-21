"""
Train and evaluate RF-DETR Seg on the KARIS dataset (COCO segmentation export).

Mirrors the YOLO train.py: config block at the top, runs saved under
KARIS_2/runs, and an evaluation on the test split (falls back to valid) that
prints mask mAP, per-class AP, speed and the F1-optimal confidence threshold.

Usage:
  python train_rf_detr.py                         # train + evaluate
  python train_rf_detr.py --subset-samples 4      # quick smoke test
  python train_rf_detr.py --eval-only             # evaluate an existing run
  python train_rf_detr.py --resume runs/rfdetr_seg_m/<run>/last.ckpt   # continue a stopped run

Ctrl+C during training stops it, keeps the best checkpoint saved so far and
continues with the evaluation. Press Ctrl+C again to skip the evaluation.
"""
from __future__ import annotations

import argparse
import json
import shutil
import time
from pathlib import Path
from typing import Iterable

import numpy as np
import rfdetr
import supervision as sv
import torch
from supervision.metrics import F1Score, MeanAveragePrecision, MetricTarget

# CONFIG
PROJECT_ROOT = Path(__file__).resolve().parents[3]
DATASET_ROOT = PROJECT_ROOT / "VisionAspect" / "Data" / "KARIS.v8i.coco-segmentation"
RUNS_DIR = PROJECT_ROOT / "VisionAspect" / "Models" / "RF_DETR" / "runs"

# Model sizes: RFDETRSegNano / RFDETRSegSmall / RFDETRSegMedium
MODEL = "RFDETRSegSmall"
RUN_NAME = "rfdetr_seg_s"

# Training parameters
EPOCHS = 50
PATIENCE = 20            # early stopping: epochs without validation mAP improvement
BATCH_SIZE = 1           # lower if you run out of GPU memory
GRAD_ACCUM_STEPS = 16     # effective batch = BATCH_SIZE * GRAD_ACCUM_STEPS (Roboflow recommends 16)
LR = 1e-4
RESOLUTION = 624         # square input size. Seg models need a multiple of 24 (SegNano: 12).
                         # 624 is the nearest valid size to the 640 px camera width. None = model default
DEVICE = None            # None = cuda if available, else cpu. Check the device line in the log
WORKERS = 0              # dataloader workers (0 is safest on Windows)
USE_EMA = True           # smoothed weights, usually a bit better
SEED = 0                 # fixed seed so runs are comparable
COS_LR = True            # cosine learning rate schedule (False = RF-DETR's default step schedule)
TENSORBOARD = True       # loss/mAP curves per epoch; needs: pip install "rfdetr[loggers]"
                         # view with: tensorboard --logdir runs/rfdetr_seg_m
SUBSET_SAMPLES = 0       # > 0 = tiny smoke-test subset with this many training images

# Evaluation
EVAL_THRESHOLD = 0.01    # low threshold so mAP sees the whole PR curve (like conf=0.001 in YOLO)
WARMUP_IMAGES = 5        # images excluded from the speed average (GPU warmup)


def resolve_device(device: str | None = None) -> str:
    if device:
        return device
    return "cuda" if torch.cuda.is_available() else "cpu"


# ----------------------------------------------------------------------------
# Smoke-test subset
# ----------------------------------------------------------------------------
def _filter_annotations(ann_data: dict, image_ids: Iterable[int]) -> dict:
    chosen = set(image_ids)
    return {
        "info": ann_data.get("info", {}),
        "licenses": ann_data.get("licenses", []),
        "categories": ann_data.get("categories", []),
        "images": [img for img in ann_data.get("images", []) if img["id"] in chosen],
        "annotations": [a for a in ann_data.get("annotations", []) if a.get("image_id") in chosen],
    }


def create_smoke_subset(dataset_root: Path, output_dir: Path, n_train: int) -> Path:
    """Copy the first few images of each split into output_dir/smoke_subset."""
    smoke_root = output_dir / "smoke_subset"
    limits = {"train": n_train, "valid": max(1, n_train // 2), "test": max(1, n_train // 2)}

    for split, limit in limits.items():
        ann_path = dataset_root / split / "_annotations.coco.json"
        if not ann_path.exists():
            continue
        with ann_path.open("r", encoding="utf-8") as f:
            ann_data = json.load(f)

        image_ids = [img["id"] for img in ann_data.get("images", [])[:limit]]
        split_dir = smoke_root / split
        split_dir.mkdir(parents=True, exist_ok=True)

        filtered = _filter_annotations(ann_data, image_ids)
        for img in filtered["images"]:
            src = dataset_root / split / img["file_name"]
            if src.exists():
                shutil.copy2(src, split_dir / Path(img["file_name"]).name)
            img["file_name"] = Path(img["file_name"]).name

        with (split_dir / "_annotations.coco.json").open("w", encoding="utf-8") as f:
            json.dump(filtered, f)

    return smoke_root


# ----------------------------------------------------------------------------
# Training
# ----------------------------------------------------------------------------
# Best-model files RF-DETR writes during training, in order of preference.
# checkpoint_best_total.pth may only be written when training ends normally; the
# EMA/regular "best" files are updated every time validation mAP improves.
BEST_CANDIDATES = ("checkpoint_best_total.pth", "checkpoint_best_ema.pth",
                   "checkpoint_best_regular.pth")


def secure_best_checkpoint(output_dir: Path, since: float = 0.0) -> Path | None:
    """Make sure checkpoint_best_total.pth holds the best model saved so far.
    Only files written after `since` (a time.time() value) are considered, so a
    leftover file from an older run in the same folder is never picked up."""
    target = output_dir / BEST_CANDIDATES[0]
    found = [output_dir / n for n in BEST_CANDIDATES
             if (output_dir / n).exists() and (output_dir / n).stat().st_mtime >= since]
    if not found:
        return None
    # newest best file = latest improvement in validation mAP
    src = max(found, key=lambda p: p.stat().st_mtime)
    if src != target:
        shutil.copy2(src, target)
        print(f"Copied {src.name} -> {target.name}")
    return target


def train(dataset_root: Path, output_dir: Path, epochs: int, batch_size: int, device: str,
          resume: Path | None = None) -> Path:
    if not (dataset_root / "train" / "_annotations.coco.json").exists():
        raise FileNotFoundError(f"COCO train annotations not found in:\n{dataset_root / 'train'}")

    print("Starting training")
    print(f"Model:   {MODEL}")
    print(f"Dataset: {dataset_root}")
    print(f"Epochs:  {epochs} (patience {PATIENCE})")
    print(f"Batch:   {batch_size} x {GRAD_ACCUM_STEPS} accumulation = {batch_size * GRAD_ACCUM_STEPS}")
    print(f"Device:  {device}")

    model = getattr(rfdetr, MODEL)()
    if RESOLUTION is not None and RESOLUTION % 24 and MODEL != "RFDETRSegNano":
        raise ValueError(f"RESOLUTION={RESOLUTION} is not a multiple of 24 (try 624 or 648)")
    extra = {} if RESOLUTION is None else {"resolution": RESOLUTION}
    if resume is not None:
        if not resume.exists():
            raise FileNotFoundError(f"Resume checkpoint not found:\n{resume}")
        extra["resume"] = str(resume)
        print(f"Resuming from: {resume}")
    print("Press Ctrl+C to stop early; the best model so far is kept and evaluated.\n")

    started = time.time()
    try:
        _run_training(model, dataset_root, output_dir, epochs, batch_size, device, extra)
        interrupted = False
    except (KeyboardInterrupt, SystemExit):
        # Ctrl+C. PyTorch Lightning may turn it into SystemExit after shutting down.
        interrupted = True
        print("\nTraining interrupted. Keeping the best checkpoint saved so far...")

    # After a resume, best files from before the resume are still valid
    checkpoint = secure_best_checkpoint(output_dir, since=0.0 if resume else started)
    if checkpoint is None:
        msg = f"No best checkpoint found in {output_dir}"
        if interrupted:
            raise SystemExit(msg + " (stopped before the first validation finished).")
        raise FileNotFoundError(msg)

    status = "stopped early" if interrupted else "finished"
    print(f"Training {status}. Best checkpoint: {checkpoint}")
    last = output_dir / "last.ckpt"
    if interrupted and last.exists():
        print(f"To continue training later: python {Path(__file__).name} --resume \"{last}\"")
    return checkpoint


def _run_training(model, dataset_root, output_dir, epochs, batch_size, device, extra):
    model.train(
        dataset_dir=str(dataset_root),
        output_dir=str(output_dir),
        epochs=epochs,
        batch_size=batch_size,
        eval_batch_size=batch_size,
        grad_accum_steps=GRAD_ACCUM_STEPS,
        lr=LR,
        device=device,
        accelerator="gpu" if device.startswith("cuda") else "cpu",
        num_workers=WORKERS,
        use_ema=USE_EMA,
        seed=SEED,
        lr_scheduler="cosine" if COS_LR else "step",
        early_stopping=True,
        early_stopping_patience=PATIENCE,
        tensorboard=TENSORBOARD,
        progress_bar="tqdm",
        **extra,
    )


# ----------------------------------------------------------------------------
# Evaluation
# ----------------------------------------------------------------------------
def _pred_names(model) -> dict[int, str]:
    """Predicted class id -> name. class_names is a dict or a list depending on version."""
    names = model.class_names
    if isinstance(names, dict):
        return {int(k): v for k, v in names.items()}
    return {i: n for i, n in enumerate(names)}


def _remap(det: sv.Detections, mapping: dict[int, int], shape) -> sv.Detections:
    """Map predicted class ids onto the dataset's class ids (matched by name)."""
    if len(det):
        det = det[np.array([int(c) in mapping for c in det.class_id], dtype=bool)]
        det.class_id = np.array([mapping[int(c)] for c in det.class_id], dtype=int)
    if det.mask is None:
        det.mask = np.zeros((len(det), shape[0], shape[1]), dtype=bool)
    return det


def evaluate(checkpoint: Path, dataset_root: Path, device: str, out_dir: Path):
    """Evaluate on the test split (falls back to valid): mask/box mAP, per-class AP,
    speed and the confidence threshold that maximizes mask F1@0.5."""
    split = "test" if (dataset_root / "test" / "_annotations.coco.json").exists() else "valid"
    split_dir = dataset_root / split
    print(f"\nEvaluating {checkpoint.name} on the '{split}' split")
    if split == "valid":
        print("Warning: no test split. These are validation numbers, "
              "and the checkpoint was selected on this same split.")

    res = {} if RESOLUTION is None else {"resolution": RESOLUTION}
    model = getattr(rfdetr, MODEL)(pretrain_weights=str(checkpoint), device=device, **res)
    ds = sv.DetectionDataset.from_coco(
        images_directory_path=str(split_dir),
        annotations_path=str(split_dir / "_annotations.coco.json"),
        force_masks=True,
    )

    # Match model classes to dataset classes by name, so id offsets can't mix them up
    gt_index = {name: i for i, name in enumerate(ds.classes)}
    mapping = {pid: gt_index[n] for pid, n in _pred_names(model).items() if n in gt_index}
    if not mapping:
        raise RuntimeError("No class names in common between the model and the dataset.")

    summary = {
        "model": MODEL, "run": RUN_NAME, "checkpoint": str(checkpoint), "split": split,
        "n_images": len(ds), "resolution": RESOLUTION, "eval_threshold": EVAL_THRESHOLD,
    }

    preds, targets, times_ms = [], [], []
    for path, image, target in ds:
        if device.startswith("cuda"):
            torch.cuda.synchronize()
        t0 = time.perf_counter()
        det = model.predict(path, threshold=EVAL_THRESHOLD)
        if device.startswith("cuda"):
            torch.cuda.synchronize()
        times_ms.append((time.perf_counter() - t0) * 1000.0)

        preds.append(_remap(det, mapping, image.shape[:2]))
        targets.append(target)

    # Accuracy
    try:
        box = MeanAveragePrecision(metric_target=MetricTarget.BOXES).update(preds, targets).compute()
        seg = MeanAveragePrecision(metric_target=MetricTarget.MASKS).update(preds, targets).compute()
        print(f"Box  mAP50: {box.map50:.3f}   mAP50-95: {box.map50_95:.3f}")
        print(f"Mask mAP50: {seg.map50:.3f}   mAP50-95: {seg.map50_95:.3f}")

        print("\nPer-class mask mAP50-95 (weakest first):")
        per_class = sorted(zip(seg.matched_classes, seg.ap_per_class.mean(axis=1)), key=lambda t: t[1])
        for cid, ap in per_class:
            print(f"  {ds.classes[int(cid)]:<25s} {ap:.3f}")

        summary.update(
            box_map50=float(box.map50), box_map50_95=float(box.map50_95),
            mask_map50=float(seg.map50), mask_map50_95=float(seg.map50_95),
            per_class_mask_map50_95={ds.classes[int(c)]: float(ap) for c, ap in per_class},
        )
    except Exception as e:  # supervision metric API differs between versions
        print(f"Could not compute mAP with this supervision version ({e}).")

    # Speed (whole predict call: preprocessing, model and postprocessing, batch 1)
    timed = times_ms[WARMUP_IMAGES:] if len(times_ms) > WARMUP_IMAGES else times_ms
    avg = float(np.mean(timed))
    print(f"\nSpeed: {avg:.1f} ms/image ({1000.0 / avg:.1f} FPS), batch 1, "
          f"averaged over {len(timed)} images after {len(times_ms) - len(timed)} warmup")
    summary.update(speed_ms_per_image=avg, speed_fps=1000.0 / avg, speed_batch=1)

    # Best confidence threshold
    try:
        best_t, best_f1 = 0.0, -1.0
        for t in np.arange(0.05, 0.96, 0.05):
            filt = [p[p.confidence >= t] if len(p) else p for p in preds]
            f1 = F1Score(metric_target=MetricTarget.MASKS).update(filt, targets).compute().f1_50
            if f1 > best_f1:
                best_t, best_f1 = float(t), float(f1)
        print(f"\nBest confidence threshold (mask F1@0.5): {best_t:.2f}  (F1 = {best_f1:.3f})")
        print("Use this as the threshold in the RF-DETR inference script")
        summary.update(best_conf=best_t, best_mask_f1_50=best_f1)
    except Exception as e:
        print(f"\nCould not compute the best threshold automatically ({e}).")

    out_file = out_dir / f"eval_{split}.json"
    with out_file.open("w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
    print(f"\nSaved evaluation results to {out_file}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Train and evaluate RF-DETR Seg on the KARIS dataset.")
    ap.add_argument("--dataset-root", type=Path, default=DATASET_ROOT)
    ap.add_argument("--epochs", type=int, default=EPOCHS)
    ap.add_argument("--batch-size", type=int, default=BATCH_SIZE)
    ap.add_argument("--device", type=str, default=DEVICE, help="cuda, cuda:0 or cpu")
    ap.add_argument("--subset-samples", type=int, default=SUBSET_SAMPLES,
                    help="Tiny smoke-test subset. 0 = full dataset.")
    ap.add_argument("--eval-only", action="store_true",
                    help="Skip training and evaluate the run's existing best checkpoint.")
    ap.add_argument("--resume", type=Path, default=None,
                    help="Continue an interrupted run from its last.ckpt.")
    args = ap.parse_args()

    device = resolve_device(args.device)
    out_dir = (RUNS_DIR / RUN_NAME).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    data_root = args.dataset_root.resolve()
    if args.subset_samples > 0:
        data_root = create_smoke_subset(data_root, out_dir, args.subset_samples)

    if args.eval_only:
        ckpt = secure_best_checkpoint(out_dir)
        if ckpt is None:
            raise FileNotFoundError(f"No best checkpoint to evaluate in:\n{out_dir}")
    else:
        ckpt = train(data_root, out_dir, args.epochs, args.batch_size, device, args.resume)

    evaluate(ckpt, data_root, device, out_dir)