from ultralytics import YOLO
from pathlib import Path
import yaml

# CONFIG
PROJECT_ROOT = Path(__file__).resolve().parents[3]
DATASET_ROOT = PROJECT_ROOT / "VisionAspect" / "Data" / "KARIS.v8i.yolo-from-coco"
DATA_YAML = DATASET_ROOT / "data.yaml"
RUNS_DIR = PROJECT_ROOT / "VisionAspect" / "Models" / "YOLO" / "runs"

# Model sizes: n = nano, s = small, m = medium, l = large, x = x-large
MODEL = "yolov8n-seg.pt"        # yolov8n-seg.pt / yolo26n-seg.pt
RUN_NAME = "karis_yolov8n"      # karis_yolov8n / karis_yolo26n

# Training parameters
EPOCHS = 50
PATIENCE = 20           # stop early if validation does not improve for N epochs
IMAGE_SIZE = 640        # Size of the images
BATCH_SIZE = -1         # -1 = auto (largest batch that fits ~60% of GPU memory, single GPU only)
EVAL_BATCH = 8          # batch size for evaluation
DEVICE = 0              # None = first GPU if available, else CPU. Check the device line in the log
WORKERS = 4             # dataloader workers (lower to 2 or 0 if Windows complains)
CACHE = False           # True/"ram" is faster; at 640 px this usually fits in RAM
SEED = 0                # fixed seed so runs are comparable
COS_LR = True           # cosine learning rate schedule
CLOSE_MOSAIC = 10       # turn mosaic off for the last N epochs

AUGMENTATION_ON = True
# Color changes
HSV_H = 0.015
HSV_S = 0.4
HSV_V = 0.3

# Geometry changes
DEGREES = 15.0          # in-plane rotation; parts can lie at any angle on the table
TRANSLATE = 0.15
SCALE = 0.3
PERSPECTIVE = 0.0005    # small viewpoint changes (the head camera views the table at an angle)

# Flips: parts are mirror-symmetric, so flipped images are still realistic
FLIPLR = 0.2
FLIPUD = 0.2

# Compositing helps a lot for cluttered scenes
MOSAIC = 1.0
MIXUP = 0.0
COPY_PASTE = 0.3  # segmentation only: pastes object instances into other images


def build_aug_args() -> dict:
    if AUGMENTATION_ON:
        return dict(
            hsv_h=HSV_H, hsv_s=HSV_S, hsv_v=HSV_V,
            degrees=DEGREES, translate=TRANSLATE, scale=SCALE,
            perspective=PERSPECTIVE, fliplr=FLIPLR, flipud=FLIPUD,
            mosaic=MOSAIC, mixup=MIXUP, copy_paste=COPY_PASTE,
        )
    return dict(
        hsv_h=0.0, hsv_s=0.0, hsv_v=0.0,
        degrees=0.0, translate=0.0, scale=0.0, perspective=0.0,
        fliplr=0.0, flipud=0.0, mosaic=0.0, mixup=0.0, copy_paste=0.0,
    )


def train():
    if not DATA_YAML.exists():
        raise FileNotFoundError(f"Could not find dataset configuration:\n{DATA_YAML}")

    print("Starting training")
    print(f"Model:   {MODEL}")
    print(f"Dataset: {DATA_YAML}")
    print(f"Epochs:  {EPOCHS} (patience {PATIENCE})")
    print(f"Image:   {IMAGE_SIZE}")
    print(f"Batch:   {BATCH_SIZE}")
    print(f"Augmentation: {'on' if AUGMENTATION_ON else 'off'}")

    model = YOLO(MODEL)
    results = model.train(
        data=str(DATA_YAML),
        epochs=EPOCHS,
        patience=PATIENCE,
        imgsz=IMAGE_SIZE,
        batch=BATCH_SIZE,
        device=DEVICE,
        workers=WORKERS,
        cache=CACHE,
        seed=SEED,
        cos_lr=COS_LR,
        close_mosaic=CLOSE_MOSAIC,
        project=str(RUNS_DIR),
        name=RUN_NAME,
        **build_aug_args(),
    )
    return results


def evaluate(save_dir: Path):
    """Evaluate best.pt on the test split (falls back to val), print per-class
    mask metrics, speed and the confidence threshold that maximizes F1."""
    best = Path(save_dir) / "weights" / "best.pt"
    model = YOLO(str(best))

    with open(DATA_YAML, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    split = "test" if cfg.get("test") else "val"
    print(f"\nEvaluating {best.name} on the '{split}' split")
    if split == "val":
        print("Warning: no test split in data.yaml. These are validation numbers, "
              "and best.pt was selected on this same split.")

    m = model.val(
        data=str(DATA_YAML), split=split, imgsz=IMAGE_SIZE,
        batch=EVAL_BATCH, device=DEVICE, conf=0.001, plots=True,
        project=str(RUNS_DIR), name=f"{RUN_NAME}_eval_{split}",
    )

    print(f"Box  mAP50: {m.box.map50:.3f}   mAP50-95: {m.box.map:.3f}")
    print(f"Mask mAP50: {m.seg.map50:.3f}   mAP50-95: {m.seg.map:.3f}")

    # Speed in ms per image, measured during this evaluation (batch BATCH_SIZE, conf=0.001).
    # Postprocessing is inflated by the low conf; time infer.py at batch 1 for real figures.
    s = m.speed
    total = s["preprocess"] + s["inference"] + s["postprocess"]
    print(f"Speed: {s['preprocess']:.1f} ms pre, {s['inference']:.1f} ms inference, "
          f"{s['postprocess']:.1f} ms post (total {total:.1f} ms/image)")

    # m.seg.maps is indexed by class id; ap_class_index lists the classes present in the split
    print("\nPer-class mask mAP50-95 (weakest first):")
    per_class = sorted(
        ((int(c), m.seg.maps[int(c)]) for c in m.seg.ap_class_index),
        key=lambda t: t[1],
    )
    for idx, ap in per_class:
        print(f"  {m.names[idx]:<25s} {ap:.3f}")

    try:
        f1 = m.seg.f1_curve.mean(0)  # mean over classes, shape (1000,)
        i = int(f1.argmax())
        print(f"\nBest confidence threshold (mask F1): {m.seg.px[i]:.3f}  (F1 = {f1[i]:.3f})")
        print("Use this as CONF in infer.py")
    except Exception as e:  # attribute names differ between versions
        print(f"\nCould not compute best threshold automatically ({e}). "
              f"Read it from MaskF1_curve.png in {m.save_dir}")


if __name__ == "__main__":
    res = train()
    evaluate(Path(res.save_dir))