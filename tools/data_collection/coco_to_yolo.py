"""
Convert a Roboflow COCO-segmentation export (the one used for RF-DETR) into
a YOLO segmentation dataset, so YOLO and RF-DETR train on the exact same data.

Input layout (Roboflow COCO export):
    <COCO_ROOT>/train/_annotations.coco.json  + images
    <COCO_ROOT>/valid/_annotations.coco.json  + images
    <COCO_ROOT>/test/_annotations.coco.json   + images   (optional)

Output layout (YOLO):
    <OUT_ROOT>/images/{train,val,test}/*.jpg
    <OUT_ROOT>/labels/{train,val,test}/*.txt   (class x1 y1 x2 y2 ... normalized)
    <OUT_ROOT>/data.yaml

Usage:
    python coco_to_yolo.py
    python coco_to_yolo.py --coco-root "path\\to\\coco" --out-root "path\\to\\yolo"
    python coco_to_yolo.py --bbox      # write detection (box) labels instead of polygons
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
from collections import Counter
from pathlib import Path

import yaml

DEFAULT_COCO_ROOT = Path(
    r"C:\Users\kimsv\PycharmProjects\KARIS_2\VisionAspect\Data\KARIS.v8i.coco-segmentation"
)
DEFAULT_OUT_ROOT = Path(
    r"C:\Users\kimsv\PycharmProjects\KARIS_2\VisionAspect\Data\KARIS.v8i.yolo-from-coco"
)

# Roboflow split name -> YOLO split name
SPLITS = {"train": "train", "valid": "val", "test": "test"}


def build_class_map(categories: list[dict], used_ids: set[int]) -> tuple[dict[int, int], list[str]]:
    """Map COCO category ids to contiguous YOLO ids 0..N-1.

    Roboflow adds a parent category (here id 0 'KARIS') that no annotation uses.
    It is dropped, so YOLO gets only the real classes.
    """
    cats = sorted(categories, key=lambda c: c["id"])
    kept = [c for c in cats if c["id"] in used_ids or c.get("supercategory") != "none"]
    coco_to_yolo = {c["id"]: i for i, c in enumerate(kept)}
    names = [c["name"] for c in kept]
    return coco_to_yolo, names


def polygon_from_segmentation(seg, w: int, h: int) -> list[float] | None:
    """Return one flat polygon [x1,y1,x2,y2,...] in pixels, or None."""
    if isinstance(seg, dict):  # RLE (rare in Roboflow exports)
        try:
            import numpy as np
            import cv2
            from pycocotools import mask as mask_utils
        except ImportError:
            return None
        rle = mask_utils.frPyObjects(seg, h, w) if isinstance(seg["counts"], list) else seg
        m = mask_utils.decode(rle).astype("uint8")
        contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        c = max(contours, key=cv2.contourArea).reshape(-1, 2)
        return c.flatten().astype(float).tolist()

    polys = [p for p in seg if len(p) >= 6]
    if not polys:
        return None
    if len(polys) == 1:
        return list(polys[0])
    # Several parts for one instance: YOLO wants one polygon, keep the largest.
    def area(p):
        xs, ys = p[0::2], p[1::2]
        return abs(sum(xs[i] * ys[i - 1] - xs[i - 1] * ys[i] for i in range(len(xs)))) / 2
    return list(max(polys, key=area))


def bbox_to_yolo(b, w, h):
    x, y, bw, bh = b
    cx, cy = (x + bw / 2) / w, (y + bh / 2) / h
    return [min(max(v, 0.0), 1.0) for v in (cx, cy, bw / w, bh / h)]


def place_image(src: Path, dst: Path, mode: str):
    if dst.exists():
        return
    if mode == "hardlink":
        try:
            os.link(src, dst)
            return
        except OSError:
            pass  # different drive / OneDrive: fall back to copy
    shutil.copy2(src, dst)


def convert_split(coco_split_dir: Path, out_root: Path, yolo_split: str,
                  coco_to_yolo: dict[int, int], use_bbox: bool, mode: str) -> Counter:
    ann_path = coco_split_dir / "_annotations.coco.json"
    with ann_path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    img_out = out_root / "images" / yolo_split
    lbl_out = out_root / "labels" / yolo_split
    img_out.mkdir(parents=True, exist_ok=True)
    lbl_out.mkdir(parents=True, exist_ok=True)

    anns_by_img: dict[int, list[dict]] = {}
    for a in data["annotations"]:
        anns_by_img.setdefault(a["image_id"], []).append(a)

    stats = Counter()
    for img in data["images"]:
        src = coco_split_dir / img["file_name"]
        if not src.exists():
            stats["missing_images"] += 1
            continue
        w, h = img["width"], img["height"]

        lines = []
        for a in anns_by_img.get(img["id"], []):
            if a.get("iscrowd", 0):
                stats["skipped_crowd"] += 1
                continue
            cls = coco_to_yolo.get(a["category_id"])
            if cls is None:
                stats["skipped_unknown_class"] += 1
                continue

            if use_bbox:
                coords = bbox_to_yolo(a["bbox"], w, h)
            else:
                poly = polygon_from_segmentation(a.get("segmentation", []), w, h)
                if poly is None:
                    stats["skipped_no_polygon"] += 1
                    continue
                coords = [min(max(v / (w if i % 2 == 0 else h), 0.0), 1.0)
                          for i, v in enumerate(poly)]
            lines.append(f"{cls} " + " ".join(f"{v:.6f}" for v in coords))
            stats[f"class_{cls}"] += 1

        name = Path(img["file_name"]).name
        place_image(src, img_out / name, mode)
        (lbl_out / (Path(name).stem + ".txt")).write_text("\n".join(lines), encoding="utf-8")
        stats["images"] += 1
        if not lines:
            stats["background_images"] += 1
    return stats


def main():
    ap = argparse.ArgumentParser(description="Convert Roboflow COCO segmentation to YOLO format.")
    ap.add_argument("--coco-root", type=Path, default=DEFAULT_COCO_ROOT)
    ap.add_argument("--out-root", type=Path, default=DEFAULT_OUT_ROOT)
    ap.add_argument("--bbox", action="store_true", help="write box labels (detection) instead of polygons")
    ap.add_argument("--copy", action="store_true", help="always copy images (default tries hardlinks first)")
    args = ap.parse_args()

    coco_root, out_root = args.coco_root.resolve(), args.out_root.resolve()
    present = {s: y for s, y in SPLITS.items() if (coco_root / s / "_annotations.coco.json").exists()}
    if "train" not in present:
        raise FileNotFoundError(f"No train/_annotations.coco.json in {coco_root}")

    # Class map from train split so all splits share the same ids.
    with (coco_root / "train" / "_annotations.coco.json").open("r", encoding="utf-8") as f:
        train = json.load(f)
    used = {a["category_id"] for a in train["annotations"]}
    coco_to_yolo, names = build_class_map(train["categories"], used)

    print("Class mapping (COCO id -> YOLO id: name):")
    for cid, yid in coco_to_yolo.items():
        print(f"  {cid} -> {yid}: {names[yid]}")

    mode = "copy" if args.copy else "hardlink"
    for coco_split, yolo_split in present.items():
        stats = convert_split(coco_root / coco_split, out_root, yolo_split,
                              coco_to_yolo, args.bbox, mode)
        print(f"\n[{coco_split} -> {yolo_split}] " +
              ", ".join(f"{k}={v}" for k, v in sorted(stats.items())))
        if stats["missing_images"]:
            print(f"  WARNING: {stats['missing_images']} images listed in the JSON were not found.")

    cfg = {
        "path": str(out_root),
        "train": "images/train",
        "val": "images/val" if "valid" in present else "images/train",
        "nc": len(names),
        "names": names,
    }
    if "test" in present:
        cfg["test"] = "images/test"
    with (out_root / "data.yaml").open("w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, sort_keys=False, allow_unicode=True)
    print(f"\nWrote {out_root / 'data.yaml'}")
    print(f'Set DATASET_ROOT = Path(r"{out_root}") in new_train.py')


if __name__ == "__main__":
    main()
