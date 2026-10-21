from pathlib import Path
import json
from PIL import Image
import yaml
import numpy as np
from pycocotools import mask as mask_utils


def load_data_yaml(path: Path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def yolo_label_to_bbox(label_line: str, img_w: int, img_h: int):
    # YOLO format: class x_center y_center width height (normalized)
    parts = label_line.strip().split()
    if len(parts) < 5:
        raise ValueError("YOLO label line must have at least 5 values")
    cls = int(parts[0])
    xc = float(parts[1]) * img_w
    yc = float(parts[2]) * img_h
    w = float(parts[3]) * img_w
    h = float(parts[4]) * img_h
    x = xc - w / 2.0
    y = yc - h / 2.0
    # if additional coordinates present, return them as polygon points (normalized)
    extras = parts[5:]
    polygon = None
    if extras:
        vals = [float(v) for v in extras]
        if len(vals) % 2 == 0:
            # convert normalized coords to absolute ints
            poly = []
            for i in range(0, len(vals), 2):
                px = vals[i] * img_w
                py = vals[i + 1] * img_h
                poly.extend([px, py])
            polygon = poly

    return cls, [x, y, w, h], polygon


def convert(yolo_root: Path, data_yaml: Path, out_dir: Path):
    """Convert a YOLO-style dataset (with a data.yaml) to COCO instance format.

    - `data_yaml` is the Roboflow/YOLO `data.yaml` which should contain paths for train/val/test and a `names` list.
    - Images are expected at the paths listed in the yaml (relative or absolute).
    - Label files are expected next to images with the same stem and `.txt` extension.

    This converter only converts bounding boxes (no polygon masks). It creates COCO-style `instances_*.json` files.
    """
    cfg = load_data_yaml(data_yaml)

    # categories
    names = cfg.get("names") or cfg.get("class_names") or []
    categories = []
    for i, n in enumerate(names):
        categories.append({"id": i + 1, "name": str(n)})

    splits = {}
    # Common keys: train, val, test
    for key in ("train", "val", "test"):
        if key in cfg:
            splits[key] = Path(cfg[key])

    if not splits:
        raise ValueError("No train/val/test keys found in data.yaml")

    out_dir.mkdir(parents=True, exist_ok=True)

    for split, split_path in splits.items():
        # If split_path points to a txt with image list, handle it
        images = []
        if split_path.is_file() and split_path.suffix == ".txt":
            with open(split_path, "r", encoding="utf-8") as f:
                for line in f:
                    p = Path(line.strip())
                    if not p.is_file():
                        # try relative to data_yaml parent
                        p = data_yaml.parent / p
                    if p.is_file():
                        images.append(p)
        elif split_path.is_dir():
            # assume images are inside this dir
            images = list(sorted(split_path.glob("**/*.*")))
        else:
            # try relative path
            p2 = data_yaml.parent / split_path
            if p2.is_dir():
                images = list(sorted(p2.glob("**/*.*")))

        images = [p for p in images if p.suffix.lower() in [".jpg", ".jpeg", ".png"]]

        coco = {"images": [], "annotations": [], "categories": categories}
        ann_id = 1
        for img_id, img_p in enumerate(images, start=1):
            im = Image.open(img_p)
            w, h = im.size
            coco["images"].append({"id": img_id, "file_name": img_p.name, "width": w, "height": h})

            label_p = img_p.with_suffix(".txt")
            if not label_p.exists():
                # try labels/ subfolder
                alt = img_p.parent / "labels" / label_p.name
                if alt.exists():
                    label_p = alt

            segmentation_added = False
            # Prefer mask PNG if available (common Roboflow export)
            mask_png = img_p.with_name(img_p.stem + "_mask.png")
            if not mask_png.exists():
                mask_png = img_p.parent / "masks" / (img_p.stem + ".png")

            if mask_png.exists():
                m = Image.open(mask_png).convert("L")
                arr = np.array(m)
                # binaryize
                binary = (arr > 0).astype(np.uint8)
                rle = mask_utils.encode(np.asfortranarray(binary))
                area = int(mask_utils.area(rle))
                coco_ann = {
                    "id": ann_id,
                    "image_id": img_id,
                    "category_id": 1,
                    "bbox": [0, 0, w, h],
                    "segmentation": rle,
                    "area": area,
                    "iscrowd": 0,
                }
                coco["annotations"].append(coco_ann)
                ann_id += 1
                segmentation_added = True

            if label_p.exists():
                with open(label_p, "r", encoding="utf-8") as lf:
                    for line in lf:
                        if not line.strip():
                            continue
                        parsed = yolo_label_to_bbox(line, w, h)
                        if parsed is None:
                            continue
                        cls, bbox, polygon = parsed
                        coco_ann = {
                            "id": ann_id,
                            "image_id": img_id,
                            "category_id": int(cls) + 1,
                            "bbox": [bbox[0], bbox[1], bbox[2], bbox[3]],
                            "area": float(bbox[2] * bbox[3]),
                            "iscrowd": 0,
                        }
                        if polygon:
                            coco_ann["segmentation"] = [polygon]
                        coco["annotations"].append(coco_ann)
                        ann_id += 1
                        segmentation_added = segmentation_added or bool(polygon)

            if not segmentation_added:
                # warn: no segmentation found for this image
                pass

        out_path = out_dir / f"instances_{split}.json"
        with open(out_path, "w", encoding="utf-8") as f:
            json.dump(coco, f)

        print(f"Wrote COCO annotations to: {out_path} (images: {len(images)}, anns: {len(coco['annotations'])})")


if __name__ == "__main__":
    import argparse

    p = argparse.ArgumentParser()
    p.add_argument("--data", required=True, help="Path to YOLO data.yaml")
    p.add_argument("--out", default="converted_coco", help="Output folder for COCO annotations")
    args = p.parse_args()

    data_yaml = Path(args.data)
    out_dir = Path(args.out)
    convert(data_yaml.parent, data_yaml, out_dir)
