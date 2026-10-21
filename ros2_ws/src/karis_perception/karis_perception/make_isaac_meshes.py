"""Write textured, centred OBJ meshes for Isaac ROS FoundationPose from config/classes.yaml.

Each mesh is centred on its bounding box; the centre goes into meshes.yaml so fp_bridge can
convert Isaac's poses back to the CAD frame. Output in <KARIS_DIR>/meshes_isaac/.

    ros2 run karis_perception make_isaac_meshes [--color R G B]
"""
import argparse
import os
from pathlib import Path

import cv2
import numpy as np
import yaml

from karis_perception.ros_convert import slug


def write_obj(mesh, out_dir, name, color):
    out_dir.mkdir(parents=True, exist_ok=True)
    tex = np.full((8, 8, 3), color[::-1], np.uint8)          # BGR
    cv2.imwrite(str(out_dir / "texture.png"), tex)
    (out_dir / f"{name}.mtl").write_text(
        "newmtl material_0\nKa 1.0 1.0 1.0\nKd 1.0 1.0 1.0\nKs 0.0 0.0 0.0\nd 1.0\nillum 2\n"
        "map_Kd texture.png\n")
    v = np.asarray(mesh.vertices, dtype=np.float64)
    n = np.asarray(mesh.vertex_normals, dtype=np.float64)
    f = np.asarray(mesh.faces, dtype=np.int64) + 1
    lines = [f"mtllib {name}.mtl", "o part"]
    lines += [f"v {a:.7f} {b:.7f} {c:.7f}" for a, b, c in v]
    lines += [f"vn {a:.6f} {b:.6f} {c:.6f}" for a, b, c in n]
    lines.append("vt 0.5 0.5")
    lines.append("usemtl material_0")
    lines += [f"f {a}/1/{a} {b}/1/{b} {c}/1/{c}" for a, b, c in f]
    (out_dir / f"{name}.obj").write_text("\n".join(lines) + "\n")
    return out_dir / f"{name}.obj"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--karis-dir", default=os.environ.get("KARIS_DIR") or os.path.join(
        os.path.dirname(os.environ.get("KARIS_WS", "/home/kimsv/Kitting-Humanoid-Robots-HT26/ros2_ws")), "local_data"))
    ap.add_argument("--classes-file", default="", help="'' = this package's config/classes.yaml")
    ap.add_argument("--color", type=int, nargs=3, default=[160, 160, 160], metavar=("R", "G", "B"))
    a = ap.parse_args()
    kdir = Path(a.karis_dir).resolve()
    import trimesh
    from karis_perception.vision import load_classes

    out_root = kdir / "meshes_isaac"
    classes = {}
    for cls_name, cfg in load_classes(a.classes_file or None).items():
        mesh = trimesh.load(str(cfg["mesh"]), force="mesh")
        mesh.apply_scale(cfg.get("scale", 1.0))
        s = slug(cls_name)
        lo, hi = mesh.vertices.min(0), mesh.vertices.max(0)
        center = (lo + hi) / 2
        centred = mesh.copy()
        centred.vertices = centred.vertices - center
        obj = write_obj(centred, out_root / s, s, a.color)
        ext = mesh.extents
        classes[cls_name] = {"slug": s, "obj": str(obj), "source": str(cfg["mesh"]),
                             "center_m": [float(x) for x in center],
                             "extents_m": [round(float(x), 5) for x in ext]}
        print(f"  {cls_name:<26s} -> {obj}  ({np.round(ext * 1000, 1)} mm)")
        if ext.max() > 1.0:
            print(f"    WARNING: {ext.max():.2f} m across: wrong scale in classes.yaml?")
    with open(out_root / "meshes.yaml", "w") as f:
        yaml.safe_dump({"classes": classes}, f, sort_keys=False, allow_unicode=True)
    print(f"Wrote {out_root / 'meshes.yaml'} ({len(classes)} classes)")


if __name__ == "__main__":
    main()
