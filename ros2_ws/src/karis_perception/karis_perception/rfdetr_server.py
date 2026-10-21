"""RF-DETR detection server for detectors.RFDETRDetector, over a local socket.

Runs in its own Python environment (RF-DETR needs newer PyTorch than ROS 2 Humble):
    /opt/rfdetr-venv/bin/python rfdetr_server.py --model <models>/RF_DETR_m.pth [--size seg-medium]
"""
import argparse
import json
import socket
import struct
import time
from pathlib import Path

import numpy as np

SIZE_CLASSES = {          # --size -> rfdetr class name
    "seg-nano": "RFDETRSegNano", "seg-small": "RFDETRSegSmall", "seg-medium": "RFDETRSegMedium",
    "seg-large": "RFDETRSegLarge", "seg-xlarge": "RFDETRSegXLarge", "seg-2xlarge": "RFDETRSeg2XLarge",
    "seg-preview": "RFDETRSegPreview",
    "nano": "RFDETRNano", "small": "RFDETRSmall", "medium": "RFDETRMedium",
    "base": "RFDETRBase", "large": "RFDETRLarge",
}


def size_hint(text):
    """'rf_detr_seg_medium.pth' -> 'seg-medium'; 'rf-detr-medium.pth' -> 'medium'."""
    t = text.lower().replace("_", "-")
    seg = "seg" in t
    for size in ("2xlarge", "xlarge", "large", "medium", "small", "nano", "base", "preview"):
        if size in t:
            key = f"seg-{size}" if seg else size
            if key in SIZE_CLASSES:
                return key
    return None


def recv_exact(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(min(n - len(buf), 1 << 20))
        if not chunk:
            raise ConnectionError("client closed the connection")
        buf += chunk
    return bytes(buf)


def recv(sock):
    lh, lp = struct.unpack("!II", recv_exact(sock, 8))
    head = json.loads(recv_exact(sock, lh))
    return head, recv_exact(sock, lp) if lp else b""


def send(sock, obj, payload=b""):
    head = json.dumps(obj).encode()
    sock.sendall(struct.pack("!II", len(head), len(payload)) + head + payload)


def load_model(path, size=None):
    import rfdetr
    path = Path(path)
    if size:
        name = SIZE_CLASSES.get(size.lower(), size)
        cls = getattr(rfdetr, name, None)
        if cls is None:
            raise ValueError(f"This rfdetr version has no {name}. Sizes: {list(SIZE_CLASSES)}")
        return cls(pretrain_weights=str(path))
    if hasattr(rfdetr, "from_checkpoint"):
        try:
            try:
                return rfdetr.from_checkpoint(str(path), trust_checkpoint=True)
            except TypeError:
                return rfdetr.from_checkpoint(str(path))
        except Exception as e:
            print(f"  from_checkpoint could not load it ({e}); trying the sizes one by one...", flush=True)
    # older rfdetr: guess the size from the checkpoint, then try all sizes
    hint = ""
    try:
        import torch
        ck = torch.load(str(path), map_location="cpu", weights_only=False)
        args = ck.get("args") if isinstance(ck, dict) else None
        get = (args.get if isinstance(args, dict) else lambda k, d="": getattr(args, k, d)) \
            if args is not None else (lambda k, d="": d)
        hint = f"{get('model_name', '')} {get('pretrain_weights', '')}"
        if get("segmentation_head", False) and "seg" not in hint.lower():
            hint = "seg " + hint
    except Exception:
        pass
    hints = [h for h in (size_hint(path.name), size_hint(hint)) if h]
    others = [n for n in dir(rfdetr) if n.startswith("RFDETR") and isinstance(getattr(rfdetr, n), type)]
    order = [SIZE_CLASSES[h] for h in hints] + list(SIZE_CLASSES.values()) + others
    last_err = None
    for name in dict.fromkeys(order):
        cls = getattr(rfdetr, name, None)
        if cls is None:
            continue
        try:
            print(f"  trying {name}...", flush=True)
            return cls(pretrain_weights=str(path))
        except Exception as e:
            last_err = e
    raise RuntimeError(f"Could not load {path} with any RF-DETR size. Pass --size. Last error: {last_err}")


def class_name_list(model):
    names = getattr(model, "class_names", None)
    if isinstance(names, dict):                          # {1: 'a', 2: 'b'} in some versions
        out = ["_unused"] * (max(names) + 1)
        for k, v in names.items():
            out[int(k)] = v
        return out
    return list(names) if names else []


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--port", type=int, default=5601)
    ap.add_argument("--size", default=None,
                    help="if auto-detect fails: seg-medium, seg-small, ..., medium, base, ... "
                         "or a class name like RFDETRSegMedium")
    a = ap.parse_args()

    from PIL import Image
    try:
        import torch
        cuda = torch.cuda.is_available()
    except ImportError:
        torch, cuda = None, False
    import rfdetr

    path = Path(a.model).resolve()
    print(f"Loading RF-DETR {path.name} (GPU: {cuda})...", flush=True)
    model = load_model(path, a.size)
    names = class_name_list(model)
    seg = "seg" in type(model).__name__.lower() or bool(
        getattr(getattr(model, "model_config", None), "segmentation_head", False))
    info = {"model": str(path), "model_class": type(model).__name__, "names": names,
            "segmentation": seg, "version": getattr(rfdetr, "__version__", "?")}
    print(f"Loaded {info['model_class']}: {len(names)} classes {names}, "
          f"{'masks' if seg else 'boxes only'}", flush=True)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", a.port))
    srv.listen(1)
    print(f"Ready on 127.0.0.1:{a.port}", flush=True)
    while True:
        conn, _ = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            while True:
                head, payload = recv(conn)
                cmd = head.get("cmd")
                if cmd == "info":
                    send(conn, info)
                elif cmd == "predict":
                    h, w = head["h"], head["w"]
                    bgr = np.frombuffer(payload, np.uint8).reshape(h, w, 3)
                    img = Image.fromarray(np.ascontiguousarray(bgr[:, :, ::-1]))   # RGB
                    if cuda:
                        torch.cuda.synchronize()
                    t0 = time.perf_counter()
                    d = model.predict(img, threshold=float(head.get("threshold", 0.5)))
                    if cuda:
                        torch.cuda.synchronize()
                    ms = (time.perf_counter() - t0) * 1000
                    n = 0 if d.xyxy is None else len(d.xyxy)
                    mask = getattr(d, "mask", None)
                    has_mask = mask is not None and n > 0
                    out = {"n": n, "infer_ms": ms, "has_mask": has_mask,
                           "boxes": [] if n == 0 else np.asarray(d.xyxy, float).tolist(),
                           "scores": [] if n == 0 else np.asarray(d.confidence, float).tolist(),
                           "class_ids": [] if n == 0 else np.asarray(d.class_id, int).tolist()}
                    pay = np.packbits(np.asarray(mask, bool).reshape(-1)).tobytes() if has_mask else b""
                    send(conn, out, pay)
                elif cmd == "bye":
                    break
        except (ConnectionError, OSError):
            pass
        finally:
            conn.close()


if __name__ == "__main__":
    main()
