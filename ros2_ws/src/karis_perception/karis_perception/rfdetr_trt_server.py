"""
RF-DETR TensorRT helper: runs an RF-DETR engine (.trt, made by `export_trt`) and answers
detection requests over a local socket, with the same protocol as rfdetr_server.py
(info / predict / bye). Started automatically by trt_support.RFDETRTRTDetector
in the RF-DETR Python (KARIS_RFDETR_PYTHON); by hand:

    /opt/rfdetr-venv/bin/python rfdetr_trt_server.py --engine models/RF-DETR_m.trt

Needs in that Python: pip install -U "rfdetr[onnx]>=1.9" "tensorrt-cu12>=10.8,<11" polygraphy onnxconverter-common
The engine runs only on the GPU and TensorRT version it was built with.

Standalone on purpose: no ROS, no karis_perception imports (it runs in another Python).
"""
import argparse
import json
import socket
import struct
import time
from pathlib import Path

import numpy as np

IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)


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


def sidecar(engine: Path) -> Path:
    return engine.with_suffix(".karis.json")


class TrtRFDETR:
    def __init__(self, engine: Path):
        import torch
        import torch.nn.functional as F
        try:
            from rfdetr.export._tensorrt.inference import TRTInference
            from rfdetr.export._runtime.decode import decode_detections
        except ImportError as e:
            raise ImportError(f"{e}\nThis rfdetr has no TensorRT runtime: in the RF-DETR Python run "
                              "pip install -U \"rfdetr[onnx]>=1.9\" \"tensorrt-cu12>=10.8,<11\" "
                              "polygraphy onnxconverter-common") from e
        self.torch, self.F, self.decode = torch, F, decode_detections
        meta = json.loads(sidecar(engine).read_text()) if sidecar(engine).exists() else {}
        self.names = meta.get("names") or []
        self.rt = TRTInference(str(engine), device="cuda:0", sync_mode=True)
        self.inp = self.rt.input_names[0]
        b = self.rt.bindings[self.inp]
        _, self.c, self.h, self.w = (int(v) for v in b.shape)
        self.in_dtype = torch.from_numpy(np.empty(0, dtype=b.dtype)).dtype
        outs = self.rt.output_names
        self.out_boxes = next(n for n in outs if "dets" in n)
        self.out_logits = next(n for n in outs if "labels" in n)
        self.out_masks = next((n for n in outs if "mask" in n), None)
        self.dev = self.rt.engine_device
        self.mean = torch.tensor(IMAGENET_MEAN, device=self.dev).view(1, 3, 1, 1)
        self.std = torch.tensor(IMAGENET_STD, device=self.dev).view(1, 3, 1, 1)
        if not self.names:
            n_cls = int(self.rt.bindings[self.out_logits].shape[-1])
            self.names = [str(i) for i in range(n_cls)]
        self.info = {"model": str(engine), "model_class": meta.get("model_class", "TensorRT"),
                     "names": self.names, "segmentation": self.out_masks is not None,
                     "version": meta.get("rfdetr", "?"), "resolution": [self.h, self.w]}

    def predict(self, bgr, threshold):
        """Same preprocessing as RFDETR.predict (bilinear, half-pixel, no antialias, ImageNet
        normalisation), done on the GPU; same decoding (rfdetr's reference decoder)."""
        torch, F = self.torch, self.F
        h, w = bgr.shape[:2]
        with torch.no_grad():
            t = torch.from_numpy(np.ascontiguousarray(bgr[:, :, ::-1])).to(self.dev)
            t = t.permute(2, 0, 1).unsqueeze(0).float().div_(255.0)
            t = F.interpolate(t, size=(self.h, self.w), mode="bilinear", align_corners=False, antialias=False)
            t = ((t - self.mean) / self.std).to(self.in_dtype).contiguous()
            out = self.rt({self.inp: t})
            boxes = out[self.out_boxes][0].float().cpu().numpy()
            logits = out[self.out_logits][0].float().cpu().numpy()
            # background_class_id=None: keep every class slot, like RFDETR.predict's PostProcess
            try:
                d = self.decode(boxes, logits, (w, h), threshold=threshold, background_class_id=None)
            except TypeError:                                  # older rfdetr: no background_class_id
                d = self.decode(boxes, logits, (w, h), threshold=threshold)
            masks = None
            if self.out_masks is not None and len(d.query_index):
                sel = torch.as_tensor(np.asarray(d.query_index), device=self.dev, dtype=torch.long)
                m = out[self.out_masks][0].index_select(0, sel).float().unsqueeze(1)
                masks = (F.interpolate(m, size=(h, w), mode="bilinear", align_corners=False)[:, 0] > 0.0)
                masks = masks.cpu().numpy()
        return d, masks


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--engine", required=True)
    ap.add_argument("--port", type=int, default=5602)
    a = ap.parse_args()

    engine = Path(a.engine).resolve()
    print(f"Loading RF-DETR TensorRT engine {engine.name}...", flush=True)
    model = TrtRFDETR(engine)
    for _ in range(3):                                        # warm-up
        model.predict(np.zeros((480, 640, 3), np.uint8), 0.5)
    print(f"Loaded: input {model.w}x{model.h}, {len(model.names)} classes {model.names}, "
          f"{'masks' if model.info['segmentation'] else 'boxes only'}", flush=True)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", a.port))
    srv.listen(1)
    print(f"Ready on 127.0.0.1:{a.port}", flush=True)
    torch = model.torch
    while True:
        conn, _ = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        try:
            while True:
                head, payload = recv(conn)
                cmd = head.get("cmd")
                if cmd == "info":
                    send(conn, model.info)
                elif cmd == "predict":
                    h, w = head["h"], head["w"]
                    bgr = np.frombuffer(payload, np.uint8).reshape(h, w, 3)
                    torch.cuda.synchronize()
                    t0 = time.perf_counter()
                    d, masks = model.predict(bgr, float(head.get("threshold", 0.5)))
                    ms = (time.perf_counter() - t0) * 1000
                    n = len(d.xyxy)
                    has_mask = masks is not None and n > 0
                    out = {"n": n, "infer_ms": ms, "has_mask": has_mask,
                           "boxes": np.asarray(d.xyxy, float).tolist(),
                           "scores": np.asarray(d.confidence, float).tolist(),
                           "class_ids": np.asarray(d.class_id, int).tolist()}
                    pay = np.packbits(masks.reshape(-1)).tobytes() if has_mask else b""
                    send(conn, out, pay)
                elif cmd == "bye":
                    break
        except (ConnectionError, OSError):
            pass
        finally:
            conn.close()


if __name__ == "__main__":
    main()
