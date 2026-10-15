"""TensorRT helpers: input size from an engine / a YOLO checkpoint."""
import json
from types import SimpleNamespace

from karis_perception import trt_support


def test_engine_imgsz(tmp_path):
    meta = json.dumps({"imgsz": [480, 640]}).encode()
    p = tmp_path / "m.engine"
    p.write_bytes(len(meta).to_bytes(4, "little") + meta + b"\0" * 8)
    assert trt_support.engine_imgsz(p) == [480, 640]


def test_yolo_train_imgsz():
    assert trt_support.yolo_train_imgsz(SimpleNamespace(overrides={"imgsz": 640})) == 640
    assert trt_support.yolo_train_imgsz(SimpleNamespace(overrides={}, ckpt={"train_args": {"imgsz": 1280}})) == 1280
    assert trt_support.yolo_train_imgsz(SimpleNamespace()) is None


def test_enable_adds_extensions():
    d = SimpleNamespace(MODEL_EXTS={".pt", ".pth"}, model_kind=lambda p: "x")
    trt_support.enable(d)
    assert {".engine", ".trt"} <= d.MODEL_EXTS
    assert d.model_kind("a.engine") == "YOLO-TRT" and d.model_kind("a.trt") == "RF-DETR-TRT"
