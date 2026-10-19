"""YOLO input size from the checkpoint."""
from types import SimpleNamespace

from karis_perception import detectors


def test_yolo_train_imgsz():
    assert detectors.yolo_train_imgsz(SimpleNamespace(overrides={"imgsz": 640})) == 640
    assert detectors.yolo_train_imgsz(
        SimpleNamespace(overrides={}, ckpt={"train_args": {"imgsz": 1280}})) == 1280
    assert detectors.yolo_train_imgsz(SimpleNamespace(overrides={"imgsz": [480, 640]})) == 640
    assert detectors.yolo_train_imgsz(SimpleNamespace()) is None


def test_only_pytorch_models():
    assert detectors.MODEL_EXTS == {".pt", ".pth", ".ckpt"}
