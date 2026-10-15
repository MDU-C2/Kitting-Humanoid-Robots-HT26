"""Detection post-processing, classes.yaml and the rotation helpers."""
import numpy as np
import pytest

from karis_perception import vision


class FakeDetector:
    names = {0: "Ratblock Chamfer", 1: "Rund mutter"}

    def __init__(self, raw):
        self.raw = raw

    def predict(self, bgr, conf, timing=None):
        return self.raw


def square_mask(h, w, x0, y0, s):
    m = np.zeros((h, w), bool)
    m[y0:y0 + s, x0:x0 + s] = True
    return m


def test_classes_yaml_meshes_exist():
    classes = vision.load_classes()
    assert {"Rund mutter", "Ihalig bojd kvadrat", "Ratblock Chamfer"} <= set(classes)
    for cfg in classes.values():
        assert cfg["mesh"].exists(), cfg["mesh"]
        assert 0 < cfg["scale"] <= 1


def test_detect_ids_sorted_and_small_masks_dropped():
    h, w = 120, 160
    raw = [(0, 0.9, np.array([90, 10, 130, 50]), square_mask(h, w, 90, 10, 40)),
           (0, 0.8, np.array([10, 10, 50, 50]), square_mask(h, w, 10, 10, 40)),
           (1, 0.7, np.array([60, 80, 63, 83]), square_mask(h, w, 60, 80, 3))]   # 9 px: dropped
    vision.MIN_MASK_AREA_PX = 300
    dets = vision.detect(FakeDetector(raw), np.zeros((h, w, 3), np.uint8))
    assert [d["object_id"] for d in dets] == ["Ratblock Chamfer_1", "Ratblock Chamfer_2"]
    assert dets[0]["center_px"][0] < dets[1]["center_px"][0]     # left to right


def test_centre_from_depth():
    K = np.array([[600.0, 0, 80], [0, 600.0, 60], [0, 0, 1]])
    depth = np.full((120, 160), 0.5)
    c = vision.centre_from_depth(square_mask(120, 160, 70, 50, 20), depth, K)
    assert np.allclose(c, [(79.5 - 80) * 0.5 / 600, (59.5 - 60) * 0.5 / 600, 0.5])


def test_rotation_helpers_roundtrip():
    Rotation = pytest.importorskip("scipy.spatial.transform").Rotation
    R = Rotation.from_euler("xyz", [10, -20, 30], degrees=True).as_matrix()
    assert np.allclose(vision.rot_to_rpy_deg(R), [10, -20, 30], atol=1e-6)
    q = vision.rot_to_quat_xyzw(R)
    assert np.allclose(np.abs(q), np.abs(Rotation.from_matrix(R).as_quat()), atol=1e-6)
