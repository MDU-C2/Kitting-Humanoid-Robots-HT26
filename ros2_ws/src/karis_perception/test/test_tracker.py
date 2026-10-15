"""PoseTracker with a fake detector and a fake FoundationPose: register, track, re-detect, drop."""
import numpy as np

from karis_perception import tracker, vision

K = np.array([[600.0, 0, 80], [0, 600.0, 60], [0, 0, 1]])
H, W = 120, 160


class State:
    def __init__(self, pose):
        self.pose = pose

    def clone(self):
        return State(self.pose.copy())


class FakeEstimator:
    """Registers every part at 0.5 m in front of its mask centre; tracking keeps the pose."""

    def __init__(self):
        self.pose_last = None
        self.n_register = self.n_track = 0

    def register(self, K, rgb, depth, ob_mask, iteration=None):
        self.n_register += 1
        ys, xs = np.nonzero(ob_mask)
        u, v = xs.mean(), ys.mean()
        pose = np.eye(4)
        pose[:3, 3] = [(u - K[0, 2]) * 0.5 / K[0, 0], (v - K[1, 2]) * 0.5 / K[1, 1], 0.5]
        self.pose_last = State(pose)
        return pose

    def track_one(self, rgb, depth, K, iteration=None):
        self.n_track += 1
        return self.pose_last.pose


class FakeDetector:
    names = {0: "Ratblock Chamfer"}

    def __init__(self):
        self.visible = True

    def predict(self, bgr, conf, timing=None):
        if not self.visible:
            return []
        m = np.zeros((H, W), bool)
        m[40:80, 60:100] = True
        return [(0, 0.9, np.array([60, 40, 100, 80]), m)]


def make_tracker():
    est = FakeEstimator()
    ests = {"Ratblock Chamfer": {"est": est, "to_origin": np.eye(4),
                                 "bbox": np.array([[-0.01] * 3, [0.01] * 3]), "extent": 0.02}}
    det = FakeDetector()
    t = tracker.PoseTracker(det, ests, None, None, K, check=lambda *a: (1.0, None, None))
    return t, det, est


def frame():
    return np.zeros((H, W, 3), np.uint8), np.full((H, W), 0.5)


def test_register_then_track():
    vision.MIN_MASK_AREA_PX = 100
    tracker.REDETECT_EVERY = 5
    t, det, est = make_tracker()
    bgr, depth = frame()
    tm = t.step(bgr, depth, 0, 0.0)
    assert tm["mode"] == "detect+register" and list(t.tracks) == ["Ratblock Chamfer_1"]
    for i in range(1, 5):
        assert t.step(bgr, depth, i, 0.1 * i)["mode"] == "track"
    assert t.step(bgr, depth, 5, 0.5)["mode"] == "detect+track"     # every 5th frame detects
    assert est.n_register == 1 and est.n_track == 5
    assert np.allclose(t.tracks["Ratblock Chamfer_1"]["pose"][:3, 3], [0, 0, 0.5], atol=1e-3)


def test_drift_registers_again():
    t, det, est = make_tracker()
    t.check = lambda *a: (0.05, None, None)        # mesh and mask no longer overlap
    bgr, depth = frame()
    t.step(bgr, depth, 0, 0.0)
    t.step(bgr, depth, 1, 0.1, force_detect=True)
    assert est.n_register == 2 and t.n_reregistered == 1


def test_lost_part_is_dropped():
    tracker.LOST_AFTER = 3
    t, det, est = make_tracker()
    bgr, depth = frame()
    t.step(bgr, depth, 0, 0.0)
    det.visible = False
    for i in range(1, 4):
        t.step(bgr, depth, i, 0.1 * i, force_detect=True)
    assert not t.tracks and t.n_dropped == 1
