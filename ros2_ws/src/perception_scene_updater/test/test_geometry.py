import math
from types import SimpleNamespace

from perception_scene_updater.geometry import compose, pair_boxes_with_labels, quat_rotate


def test_compose_translation_and_rotation():
    # frame b is frame a rotated 90 deg about z and shifted by (1, 0, 0)
    s = math.sqrt(0.5)
    t_ab = ((1.0, 0.0, 0.0), (0.0, 0.0, s, s))
    pos, rot = compose(t_ab, ((1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))
    assert all(abs(a - b) < 1e-9 for a, b in zip(pos, (1.0, 1.0, 0.0)))
    assert all(abs(a - b) < 1e-9 for a, b in zip(rot, (0.0, 0.0, s, s)))


def test_camera_to_pelvis_example():
    # pelvis <- camera from the real robot (tf2_echo pelvis head_camera_color_optical_frame)
    t = ((0.054, 0.033, 0.474), (0.659, -0.659, 0.256, -0.256))
    # a part 0.65 m straight in front of the camera lies forward and below the pelvis
    pos, _ = compose(t, ((0.0, 0.0, 0.65), (0.0, 0.0, 0.0, 1.0)))
    assert 0.4 < pos[0] < 0.6 and abs(pos[1] - 0.033) < 0.01 and pos[2] < 0.05


def test_rotate_identity():
    assert quat_rotate((0.0, 0.0, 0.0, 1.0), (1.0, 2.0, 3.0)) == (1.0, 2.0, 3.0)


def test_pair_boxes_with_labels():
    m = lambda ns, i, t, text="": SimpleNamespace(ns=ns, id=i, type=t, text=text)   # noqa: E731
    markers = [m("", 0, 3), m("karis_parts", 0, 1), m("karis_labels", 0, 9, "ratblock_chamfer_2"),
               m("karis_parts", 1, 2), m("karis_labels", 1, 9, "ratblock_chamfer_hole_48540193_1")]
    out = pair_boxes_with_labels(markers)
    assert list(out) == ["ratblock_chamfer_2"]          # the SPHERE (slot) is skipped
