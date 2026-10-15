"""Drift-check silhouette and mesh simplification (isaac_backend)."""
import numpy as np
import pytest

isaac_backend = pytest.importorskip("karis_perception.isaac_backend")   # needs rclpy + karis_interfaces


def box_mesh(s=0.02):
    import trimesh
    m = trimesh.creation.box(extents=[s, s, s])
    return np.asarray(m.vertices), np.asarray(m.faces)


def test_silhouette_matches_projection():
    K = np.array([[600.0, 0, 80], [0, 600.0, 60], [0, 0, 1]])
    v, f = box_mesh(0.02)
    pose = np.eye(4)
    pose[2, 3] = 0.5
    sil = isaac_backend.render_silhouette(v, f, pose, K, (120, 160))
    # front face 0.02 m at 0.49 m -> about 24.5 px wide
    ys, xs = np.nonzero(sil)
    assert 22 <= xs.max() - xs.min() + 1 <= 27
    assert abs(xs.mean() - 80) < 1.5 and abs(ys.mean() - 60) < 1.5


def test_simplify_keeps_small_meshes():
    v, f = box_mesh()
    v2, f2 = isaac_backend.simplify_for_silhouette(v, f)
    assert len(f2) == len(f)
