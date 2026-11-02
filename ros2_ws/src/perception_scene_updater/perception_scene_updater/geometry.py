"""
Small pose maths without ROS or numpy, so it can be unit tested anywhere.

Poses are (position (x, y, z), quaternion (x, y, z, w)). A transform T_a_b maps coordinates
in frame b to frame a: p_a = T_a_b * p_b.
"""
import math


def quat_mul(q1, q2):
    """Hamilton product q1 * q2 (both x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def quat_normalize(q):
    n = math.sqrt(sum(v * v for v in q))
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return tuple(v / n for v in q)


def quat_rotate(q, v):
    """Rotate vector v by unit quaternion q."""
    qv = (v[0], v[1], v[2], 0.0)
    qc = (-q[0], -q[1], -q[2], q[3])
    r = quat_mul(quat_mul(q, qv), qc)
    return (r[0], r[1], r[2])


def compose(t_ab, p_b):
    """Pose of p_b expressed in frame a, given transform T_a_b.

    t_ab and p_b are ((x, y, z), (qx, qy, qz, qw)). Returns the same form."""
    (ta, qa), (pb, qb) = t_ab, p_b
    qa = quat_normalize(qa)
    rp = quat_rotate(qa, pb)
    pos = (ta[0] + rp[0], ta[1] + rp[1], ta[2] + rp[2])
    rot = quat_normalize(quat_mul(qa, quat_normalize(qb)))
    return pos, rot


def pair_boxes_with_labels(markers, box_ns="karis_parts", label_ns="karis_labels"):
    """The perception's /karis/markers has, per part with a full pose, a CUBE in namespace
    `box_ns` and a TEXT_VIEW_FACING in `label_ns` with the same id; the text is the part's
    object_id (e.g. "ratblock_chamfer_2"). Position-only parts/slots are SPHEREs: skipped.

    `markers` is a list of objects with .ns, .id, .type, .text (visualization_msgs/Marker).
    Returns {object_id: cube_marker}. CUBE = 1, TEXT_VIEW_FACING = 9 (Marker constants)."""
    cube, text = 1, 9
    labels = {m.id: m.text for m in markers if m.ns == label_ns and m.type == text}
    out = {}
    for m in markers:
        if m.ns == box_ns and m.type == cube:
            out[labels.get(m.id, f"part_{m.id}")] = m
    return out
