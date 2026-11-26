"""Tray pose from its plane, outline and slots (no FoundationPose), with the slot layout of the
tray CAD (<karis_dir>/tray_layout.yaml, made by experiments/tray_slots/make_layout_from_cad.py).

The tray is a flat plate with holes. Each detection frame:
  1. plane:    RANSAC plane through the tray mask's depth points -> tray z axis (towards the camera)
  2. outline:  direction of the mask's edges on that plane (edges on the image border are ignored,
               so a tray cut off by the image still works) -> the tray's axes, up to 90 degrees
  3. slots:    the HOLE detections (centre: camera ray onto the plane) and the parts lying in the
               tray (centre: their depth points dropped straight onto the plane, so a tall part
               gives no parallax) are matched with the layout for each of the 4 candidate
               rotations, together with the tray outline (the visible outline must lie inside the
               tray rectangle); the best match gives the rotation and the position. With
               HOLES_FIRST or more matched holes the final fit uses the holes only (flat, exact).
               A jump of more than JUMP_M / JUMP_DEG is only taken when the next detection frame
               confirms it. A part matches
               its hole, or for PAIR_TYPES (U-parts across two neighbouring slits) the midpoint of
               the two: a "place", computed from the layout.

Each slot then gets the tray's orientation (TrayEstimator.slots): a fixed id, a state (empty: its
hole is seen, position = the detected hole centre; filled: a part lies on it, also both slits under
a U-part; not_seen) and otherwise the layout position on the fitted tray.

Tray frame: origin at the centre of the top surface, z up (out of the plate), x along the long side
towards the row of 8 slits, y = z cross x. Without a layout the pose comes from the outline alone
(x along the long side, pointing up in the image) and can flip by 180 degrees when the tray turns.
/karis/save_tray_layout saves a layout measured by the camera instead (it overwrites the CAD one).
"""
from __future__ import annotations

import time
from pathlib import Path

import cv2
import numpy as np

PLANE_TOL_M = 0.006       # RANSAC inlier distance to the plane
PLANE_POINTS = 1000       # tray depth points sampled for the plane fit (plenty for ~0.05 deg)
SLOT_MATCH_M = 0.015      # an observed slot within this distance of a layout slot matches it
MIN_SLOT_MATCHES = 2      # fewer matched slots: keep the previous pose (or the outline pose)
MAX_REFINE_DEG = 10.0     # the slot fit may turn the outline's rotation by at most this much
BORDER_PX = 4             # outline points this close to the image border are ignored
SMOOTH = 0.5              # 0 = no smoothing, 0.5 = average with the previous pose when close
SMOOTH_MAX_M, SMOOTH_MAX_DEG = 0.02, 8.0
THICKNESS_M = 0.003       # box height for markers
LOST_AFTER = 3            # forget the pose after this many detection frames without the tray
PAIR_TYPES = {"Ihalig bojd kvadrat"}   # a part of these types sits across two neighbouring holes
PAIR_DIST_M = (0.022, 0.038)          # two holes this far apart form a place for one such part
HOLES_FIRST = 3           # with at least this many matched holes, refine the fit on the holes only
                          # (the position is always CHOSEN with all holes, parts and the outline)
OUTLINE_MIN = 0.9         # a candidate pose must have this fraction of the visible tray outline
OUTLINE_MARGIN_M = 0.012  # inside its rectangle (layout size + this margin)
JUMP_M, JUMP_DEG = 0.02, 5.0   # a bigger jump of the tray is only taken when the next detection
                               # frame confirms it (a covered tray can give one wrong fit)
PART_DEPTH_MIN_PX = 30    # a part's centre from its depth points (straight down onto the tray)
                          # when it has this many; else along the camera ray (parallax)


def slot_type(cls_name: str) -> str:
    """'Rund mutter HOLE 11432713' -> 'Rund mutter'; part classes stay as they are."""
    i = cls_name.find(" HOLE")
    return cls_name[:i] if i >= 0 else cls_name


def _rot2(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s], [s, c]])


class TrayEstimator:
    def __init__(self, cls_name: str, layout_path: Path | None, log=print):
        self.cls_name, self.layout_path, self.log = cls_name, layout_path, log
        self.layout, self.cls_id = None, None
        self.pose, self.size, self.conf, self.method = None, None, 0.0, ""
        self.n_matched, self.t_seen, self.missed = 0, 0.0, 0
        self.pending = None               # a jumped pose waiting for confirmation
        self.matched_px = []              # (u, v) of layout slots, for the overlay
        self.slots = []                   # slot poses of the last update (see _slot_poses)
        self.last = None                  # last observation, for save_layout
        self.warned = 0.0
        self.load_layout()

    # ------------------------------------------------------------ layout file
    def load_layout(self):
        self.layout = None
        p = self.layout_path
        if p is None or not p.is_file():
            self.log(f"tray '{self.cls_name}': no slot layout yet ({p}); pose from the outline only "
                     "(can flip 180 deg). Save one with the whole tray in view: ros2 service call "
                     "/karis/save_tray_layout std_srvs/srv/Trigger")
            return
        import yaml
        d = yaml.safe_load(p.read_text()) or {}
        slots = [(str(s["type"]), float(s["x"]), float(s["y"])) for s in d.get("slots", [])]
        self.layout = {"size": np.array(d.get("size", [0.0, 0.0]), float), "slots": slots,
                       "xy": np.array([[x, y] for _, x, y in slots]).reshape(-1, 2),
                       "types": [t for t, _, _ in slots]}
        # match targets: every hole (for HOLE detections and parts that sit on one hole), and for
        # PAIR_TYPES every pair of neighbouring holes (for a part that sits across two)
        targets = [{"key": t, "xy": np.array([x, y]), "members": [j]} for j, (t, x, y) in enumerate(slots)]
        for j1, (t1, x1, y1) in enumerate(slots):
            for j2 in range(j1 + 1, len(slots)):
                t2, x2, y2 = slots[j2]
                if t1 == t2 and t1 in PAIR_TYPES and PAIR_DIST_M[0] < np.hypot(x2 - x1, y2 - y1) < PAIR_DIST_M[1]:
                    targets.append({"key": t1 + "|place", "xy": np.array([(x1 + x2) / 2, (y1 + y2) / 2]),
                                    "members": [j1, j2]})
        self.layout["targets"] = targets
        self.layout["t_xy"] = np.array([tg["xy"] for tg in targets]).reshape(-1, 2)
        self.layout["t_keys"] = [tg["key"] for tg in targets]
        n_places = len(targets) - len(slots)
        self.log(f"tray '{self.cls_name}': slot layout {p} ({len(slots)} slots, {n_places} two-hole places, "
                 f"{self.layout['size'][0] * 1000:.0f} x {self.layout['size'][1] * 1000:.0f} mm)")

    def save_layout(self):
        """Save the slots of the last observation in the outline frame -> (ok, message)."""
        o = self.last
        if o is None:
            return False, f"no '{self.cls_name}' seen yet"
        if o["clipped"]:
            return False, "the tray touches the image border: put the whole tray in view"
        if len(o["slots"]) < 3:
            return False, f"only {len(o['slots'])} slots seen; need at least 3"
        T, R2 = o["rect_t"], _rot2(o["rect_a"])
        rows, kept = [], []
        for s_ in o["slots"]:
            typ, p = s_["type"], s_["p"]
            q = R2.T @ (p - T)
            if any(t == typ and np.linalg.norm(q - k) < 0.01 for t, k in kept):
                continue                                      # hole and its part: the same slot
            kept.append((typ, q))
            rows.append({"type": typ, "x": round(float(q[0]), 4), "y": round(float(q[1]), 4)})
        import yaml
        self.layout_path.parent.mkdir(parents=True, exist_ok=True)
        self.layout_path.write_text(
            "# Tray slot layout, saved with /karis/save_tray_layout (or tray_slots_test.py).\n"
            "# Slots may be deleted here (e.g. a hole the detector labels wrongly).\n"
            "# Tray frame: origin = centre of the top surface, z up, x along the long side\n"
            "# (pointing up in the image when it was saved), metres.\n"
            + yaml.safe_dump({"class": self.cls_name, "saved": time.strftime("%Y-%m-%d %H:%M:%S"),
                              "size": [round(float(v), 4) for v in o["rect_size"]],
                              "slots": rows}, sort_keys=False, allow_unicode=True))
        self.load_layout()
        return True, f"saved {len(rows)} slots to {self.layout_path}"

    # ------------------------------------------------------------ per detection frame
    def update(self, dets, depth, K):
        """dets: this frame's detections (all classes). Updates self.pose (4x4, camera <- tray)."""
        trays = [d for d in dets if d["cls_name"] == self.cls_name]
        if not trays:
            self.missed += 1
            if self.missed >= LOST_AFTER:
                self.pose, self.matched_px, self.slots, self.pending = None, [], [], None
            return
        self.missed = 0
        tray = max(trays, key=lambda d: np.count_nonzero(d["mask"]))
        self.cls_id = int(tray["cls_id"])
        o = self._observe(tray, dets, depth, K)
        if o is None:
            return
        self.last = o
        pose2d, method, n = self._fit(o)
        if pose2d is None:
            return
        a, t = pose2d
        R2 = _rot2(a)
        x3 = o["e1"] * R2[0, 0] + o["e2"] * R2[1, 0]
        z3 = o["n"]
        y3 = np.cross(z3, x3)
        P = np.eye(4)
        P[:3, :3] = np.stack([x3, y3, z3], axis=1)
        P[:3, 3] = o["c0"] + t[0] * o["e1"] + t[1] * o["e2"]
        if self.pose is not None and self._jump(P, self.pose):
            # a big jump: only taken when the next detection frame agrees (else a hand over the
            # tray could move it for one frame)
            if self.pending is None or self._jump(P, self.pending):
                self.pending = P
                return
        self.pending = None
        self.pose = self._smooth(P)
        self.size = self.layout["size"] if self.layout is not None and self.layout["size"].all() \
            else o["rect_size"]
        self.conf, self.method, self.n_matched = float(tray.get("conf", 0.0)), method, n
        self.t_seen = time.perf_counter()
        self._overlay(o, K)
        self.slots = self._slot_poses(o)

    def _slot_poses(self, o):
        """Slots with the tray's orientation. Each: {"id", "type", "state", "pose" (4x4 camera <-
        slot: centre on the tray surface, rotation = the tray's), "xy_tray" (m, in the tray frame),
        "conf", "cls", "source"}. Id "<type> hole <n>", n counted per type ("Ratblock Chamfer hole 1").
        With a layout: one per layout slot, n in layout order (fixed), state empty (its HOLE is
        detected: position = the detected hole centre on the tray plane), filled (a part lies in
        it) or not_seen (both: position = the layout slot on the fitted tray);
        detections that match no layout slot are added as "<type> hole extra <n>". Without a
        layout: one per detection on the tray, n in detection order (not fixed)."""
        P = self.pose
        R = P[:3, :3]

        def pose_at(xy):
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = P[:3, 3] + R[:, 0] * xy[0] + R[:, 1] * xy[1]
            return T

        def tray_xy(p2):                      # plane coords -> tray frame (x, y)
            q = o["c0"] + p2[0] * o["e1"] + p2[1] * o["e2"] - P[:3, 3]
            return np.array([q @ R[:, 0], q @ R[:, 1]])

        out, used, count = [], set(), {}

        def next_id(typ, extra=""):
            count[(typ, extra)] = count.get((typ, extra), 0) + 1
            return f"{typ} hole {extra}{count[(typ, extra)]}"

        if self.layout is not None and self.method == "slots":
            # every detection on the tray -> its nearest target with the same key (hole -> hole,
            # part -> its hole, or its two-hole place) within SLOT_MATCH_M, at the final pose
            by_layout = {}
            tx, tk, tg = self.layout["t_xy"], self.layout["t_keys"], self.layout["targets"]
            for i, s_ in enumerate(o["slots"]):
                d = np.linalg.norm(tx - tray_xy(s_["p"]), axis=1)
                d[[k != s_["key"] for k in tk]] = np.inf
                if len(d) and d.min() < SLOT_MATCH_M:
                    for j in tg[int(d.argmin())]["members"]:     # a place covers both its holes
                        by_layout.setdefault(j, []).append(i)
            for j, (typ, x, y) in enumerate(self.layout["slots"]):
                hits = [o["slots"][i] for i in by_layout.get(j, [])]
                for i in by_layout.get(j, []):
                    used.add(i)
                holes = [h for h in hits if h["hole"]]
                # a part on a slot covers its hole; a hole seen = empty
                state = ("filled" if any(not h["hole"] for h in hits) else
                         "empty" if hits else "not_seen")
                # empty: the detected hole centre (on the tray plane), measured directly;
                # filled / not_seen: the layout position on the fitted tray
                if state == "empty":
                    xy, src = np.mean([tray_xy(h["p"]) for h in holes], axis=0), "detected"
                else:
                    xy, src = np.array([x, y]), "layout"
                out.append({"id": next_id(typ), "type": typ, "state": state, "source": src,
                            "pose": pose_at(xy), "xy_tray": xy, "xy_layout": np.array([x, y]),
                            "conf": max((h["conf"] for h in hits), default=0.0),
                            "cls": hits[0]["cls"] if hits else ""})
        extra = "extra " if self.layout is not None and self.method == "slots" else ""
        for i, s_ in enumerate(o["slots"]):
            if i in used:
                continue
            xy = tray_xy(s_["p"])
            out.append({"id": next_id(s_["type"], extra), "type": s_["type"], "source": "detected",
                        "state": "empty" if s_["hole"] else "filled", "pose": pose_at(xy),
                        "xy_tray": xy, "conf": s_["conf"], "cls": s_["cls"]})
        return out

    def _observe(self, tray, dets, depth, K):
        mask = tray["mask"]
        h, w = mask.shape
        ys, xs = np.nonzero(mask & (depth > 0))
        if len(xs) < 200:
            return None
        sel = np.random.default_rng(0).choice(len(xs), min(PLANE_POINTS, len(xs)), replace=False)
        u, v, z = xs[sel], ys[sel], depth[ys[sel], xs[sel]]
        pts = np.stack([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z], 1)
        plane = self._ransac_plane(pts)
        if plane is None:
            return None
        n, c0 = plane
        cam_x = np.array([1.0, 0.0, 0.0])
        e1 = cam_x - n * (n @ cam_x)
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(n, e1)
        Kinv = np.linalg.inv(K)

        def to_plane(uv):
            """pixels (N,2) -> 2D plane coordinates (N,2), by ray / plane intersection"""
            d = (Kinv @ np.c_[uv, np.ones(len(uv))].T).T
            s = (n @ c0) / (d @ n)
            p = d * s[:, None] - c0
            return np.stack([p @ e1, p @ e2], 1)

        cs, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        c = max(cs, key=len)[:, 0, :].astype(np.float64)
        inside = ((c[:, 0] > BORDER_PX) & (c[:, 0] < w - 1 - BORDER_PX)
                  & (c[:, 1] > BORDER_PX) & (c[:, 1] < h - 1 - BORDER_PX))
        q = to_plane(c)
        # edge direction modulo 90 deg: circular mean of 4*angle over short outline segments
        k = 6
        a_idx = np.arange(0, len(q) - k, 2)
        seg = q[a_idx + k] - q[a_idx]
        ok = inside[a_idx] & inside[a_idx + k]
        L = np.linalg.norm(seg, axis=1)
        ang = np.arctan2(seg[:, 1], seg[:, 0])
        if ok.sum() < 20:
            return None
        th = np.angle(np.sum(L[ok] * np.exp(4j * ang[ok]))) / 4
        for _ in range(2):                                   # again with the straight edges only
            dev = np.abs(np.angle(np.exp(4j * (ang - th)))) / 4
            good = ok & (dev < np.radians(5))
            if good.sum() < 10:
                break
            th = np.angle(np.sum(L[good] * np.exp(4j * ang[good]))) / 4
        # outline extent along the two directions (long side -> x)
        qi = q[inside] if inside.sum() > 20 else q
        r = qi @ _rot2(th)
        lo, hi = np.percentile(r, 1, axis=0), np.percentile(r, 99, axis=0)
        ext, mid = hi - lo, _rot2(th) @ ((hi + lo) / 2)
        a_long = th if ext[0] >= ext[1] else th + np.pi / 2
        # x along the long side, pointing up in the image (camera -y)
        x3 = e1 * np.cos(a_long) + e2 * np.sin(a_long)
        if x3[1] > 0:
            a_long += np.pi
        slots = []
        for d in dets:
            if d is tray:
                continue
            typ = slot_type(d["cls_name"])
            if typ == self.cls_name:
                continue
            cu, cv_ = d["center_px"]
            if not mask[int(round(cv_)), int(round(cu))]:
                continue                                      # not on the tray
            hole = typ != d["cls_name"]
            p_ray = to_plane(np.array([[cu, cv_]]))[0]
            p, how = p_ray, "ray"
            if not hole:
                # a part sticks up: the ray through its mask centre meets the tray behind it.
                # Its 3D centre from depth, dropped straight down onto the tray plane instead.
                m = cv2.erode(d["mask"].astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
                yy, xx = np.nonzero(m & (depth > 0))
                if len(xx) >= PART_DEPTH_MIN_PX:
                    zz = depth[yy, xx]
                    P3 = np.stack([(xx - K[0, 2]) * zz / K[0, 0], (yy - K[1, 2]) * zz / K[1, 1], zz], 1)
                    above = (P3 - c0) @ n
                    P3 = P3[(above > -0.005) & (above < 0.08)]          # on/above the tray only
                    if len(P3) >= PART_DEPTH_MIN_PX:
                        q3 = P3.mean(0) - c0
                        p, how = np.array([q3 @ e1, q3 @ e2]), "depth"
            key = typ + "|place" if (not hole and typ in PAIR_TYPES) else typ
            slots.append({"cls": d["cls_name"], "type": typ, "hole": hole, "key": key, "how": how,
                          "conf": float(d.get("conf", 0.0)), "px": (float(cu), float(cv_)),
                          "p": p, "p_ray": p_ray})
        ol = q[inside]
        if len(ol) > 300:
            ol = ol[np.linspace(0, len(ol) - 1, 300).astype(int)]
        return {"n": n, "c0": c0, "e1": e1, "e2": e2, "theta": th, "slots": slots, "outline": ol,
                "rect_a": a_long, "rect_t": mid, "rect_size": np.sort(ext)[::-1].copy(),
                "clipped": bool((~inside).sum() > 3)}

    @staticmethod
    def _ransac_plane(pts, iters=150):
        rng = np.random.default_rng(1)
        tri = pts[rng.integers(0, len(pts), (iters, 3))]               # iters hypotheses at once
        N = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        ln = np.linalg.norm(N, axis=1)
        ok = ln > 1e-9
        if not ok.any():
            return None
        N, A = N[ok] / ln[ok, None], tri[ok, 0]
        counts = (np.abs(pts @ N.T - np.sum(N * A, 1)) < PLANE_TOL_M).sum(0)
        k = int(np.argmax(counts))
        if counts[k] < 100:
            return None
        P = pts[np.abs((pts - A[k]) @ N[k]) < PLANE_TOL_M]
        c0 = P.mean(0)
        n = np.linalg.eigh((P - c0).T @ (P - c0))[1][:, 0]    # smallest-variance direction
        if n @ c0 > 0:                                       # towards the camera
            n = -n
        return n, c0

    def _fit(self, o):
        """-> ((angle of tray x in the plane basis, tray origin in the plane), method, n matched)"""
        self._pairs_last = []
        if self.layout is None or len(self.layout["slots"]) == 0:
            return (o["rect_a"], o["rect_t"]), "outline", 0
        L, types = self.layout["t_xy"], self.layout["t_keys"]
        obs = o["slots"]                      # holes AND parts choose the position
        half = self.layout["size"] / 2 + OUTLINE_MARGIN_M if self.layout["size"].all() else None
        ol = o["outline"]

        def outline_in(a, t):
            """fraction of the visible tray outline inside the tray rectangle at pose (a, t)"""
            if half is None or len(ol) == 0:
                return 1.0
            q = (ol - t) @ _rot2(a)                # plane -> tray frame
            return float(np.mean((np.abs(q[:, 0]) <= half[0]) & (np.abs(q[:, 1]) <= half[1])))

        best = None
        if obs:
            O = np.array([s_["p"] for s_ in obs])
            same = np.array([[s_["key"] == tj for tj in types] for s_ in obs])   # obs x target
        for kq in range(4 if obs else 0):
            a = o["theta"] + kq * np.pi / 2
            Lr = L @ _rot2(a).T
            # translation votes: every same-type (observed i, layout j) pair; score = observed
            # slots with a same-type layout slot within SLOT_MATCH_M, then the mean distance
            for i, j in zip(*np.nonzero(same)):
                t = O[i] - Lr[j]
                D = np.linalg.norm(O[:, None, :] - (Lr + t)[None, :, :], axis=2)
                D[~same] = np.inf
                dmin = D.min(1)
                hit = dmin < SLOT_MATCH_M
                if hit.sum() < MIN_SLOT_MATCHES:
                    continue
                if best is not None and best[0][0] and hit.sum() < best[0][1]:
                    continue                  # cannot beat the best: skip the outline check
                frac = outline_in(a, t)
                # the outline must agree; then most matches; then the outline; then the distance
                key = (frac >= OUTLINE_MIN, int(hit.sum()), round(frac, 2), -float(dmin[hit].mean()))
                if best is None or key > best[0]:
                    best = (key, a, t)
        if best is not None:
            _, a, t = best
            pairs = self._pairs(obs, L @ _rot2(a).T + t, types)
            ok = len(pairs) >= MIN_SLOT_MATCHES and best[0][0]
            best = (best[0], a, pairs) if ok else None
        if best is None:
            if self.pose is None and time.time() - self.warned > 10:
                self.warned = time.time()
                self.log(f"tray: no pose that matches {MIN_SLOT_MATCHES}+ slots and the tray outline yet")
            return (None, "", 0)
        _, a0, pairs = best
        self._pairs_last = pairs
        hole_pairs = [pr for pr in pairs if obs[pr[0]]["hole"]]
        if len(hole_pairs) >= HOLES_FIRST:
            pairs = hole_pairs                # refine on the holes: flat and exact
        # least-squares 2D rotation + translation on the matched slots
        A = np.array([L[j] for _, j, _ in pairs])
        B = np.array([obs[i]["p"] for i, _, _ in pairs])
        if len(pairs) >= 2:
            ca, cb = A.mean(0), B.mean(0)
            H = (A - ca).T @ (B - cb)
            a = np.arctan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
            if abs(np.angle(np.exp(1j * (a - a0)))) > np.radians(MAX_REFINE_DEG):
                a = a0
            t = cb - _rot2(a) @ ca
        else:
            a, t = a0, B[0] - _rot2(a0) @ A[0]
        return (a, t), "slots", len(pairs)

    @staticmethod
    def _pairs(obs, Lt, types):
        """Greedy one-to-one matches (obs index, layout index, distance) within SLOT_MATCH_M."""
        cand = []
        for i, s_ in enumerate(obs):
            for j, tj in enumerate(types):
                if tj == s_["key"]:
                    d = float(np.linalg.norm(s_["p"] - Lt[j]))
                    if d < SLOT_MATCH_M:
                        cand.append((d, i, j))
        used_i, used_j, out = set(), set(), []
        for d, i, j in sorted(cand):
            if i in used_i or j in used_j:
                continue
            used_i.add(i)
            used_j.add(j)
            out.append((i, j, d))
        return out

    @staticmethod
    def _jump(P, Q):
        dt = np.linalg.norm(P[:3, 3] - Q[:3, 3])
        c = (np.trace(P[:3, :3] @ Q[:3, :3].T) - 1) / 2
        return dt > JUMP_M or np.degrees(np.arccos(np.clip(c, -1, 1))) > JUMP_DEG

    def _smooth(self, P):
        Q = self.pose
        if Q is None or SMOOTH <= 0:
            return P
        dt = np.linalg.norm(P[:3, 3] - Q[:3, 3])
        c = (np.trace(P[:3, :3] @ Q[:3, :3].T) - 1) / 2
        if dt > SMOOTH_MAX_M or np.degrees(np.arccos(np.clip(c, -1, 1))) > SMOOTH_MAX_DEG:
            return P                                         # moved: take the new pose
        out = np.eye(4)
        out[:3, 3] = SMOOTH * Q[:3, 3] + (1 - SMOOTH) * P[:3, 3]
        U, _, Vt = np.linalg.svd(SMOOTH * Q[:3, :3] + (1 - SMOOTH) * P[:3, :3])
        out[:3, :3] = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt
        return out

    def _overlay(self, o, K):
        """Pixels of the layout slots at the current pose (drawn on the annotated image)."""
        self.matched_px = []
        if self.layout is None or self.pose is None:
            return
        for x, y in self.layout["xy"]:
            p = self.pose[:3, :3] @ np.array([x, y, 0.0]) + self.pose[:3, 3]
            if p[2] > 0:
                self.matched_px.append((K[0, 0] * p[0] / p[2] + K[0, 2], K[1, 1] * p[1] / p[2] + K[1, 2]))

    # ------------------------------------------------------------ drawing
    def draw(self, vis, K, color):
        if self.pose is None:
            return vis
        P, (lx, ly) = self.pose, self.size

        def px(p):
            q = P[:3, :3] @ p + P[:3, 3]
            return (int(K[0, 0] * q[0] / q[2] + K[0, 2]), int(K[1, 1] * q[1] / q[2] + K[1, 2]))

        corners = [px(np.array([sx * lx / 2, sy * ly / 2, 0.0])) for sx, sy in
                   ((1, 1), (1, -1), (-1, -1), (-1, 1))]
        cv2.polylines(vis, [np.array(corners, np.int32)], True, color, 2, cv2.LINE_AA)
        o = px(np.zeros(3))
        s = 0.35 * min(lx, ly)
        for axis, col in ((np.array([s, 0, 0]), (255, 0, 0)), (np.array([0, s, 0]), (0, 255, 0)),
                          (np.array([0, 0, s]), (0, 0, 255))):        # RGB image: x red, y green, z blue
            cv2.arrowedLine(vis, o, px(axis), col, 2, cv2.LINE_AA, tipLength=0.15)
        # every slot: short tray-oriented axes at its centre and its id
        cols = {"empty": (0, 220, 0), "filled": (255, 160, 0), "not_seen": (160, 160, 160)}
        for s_ in self.slots:
            T = s_["pose"]

            def spx(p, T=T):
                q = T[:3, :3] @ p + T[:3, 3]
                return (int(K[0, 0] * q[0] / q[2] + K[0, 2]), int(K[1, 1] * q[1] / q[2] + K[1, 2]))

            c = spx(np.zeros(3))
            cv2.arrowedLine(vis, c, spx(np.array([0.02, 0, 0])), (255, 0, 0), 1, cv2.LINE_AA, tipLength=0.3)
            cv2.arrowedLine(vis, c, spx(np.array([0, 0.02, 0])), (0, 255, 0), 1, cv2.LINE_AA, tipLength=0.3)
            cv2.circle(vis, c, 4, cols.get(s_["state"], (255, 255, 255)), -1, cv2.LINE_AA)
            short = "".join(w[0] for w in s_["type"].split()).upper() + s_["id"].rsplit(" ", 1)[-1]
            cv2.putText(vis, short, (c[0] + 5, c[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1, cv2.LINE_AA)
        return vis
