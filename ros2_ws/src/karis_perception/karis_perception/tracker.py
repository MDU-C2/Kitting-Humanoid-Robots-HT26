"""Per-frame pose tracking of all parts: detection, FoundationPose registration and tracking.

Parts are tracked from their previous pose every frame. Every REDETECT_EVERY frames the
detector runs: new parts are registered, parts whose silhouette IoU drops below
REREGISTER_IOU are registered again, and parts missed LOST_AFTER times are dropped.
Classes without a mesh (tray slots) only get a centre point.
"""
from __future__ import annotations

import csv
import time

import cv2
import numpy as np

from karis_perception import vision

# ------------------------------------------------------------ settings
TRACK_ITERATIONS = 1      # refinement iterations per tracked frame
REDETECT_EVERY = 5        # run the detector every N processed frames (1 = every frame)
REREGISTER_IOU = 0.2      # re-register a track if its silhouette IoU drops below this
                          # (hollow parts give ~0.4 even when correct)
LOST_AFTER = 3            # drop a track not seen in this many detections in a row
MATCH_PX = 40             # max distance [px] between a track and a detection to match
DRIFT_PX = 120            # unmatched detection this close to an unmatched track: same part, re-register
MAX_REGISTER_PER_DETECT = None   # registrations per detection frame after the first (None = all)


def project(K, p):
    return np.array([K[0, 0] * p[0] / p[2] + K[0, 2], K[1, 1] * p[1] / p[2] + K[1, 2]])


def track_pixel(trk, ests, K):
    """Pixel of the part's bounding-box centre at its current pose."""
    e = ests[trk["cls_name"]]
    c = (trk["pose"] @ np.linalg.inv(e["to_origin"]))[:3, 3]
    return project(K, c) if c[2] > 0 else np.array([-1e6, -1e6])


def rot_angle_deg(Ra, Rb):
    c = (np.trace(Ra @ Rb.T) - 1) / 2
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def draw_frame(rgb, tracks, slots, ests, K, fpu, info):
    vis = rgb.copy()
    for trk in tracks.values():
        e = ests[trk["cls_name"]]
        color = vision.class_color(trk["cls_id"])
        center_pose = trk["pose"] @ np.linalg.inv(e["to_origin"])
        vis = fpu.draw_posed_3d_box(K, img=vis, ob_in_cam=center_pose, bbox=e["bbox"])
        vis = fpu.draw_xyz_axis(vis, ob_in_cam=center_pose, scale=0.6 * e["extent"], K=K,
                                thickness=2, transparency=0, is_input_rgb=True)
        u, v = track_pixel(trk, ests, K)
        cv2.putText(vis, trk["id"], (int(u) - 30, int(v) - int(0.5 * e["extent"] * K[0, 0]
                    / max(center_pose[2, 3], 1e-3)) - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
    for s in slots:
        if s["centre"] is None:
            continue
        u, v = project(K, s["centre"])
        cv2.drawMarker(vis, (int(u), int(v)), vision.class_color(s["cls_id"]),
                       cv2.MARKER_CROSS, 10, 2)

    h = vis.shape[0]
    panel = np.full((h, 360, 3), 30, np.uint8)
    lines = [(f"frame {info['frame']}  t={info['t']:.2f}s", (255, 255, 255)),
             (f"{info['mode']}  {info['ms']:.0f} ms", (255, 255, 255)), ("", None)]
    for trk in tracks.values():
        t = trk["pose"][:3, 3]
        r = vision.rot_to_rpy_deg(trk["pose"][:3, :3])
        lines.append((trk["id"], vision.class_color(trk["cls_id"])))
        lines.append((f"  xyz {t[0]:+.3f} {t[1]:+.3f} {t[2]:+.3f} m", (220, 220, 220)))
        lines.append((f"  rpy {r[0]:+.0f} {r[1]:+.0f} {r[2]:+.0f} deg", (220, 220, 220)))
    y = 20
    for text, col in lines:
        if y > h - 8:
            break
        if text:
            cv2.putText(panel, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1, cv2.LINE_AA)
        y += 16
    return np.hstack([vis, panel])


class PoseTracker:
    """Detection + registration + tracking of every part, one frame at a time."""

    def __init__(self, model, ests, fpu, glctx, K, max_register=None, check=None):
        """check(entry, pose, K, depth, mask, fpu, glctx) -> (iou, depth_err, mask); default
        isaac_backend.silhouette_check."""
        self.model, self.ests, self.fpu, self.glctx, self.K = model, ests, fpu, glctx, K
        if check is None:
            from karis_perception.isaac_backend import silhouette_check as check
        self.check = check
        self.max_register = max_register if max_register is not None else MAX_REGISTER_PER_DETECT
        self.reg_times, self.track_times, self.det_times = [], [], []
        self.pose_rows, self.time_rows = [], []
        self.n_reregistered, self.n_dropped, self.n_new = 0, 0, 0
        self.reset()

    def reset(self):
        """Forget all parts; the next frame detects and registers from scratch."""
        self.tracks, self.slots, self.counters = {}, [], {}
        self.k = 0
        self.pending = False      # registrations left for the next frame

    def _register(self, trk, det, depth, rgb):
        est = self.ests[trk["cls_name"]]["est"]
        vision.cuda_sync()
        t0 = time.perf_counter()
        pose = est.register(K=self.K, rgb=rgb, depth=depth, ob_mask=det["mask"],
                            iteration=vision.REG_ITERATIONS)
        vision.cuda_sync()
        ms = (time.perf_counter() - t0) * 1000
        self.reg_times.append(ms)
        trk["pose"] = np.asarray(pose, np.float64).reshape(4, 4)
        trk["state"] = est.pose_last.clone()
        trk["mode"] = "register"
        trk["conf"] = det.get("conf")
        trk["iou"] = None
        return ms

    def _closest(self, det, track_ids, max_px):
        """Nearest track of the same class among track_ids, within max_px."""
        c = np.array(det["center_px"])
        best, best_d = None, max_px
        for tid in track_ids:
            trk = self.tracks[tid]
            if trk["cls_name"] != det["cls_name"]:
                continue
            d = np.linalg.norm(track_pixel(trk, self.ests, self.K) - c)
            if d < best_d:
                best, best_d = tid, d
        return best

    def _new_track(self, det, depth, rgb):
        n = self.counters.get(det["cls_name"], 0) + 1
        self.counters[det["cls_name"]] = n
        tid = f"{det['cls_name']}_{n}"
        trk = {"id": tid, "cls_name": det["cls_name"], "cls_id": det["cls_id"], "missed": 0}
        ms = self._register(trk, det, depth, rgb)
        self.tracks[tid] = trk
        return ms

    def step(self, bgr, depth, frame_no, t_rec, force_detect=False):
        """Process one frame. Returns the timing dict (with 'mode')."""
        K, ests = self.K, self.ests
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        tm = {"frame": frame_no, "time_s": round(t_rec, 3), "det_ms": 0.0, "n_register": 0,
              "register_ms": 0.0, "n_track": 0, "track_ms": 0.0}

        # 1. track existing parts from their previous pose
        for trk in self.tracks.values():
            est = ests[trk["cls_name"]]["est"]
            est.pose_last = trk["state"]
            vision.cuda_sync()
            t0 = time.perf_counter()
            pose = est.track_one(rgb=rgb, depth=depth, K=K, iteration=TRACK_ITERATIONS)
            vision.cuda_sync()
            ms = (time.perf_counter() - t0) * 1000
            self.track_times.append(ms)
            tm["n_track"] += 1
            tm["track_ms"] += ms
            trk["pose"] = np.asarray(pose, np.float64).reshape(4, 4)
            trk["state"] = est.pose_last.clone()
            trk["mode"] = "track"

        # 2. detection frame: find new parts, check / repair existing tracks
        detect_now = force_detect or self.pending or self.k % REDETECT_EVERY == 0
        if detect_now:
            first = not self.tracks
            vision.cuda_sync()
            t0 = time.perf_counter()
            dets = vision.detect(self.model, bgr, depth=depth)
            vision.cuda_sync()
            tm["det_ms"] = (time.perf_counter() - t0) * 1000
            self.det_times.append(tm["det_ms"])

            self.slots = []
            unmatched = set(self.tracks)
            new_dets, matched = [], []
            for det in dets:
                if det["cls_name"] not in ests:
                    self.slots.append({"cls_name": det["cls_name"], "cls_id": det["cls_id"],
                                       "conf": det.get("conf"),
                                       "centre": vision.centre_from_depth(det["mask"], depth, K)})
                    continue
                if np.count_nonzero(det["mask"] & (depth > 0)) < vision.MIN_VALID_DEPTH_PX:
                    continue
                best = self._closest(det, unmatched, MATCH_PX)
                if best is None:
                    new_dets.append(det)
                else:
                    unmatched.discard(best)
                    matched.append((best, det))

            # registrations: drifted and lost tracks first, then new parts
            todo = []
            for det in new_dets:
                drifted = self._closest(det, unmatched, DRIFT_PX)
                if drifted is not None:
                    unmatched.discard(drifted)
                    self.tracks[drifted]["missed"] = 0
                    todo.append((drifted, det))
                else:
                    todo.append((None, det))
            for tid, det in matched:
                trk = self.tracks[tid]
                trk["missed"] = 0
                trk["conf"] = det.get("conf")
                iou, _, _ = self.check(ests[trk["cls_name"]], trk["pose"], K, depth,
                                             det["mask"], self.fpu, self.glctx)
                trk["iou"] = iou
                if iou is not None and iou < REREGISTER_IOU:
                    todo.insert(0, (tid, det))
            todo.sort(key=lambda x: x[0] is None)

            limit = self.max_register if (self.max_register and not first) else len(todo)
            for tid, det in todo[:limit]:
                if tid is None:
                    tm["register_ms"] += self._new_track(det, depth, rgb)
                    self.n_new += 1
                else:
                    tm["register_ms"] += self._register(self.tracks[tid], det, depth, rgb)
                    self.n_reregistered += 1
                tm["n_register"] += 1
            self.pending = len(todo) > limit

            for tid in list(unmatched):
                self.tracks[tid]["missed"] += 1
                if self.tracks[tid]["missed"] >= LOST_AFTER:
                    del self.tracks[tid]
                    self.n_dropped += 1
        self.k += 1

        tm["frame_ms"] = tm["det_ms"] + tm["register_ms"] + tm["track_ms"]
        tm["mode"] = ("detect+register" if tm["n_register"] else
                      "detect+track" if detect_now else "track")
        self.time_rows.append(tm)

        for trk in self.tracks.values():
            P = trk["pose"]
            r = vision.rot_to_rpy_deg(P[:3, :3])
            q = vision.rot_to_quat_xyzw(P[:3, :3])
            self.pose_rows.append({
                "frame": frame_no, "time_s": round(t_rec, 3), "object_id": trk["id"],
                "class": trk["cls_name"], "mode": trk["mode"],
                "x_m": round(P[0, 3], 4), "y_m": round(P[1, 3], 4), "z_m": round(P[2, 3], 4),
                "roll_deg": round(r[0], 1), "pitch_deg": round(r[1], 1), "yaw_deg": round(r[2], 1),
                "qx": round(q[0], 5), "qy": round(q[1], 5), "qz": round(q[2], 5), "qw": round(q[3], 5),
                "_R": P[:3, :3].copy()})
        if detect_now:
            for i, s in enumerate(self.slots):
                if s["centre"] is not None:
                    self.pose_rows.append({
                        "frame": frame_no, "time_s": round(t_rec, 3),
                        "object_id": f"{s['cls_name']}_{i + 1}", "class": s["cls_name"],
                        "mode": "centre", "x_m": round(s["centre"][0], 4),
                        "y_m": round(s["centre"][1], 4), "z_m": round(s["centre"][2], 4)})
        return tm

    def draw(self, bgr, info):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        vis = draw_frame(rgb, self.tracks, self.slots, self.ests, self.K, self.fpu, info)
        return cv2.cvtColor(vis, cv2.COLOR_RGB2BGR)

    def save_csv(self, out_dir):
        p_fields = ["frame", "time_s", "object_id", "class", "mode", "x_m", "y_m", "z_m",
                    "roll_deg", "pitch_deg", "yaw_deg", "qx", "qy", "qz", "qw"]
        with open(out_dir / "poses.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=p_fields, extrasaction="ignore")
            w.writeheader()
            w.writerows(self.pose_rows)
        t_fields = ["frame", "time_s", "mode", "det_ms", "n_register", "register_ms",
                    "n_track", "track_ms", "frame_ms"]
        with open(out_dir / "timing.csv", "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=t_fields, extrasaction="ignore")
            w.writeheader()
            for r in self.time_rows:
                w.writerow({k: (round(v, 1) if isinstance(v, float) and k != "time_s" else v)
                            for k, v in r.items()})

    def print_summary(self):
        def stat(vals):
            vals = [v for v in vals if v]
            return (f"mean {np.mean(vals):7.1f}  median {np.median(vals):7.1f}  "
                    f"min {np.min(vals):7.1f}  max {np.max(vals):7.1f}") if vals else "-"

        rows = self.time_rows
        track_frames = [r["frame_ms"] for r in rows if r["mode"] == "track"]
        print(f"\nTiming over {len(rows)} frames (ms, GPU-synchronised):")
        print(f"  Detector per detection frame {stat(self.det_times)}")
        print(f"  Register per object          {stat(self.reg_times)}   "
              f"({len(self.reg_times)} registrations)")
        print(f"  Track per object             {stat(self.track_times)}")
        print(f"  Frame, tracking only         {stat(track_frames)}")
        print(f"  Frame, all                   {stat([r['frame_ms'] for r in rows])}")
        if track_frames:
            n_parts = np.median([r["n_track"] for r in rows if r["mode"] == "track"])
            print(f"  -> tracking rate: {1000 / np.median(track_frames):.1f} frames/s "
                  f"with {n_parts:.0f} parts")
        print(f"  Registrations: {self.n_new} new parts, {self.n_reregistered} re-registered "
              f"(lost or drifted); dropped {self.n_dropped} tracks")

        print("\nPose jitter per track (spread over all its frames; only meaningful if the "
              "camera and the part did not move):")
        print(f"  {'track':<26s} {'frames':>6s} {'pos std x/y/z [mm]':>22s} {'rot RMS [deg]':>14s}")
        by_track = {}
        for r in self.pose_rows:
            if "_R" in r:
                by_track.setdefault(r["object_id"], []).append(r)
        for tid, rs in by_track.items():
            xyz = np.array([[r["x_m"], r["y_m"], r["z_m"]] for r in rs]) * 1000
            Rs = [r["_R"] for r in rs]
            Rref = Rs[len(Rs) // 2]
            rot = np.sqrt(np.mean([rot_angle_deg(R, Rref) ** 2 for R in Rs]))
            sd = xyz.std(0)
            print(f"  {tid:<26s} {len(rs):>6d} {sd[0]:7.1f}{sd[1]:7.1f}{sd[2]:7.1f}     {rot:9.1f}")
