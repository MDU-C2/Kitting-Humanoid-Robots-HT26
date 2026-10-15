"""
KARIS camera player: plays a recording or the test photos as if it were the G1 head
camera, on the same ROS 2 topics as robot/g1_camera_ros2.py. Test the whole ROS 2 chain
(perception -> motion) without the robot.

    ros2 run karis_perception camera_player --ros-args -p source:=$KARIS_DIR/videos/20261006_101500.bag
    ros2 launch karis_bringup perception.launch.py player:=$KARIS_DIR/data      (the test photos)

source can be
    a RealSense .bag (from g1_capture.py; needs pyrealsense2; a bag that was not closed
      properly is repaired first, into <name>.reindexed.bag), or
    a folder with rgb/, depth/ (uint16 mm, aligned) and cam_K.txt plus timestamps.csv or
      more than 20 frames (g1_capture.py RECORD_FORMAT="png", live_foundationpose.py SAVE_RAW):
      played as a video, or
    the image-test folder data/ (rgb/, depth/, cam_K/<name>.txt or cam_K.txt): PHOTO MODE.
      Each photo is sent again and again (`still_rate` Hz) for `still_s` seconds, like a
      camera looking at a still scene, and the perception is reset (/karis/reset) at every
      new photo so the parts of the previous photo are not carried over. Make still_s long
      enough for the registrations.
Parameters: rate (Hz, 0 = speed of the recording), loop (true), still_s (20.0),
still_rate (2.0), reset_between_photos (true), topic_prefix (/g1/head_camera),
frame_id (head_camera_color_optical_frame).
"""
import csv
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
try:
    from rclpy.qos import ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
except ImportError:                     # older rclpy names
    from rclpy.qos import (QoSReliabilityPolicy as ReliabilityPolicy,
                           QoSHistoryPolicy as HistoryPolicy, QoSDurabilityPolicy as DurabilityPolicy)
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_srvs.srv import Trigger

from karis_perception import ros_convert as rc

IMAGE_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=1)
INFO_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                      depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
IMAGE_EXTS = {".png", ".jpg", ".jpeg"}


def bag_frames(path):
    """(iterator of (t_s, bgr, depth_u16, K), False) from a RealSense .bag."""
    import pyrealsense2 as rs
    pipe, cfg = rs.pipeline(), rs.config()
    rs.config.enable_device_from_file(cfg, str(path), repeat_playback=False)
    prof = pipe.start(cfg)
    dev = prof.get_device()
    dev.as_playback().set_real_time(False)
    scale = dev.first_depth_sensor().get_depth_scale()
    cs = prof.get_stream(rs.stream.color).as_video_stream_profile()
    i = cs.get_intrinsics()
    K = np.array([[i.fx, 0, i.ppx], [0, i.fy, i.ppy], [0, 0, 1.0]])
    fmt = cs.format()
    align = rs.align(rs.stream.color)

    def it():
        t0 = None
        try:
            while True:
                ok, fs = pipe.try_wait_for_frames(3000)
                if not ok:
                    break
                fs = align.process(fs)
                c, d = fs.get_color_frame(), fs.get_depth_frame()
                if not c or not d:
                    continue
                img = np.asanyarray(c.get_data()).copy()
                if fmt == rs.format.rgb8:
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                raw = np.asanyarray(d.get_data()).copy()
                if abs(scale - 0.001) > 1e-6:
                    raw = (raw.astype(np.float32) * scale * 1000).astype(np.uint16)
                t = c.get_timestamp() / 1000.0
                t0 = t if t0 is None else t0
                yield t - t0, img, raw, K
        finally:
            pipe.stop()
    return it(), False


def folder_frames(folder):
    """(iterator of (t_s, bgr, depth_u16, K), is_photos) from rgb/ + depth/ +
    cam_K.txt or cam_K/<stem>.txt (per photo, as in the image test)."""
    files = sorted(p for p in (folder / "rgb").iterdir() if p.suffix.lower() in IMAGE_EXTS)
    if not files:
        raise FileNotFoundError(f"no images in {folder / 'rgb'}")
    k_file = folder / "cam_K.txt"

    def load_K(stem):
        per = folder / "cam_K" / f"{stem}.txt"
        f = per if per.exists() else k_file
        if not f.exists():
            raise FileNotFoundError(f"no intrinsics for {stem}: {per} or {k_file}")
        return np.loadtxt(f).reshape(3, 3)

    stamps = {}
    if (folder / "timestamps.csv").exists():
        with open(folder / "timestamps.csv") as f:
            stamps = {int(r["frame"]): float(r["time_s"]) for r in csv.DictReader(f)}
    is_video = bool(stamps) or len(files) > 20

    def it():
        for k, f in enumerate(files):
            bgr = cv2.imdecode(np.fromfile(str(f), np.uint8), cv2.IMREAD_COLOR)
            dp = folder / "depth" / f"{f.stem}.png"
            if not dp.exists():
                print(f"  {f.name}: no depth/{f.stem}.png, skipped", flush=True)
                continue
            raw = cv2.imdecode(np.fromfile(str(dp), np.uint8), cv2.IMREAD_UNCHANGED)
            if raw is None or raw.shape[:2] != bgr.shape[:2] or raw.dtype != np.uint16:
                print(f"  {f.name}: depth must be uint16 and {bgr.shape[1]}x{bgr.shape[0]} "
                      "(aligned to colour), skipped", flush=True)
                continue
            n = int(f.stem) if f.stem.isdigit() else k
            yield stamps.get(n, k / 30.0), bgr, raw, load_K(f.stem)
    return it(), not is_video


class CameraPlayer(Node):
    def __init__(self):
        super().__init__("karis_camera_player")
        p = lambda n, v: self.declare_parameter(n, v).value   # noqa: E731
        self.source = Path(p("source", "")).expanduser()
        self.rate = float(p("rate", 0.0))
        self.loop = bool(p("loop", True))
        self.still_s = float(p("still_s", 20.0))
        self.still_rate = float(p("still_rate", 2.0))
        self.reset_photos = bool(p("reset_between_photos", True))
        self.frame_id = p("frame_id", "head_camera_color_optical_frame")
        ns = p("topic_prefix", "/g1/head_camera").rstrip("/")
        if not self.source.exists():
            raise FileNotFoundError(f"source '{self.source}' does not exist (-p source:=...)")
        self.pub_c = self.create_publisher(CompressedImage, f"{ns}/color/image_raw/compressed", IMAGE_QOS)
        self.pub_d = self.create_publisher(
            CompressedImage, f"{ns}/aligned_depth_to_color/image_raw/compressedDepth", IMAGE_QOS)
        self.pub_i = self.create_publisher(CameraInfo, f"{ns}/color/camera_info", INFO_QOS)
        self.reset_cli = self.create_client(Trigger, "/karis/reset")
        self.get_logger().info(f"Playing {self.source} on {ns}/...")

    def open(self):
        if self.source.suffix.lower() == ".bag":
            return bag_frames(self._indexed_bag(self.source))
        return folder_frames(self.source)

    def _indexed_bag(self, src):
        """A bag that was not closed properly ("Bag unindexed") gets its index rebuilt
        once, into <name>.reindexed.bag next to it (the original is left as it is)."""
        from karis_perception import bag_reindex
        if bag_reindex.is_indexed(src):
            return src
        dst = src.with_name(src.stem + ".reindexed.bag")
        if dst.exists() and dst.stat().st_mtime >= src.stat().st_mtime:
            return dst
        self.get_logger().warn(f"{src.name} was not closed properly (no index): repairing it "
                               f"into {dst.name}, once (can take a minute for a big file)")
        bag_reindex.reindex(src, dst, log=self.get_logger().info)
        return dst

    def _send(self, bgr, raw, info):
        stamp = self.get_clock().now().to_msg()
        info.header.stamp = stamp
        self.pub_c.publish(rc.bgr_to_compressed(bgr, stamp, self.frame_id, 90))
        self.pub_d.publish(rc.depth_to_compressed(raw, stamp, self.frame_id))
        self.pub_i.publish(info)

    def _sleep(self, until):
        while time.time() < until and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=min(0.05, max(0.0, until - time.time())))

    def run(self):
        while rclpy.ok():
            frames, photos = self.open()
            t_start, n = time.time(), 0
            for t_rec, bgr, raw, K in frames:
                if not rclpy.ok():
                    return
                info = self._info(K, bgr.shape)
                if photos:                                    # photo mode: a still scene
                    if self.reset_photos and self.reset_cli.service_is_ready():
                        self.reset_cli.call_async(Trigger.Request())
                    self.get_logger().info(f"Photo {n + 1}: showing it for {self.still_s:.0f} s")
                    t_end = time.time() + self.still_s
                    while time.time() < t_end and rclpy.ok():
                        self._send(bgr, raw, info)
                        self._sleep(min(t_end, time.time() + 1.0 / self.still_rate))
                else:
                    self._sleep(t_start + (n / self.rate if self.rate > 0 else t_rec))
                    self._send(bgr, raw, info)
                n += 1
            what = "photos" if photos else "frames"
            self.get_logger().info(f"End of the recording ({n} {what})" + (", again" if self.loop else ""))
            if not self.loop:
                return

    def _info(self, K, shape):
        m = CameraInfo()
        m.header.frame_id = self.frame_id
        m.height, m.width = shape[:2]
        m.distortion_model = "plumb_bob"
        m.d = [0.0] * 5
        m.k = [float(v) for v in K.reshape(-1)]
        m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        m.p = [float(v) for v in (K[0, 0], 0.0, K[0, 2], 0.0, 0.0, K[1, 1], K[1, 2], 0.0,
                                  0.0, 0.0, 1.0, 0.0)]
        return m


def main(args=None):
    rclpy.init(args=args)
    node = CameraPlayer()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
