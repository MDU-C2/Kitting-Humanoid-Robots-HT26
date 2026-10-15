#!/usr/bin/env python3
"""
KARIS: G1 head camera (D435i) -> ROS 2. Runs on the G1 developer computer (PC2,
192.168.123.164, ROS 2 Foxy). Replaces g1_stream_server.py: instead of a private
TCP stream, the frames go out as normal ROS 2 topics that any computer on the
robot network can subscribe to.

Published (namespace /g1/head_camera, the same names realsense2_camera uses):
    color/image_raw/compressed                    sensor_msgs/CompressedImage  JPEG
    aligned_depth_to_color/image_raw/compressedDepth  sensor_msgs/CompressedImage  16-bit PNG, mm
    color/camera_info                             sensor_msgs/CameraInfo
    aligned_depth_to_color/camera_info            sensor_msgs/CameraInfo (same K: depth is aligned)
  with publish_raw:=true also the uncompressed
    color/image_raw  (bgr8)  and  aligned_depth_to_color/image_raw  (16UC1)
  (about 15 MB/s at 15 Hz: fine on the cable, too much for WiFi).

Colour and depth of one frame carry the SAME stamp, and the depth is aligned
to the colour camera (rs.align), exactly like g1_capture.py and g1_stream_server.py.
The compressedDepth format is the one image_transport uses, so rqt_image_view and
`ros2 run image_transport republish` can read it too.

Run on the G1 (see start_head_camera.sh, which sets up the environment):
    sudo systemctl stop teleimager.service      # only one program can use the camera
    bash start_head_camera.sh
    bash start_head_camera.sh --ros-args -p publish_rate:=30.0 -p width:=1280 -p height:=720

Parameters (--ros-args -p name:=value):
    width, height, fps    camera mode (640, 480, 30)
    publish_rate          frames per second sent out (15.0); the camera keeps running at fps
    jpeg_quality          colour compression (90)
    png_level             depth PNG compression 0-9 (1 = fast)
    publish_raw           also publish uncompressed images (false)
    frame_id              TF frame of the images (head_camera_color_optical_frame)
"""
import array
import struct
import threading
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile
try:
    from rclpy.qos import ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
except ImportError:                     # older rclpy names
    from rclpy.qos import (QoSReliabilityPolicy as ReliabilityPolicy,
                           QoSHistoryPolicy as HistoryPolicy, QoSDurabilityPolicy as DurabilityPolicy)
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

# compressed_depth_image_transport puts this 12-byte header in front of the PNG:
# int32 format (0 = INV_DEPTH, unused for 16UC1), float32 depthQuantA, float32 depthQuantB
DEPTH_HEADER = struct.pack("<iff", 0, 0.0, 0.0)

# images: newest frame only, never block the camera (subscribers should use the same)
IMAGE_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=1)
# camera_info: reliable + transient local, so a late subscriber still gets it
INFO_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                      depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class RealSense:
    """Aligned colour + depth in a background thread; keeps only the newest frame."""

    def __init__(self, width, height, fps, log):
        self.log = log
        self.pipe, profile = self._start(width, height, fps)
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        self.intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.align = rs.align(rs.stream.color)
        self.lock = threading.Lock()
        self.latest = None          # (seq, time.time(), bgr, depth_uint16)
        self.seq = 0
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _start(self, width, height, fps, attempts=3):
        """If the camera sends nothing (common right after teleimager used it): reset, retry."""
        for attempt in range(1, attempts + 1):
            pipe, cfg = rs.pipeline(), rs.config()
            cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
            cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
            try:
                profile = pipe.start(cfg)
                pipe.wait_for_frames(15000)
                for _ in range(30):                     # let auto exposure settle
                    pipe.wait_for_frames(5000)
                return pipe, profile
            except RuntimeError as e:
                self.log.warn(f"No frames from the camera ({e}), attempt {attempt}/{attempts}")
                try:
                    pipe.stop()
                except RuntimeError:
                    pass
                if attempt == attempts:
                    raise RuntimeError(
                        "The camera sends no frames. Is something else using it? "
                        "(sudo systemctl stop teleimager.service, or "
                        "sudo /unitree/sbin/mscli stopservice video_hub_pc4). "
                        "Otherwise replug it / reboot the G1.") from e
                devs = rs.context().query_devices()
                if len(devs):
                    self.log.info("Resetting the camera...")
                    devs[0].hardware_reset()
                    time.sleep(6)

    def _loop(self):
        while self.running:
            try:
                fs = self.align.process(self.pipe.wait_for_frames(5000))
            except RuntimeError as e:
                self.log.warn(f"camera: {e}")
                continue
            c, d = fs.get_color_frame(), fs.get_depth_frame()
            if not c or not d:
                continue
            bgr = np.asanyarray(c.get_data()).copy()
            depth = np.asanyarray(d.get_data()).copy()
            with self.lock:
                self.seq += 1
                self.latest = (self.seq, time.time(), bgr, depth)

    def get(self):
        with self.lock:
            return self.latest

    def stop(self):
        self.running = False
        self.thread.join(timeout=2)
        try:
            self.pipe.stop()
        except RuntimeError:
            pass


class G1CameraNode(Node):
    def __init__(self):
        super().__init__("g1_head_camera")
        p = self.declare_parameter
        width = p("width", 640).value
        height = p("height", 480).value
        fps = p("fps", 30).value
        rate = float(p("publish_rate", 15.0).value)
        self.jpeg_quality = int(p("jpeg_quality", 90).value)
        self.png_level = int(p("png_level", 1).value)
        self.publish_raw = bool(p("publish_raw", False).value)
        self.frame_id = p("frame_id", "head_camera_color_optical_frame").value
        ns = p("topic_prefix", "/g1/head_camera").value.rstrip("/")

        self.cam = RealSense(width, height, fps, self.get_logger())
        if abs(self.cam.depth_scale - 0.001) > 1e-6:
            self.get_logger().warn(f"depth scale is {self.cam.depth_scale}, not 0.001: "
                                   "the published depth is still in camera units")
        self.info = self._camera_info()

        self.pub_c = self.create_publisher(CompressedImage, f"{ns}/color/image_raw/compressed", IMAGE_QOS)
        self.pub_d = self.create_publisher(
            CompressedImage, f"{ns}/aligned_depth_to_color/image_raw/compressedDepth", IMAGE_QOS)
        self.pub_ci = self.create_publisher(CameraInfo, f"{ns}/color/camera_info", INFO_QOS)
        self.pub_di = self.create_publisher(CameraInfo, f"{ns}/aligned_depth_to_color/camera_info", INFO_QOS)
        if self.publish_raw:
            self.pub_rc = self.create_publisher(Image, f"{ns}/color/image_raw", IMAGE_QOS)
            self.pub_rd = self.create_publisher(Image, f"{ns}/aligned_depth_to_color/image_raw", IMAGE_QOS)

        self.last_seq, self.n_sent, self.t_report, self.bytes_sent = -1, 0, time.time(), 0
        self.timer = self.create_timer(1.0 / rate, self.publish)
        i = self.cam.intr
        self.get_logger().info(
            f"Publishing {i.width}x{i.height} at {rate:.0f} Hz on {ns}/... "
            f"(fx={i.fx:.1f} cx={i.ppx:.1f} cy={i.ppy:.1f}, frame '{self.frame_id}')")

    def _camera_info(self):
        i = self.cam.intr
        m = CameraInfo()
        m.header.frame_id = self.frame_id
        m.width, m.height = i.width, i.height
        m.distortion_model = "plumb_bob"
        m.d = [float(x) for x in list(i.coeffs)[:5]]
        m.k = [i.fx, 0.0, i.ppx, 0.0, i.fy, i.ppy, 0.0, 0.0, 1.0]
        m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        m.p = [i.fx, 0.0, i.ppx, 0.0, 0.0, i.fy, i.ppy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return m

    def publish(self):
        frame = self.cam.get()
        if frame is None or frame[0] == self.last_seq:
            return
        seq, t_cap, bgr, depth = frame
        self.last_seq = seq
        # stamp = capture time (robot clock; run chrony/NTP on both computers for latency numbers)
        stamp = Time(nanoseconds=int(t_cap * 1e9)).to_msg()

        ok_c, jpg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        ok_d, png = cv2.imencode(".png", depth, [cv2.IMWRITE_PNG_COMPRESSION, self.png_level])
        if not (ok_c and ok_d):
            self.get_logger().warn("encoding failed")
            return
        c = CompressedImage()
        c.header.stamp, c.header.frame_id = stamp, self.frame_id
        c.format = "bgr8; jpeg compressed bgr8"
        c.data = array.array("B", jpg.tobytes())   # array: fast, no per-byte check
        d = CompressedImage()
        d.header.stamp, d.header.frame_id = stamp, self.frame_id
        d.format = "16UC1; compressedDepth png"
        d.data = array.array("B", DEPTH_HEADER + png.tobytes())
        self.info.header.stamp = stamp
        self.pub_c.publish(c)
        self.pub_d.publish(d)
        self.pub_ci.publish(self.info)
        self.pub_di.publish(self.info)
        if self.publish_raw:
            self.pub_rc.publish(self._raw(bgr, "bgr8", stamp))
            self.pub_rd.publish(self._raw(depth, "16UC1", stamp))

        self.n_sent += 1
        self.bytes_sent += len(c.data) + len(d.data)
        if time.time() - self.t_report > 10:
            el = time.time() - self.t_report
            self.get_logger().info(f"{self.n_sent / el:.1f} frames/s, "
                                   f"{self.bytes_sent / el / 1e6:.1f} MB/s compressed")
            self.n_sent, self.bytes_sent, self.t_report = 0, 0, time.time()

    def _raw(self, arr, encoding, stamp):
        m = Image()
        m.header.stamp, m.header.frame_id = stamp, self.frame_id
        m.height, m.width = arr.shape[:2]
        m.encoding = encoding
        m.is_bigendian = 0
        m.step = arr.strides[0]
        m.data = array.array("B", arr.tobytes())
        return m

    def destroy_node(self):
        self.cam.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = G1CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
