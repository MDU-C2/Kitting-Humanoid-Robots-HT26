"""
KARIS pose estimator node: RGB-D from the G1 head camera (ROS 2 topics) ->
detector (YOLO / RF-DETR) -> FoundationPose 6D pose + tracking of every part ->
ROS 2 topics, TF and a service for motion planning.

The detector (detectors.py), the tracker (tracker.py) and the part classes with their meshes
(config/classes.yaml, meshes/) are in this package. Only the trained models come from the
local data folder (KARIS_DIR/models, not in git). Settings: karis_bringup/config/perception.yaml.

FoundationPose itself runs in the Isaac helper container: NVIDIA Isaac ROS FoundationPose
(TensorRT), reached through the service /karis/fp/estimate_pose (karis_isaac_fp). This
node only runs the detector and the tracking bookkeeping; the drift check and the drawing
are done on the CPU (isaac_backend). The perception image has no FoundationPose of its own.

Runs in the perception container (ROS 2 Humble):
    ros2 launch karis_bringup perception.launch.py model:=best8s.pt [backend:=isaac]
  or without launch:
    ros2 run karis_perception pose_estimator --ros-args -p model:=best8s.pt

Subscribes (parameters color_topic, depth_topic, info_topic; '.../compressed' and
'.../compressedDepth' topics are decoded here, others are raw sensor_msgs/Image):
    /g1/head_camera/color/image_raw/compressed
    /g1/head_camera/aligned_depth_to_color/image_raw/compressedDepth
    /g1/head_camera/color/camera_info

Publishes:
    /karis/object_poses        karis_interfaces/ObjectPoseArray  poses in the camera frame
    /karis/object_poses_base   karis_interfaces/ObjectPoseArray  the same in target_frame (e.g.
                               pelvis), when TF from the robot model is available
    /tf                        <camera frame> -> karis/<object_id>  for every part
    /karis/markers             visualization_msgs/MarkerArray  boxes + names for RViz
    /karis/viz/annotated/compressed   sensor_msgs/CompressedImage  the live overlay
                               (rqt_image_view /karis/viz/annotated, transport "compressed")
Services:
    /karis/get_objects   karis_interfaces/GetObjects   current parts, optionally in another frame
    /karis/reset         std_srvs/Trigger   forget all parts, detect + register again
    /karis/detect_now    std_srvs/Trigger   run the detector on the next frame
    /karis/reregister    std_srvs/Trigger   register every tracked part again (e.g. before a grasp)
"""
from __future__ import annotations

import os
import shutil
import tempfile
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
try:
    from rclpy.qos import ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
except ImportError:                     # older rclpy names
    from rclpy.qos import (QoSReliabilityPolicy as ReliabilityPolicy,
                           QoSHistoryPolicy as HistoryPolicy, QoSDurabilityPolicy as DurabilityPolicy)
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformBroadcaster

from karis_interfaces.msg import ObjectPose, ObjectPoseArray
from karis_interfaces.srv import GetObjects

from karis_perception import ros_convert as rc

IMAGE_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=2)
# camera_info: reliable + volatile matches every publisher (volatile and transient-local)
INFO_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                      depth=1, durability=DurabilityPolicy.VOLATILE)
POSE_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST, depth=5)
WINDOW = "KARIS pose estimator (ROS 2)"
# the local data folder is local_data/ in the repo (next to ros2_ws)
KARIS_DIR_GUESSES = [str(Path(os.environ["KARIS_WS"]).parent / "local_data")] if os.environ.get("KARIS_WS") else []
KARIS_DIR_GUESSES.append(str(Path.home() / "Kitting-Humanoid-Robots-HT26" / "local_data"))


def find_karis_dir(param_value):
    """The local data folder (models/, meshes_isaac/, data/, videos/): the parameter
    karis_dir, $KARIS_DIR, or local_data/ next to ros2_ws."""
    for d in [param_value, os.environ.get("KARIS_DIR", "")] + KARIS_DIR_GUESSES:
        if d and (Path(d) / "models").is_dir():
            return Path(d).resolve()
    raise FileNotFoundError(
        "Cannot find the KARIS data folder (with models/). Set KARIS_DIR or the parameter "
        "karis_dir, e.g. -p karis_dir:=$KARIS_REPO/local_data")


class PoseEstimatorNode(Node):
    def __init__(self):
        super().__init__("karis_pose_estimator")
        p = lambda n, v: self.declare_parameter(n, v).value   # noqa: E731
        self.karis_dir = find_karis_dir(p("karis_dir", ""))
        self.model_choice = str(p("model", "") or "")
        self.color_topic = p("color_topic", "/g1/head_camera/color/image_raw/compressed")
        self.depth_topic = p("depth_topic",
                             "/g1/head_camera/aligned_depth_to_color/image_raw/compressedDepth")
        self.info_topic = p("info_topic", "/g1/head_camera/color/camera_info")
        self.depth_scale = float(p("depth_scale", 0.001))     # 16UC1 units -> m
        self.max_dt = float(p("max_stamp_diff", 0.04))        # colour/depth pairing [s]
        self.camera_frame = p("camera_frame", "")             # "" = frame_id of the images
        self.target_frame = p("target_frame", "pelvis")       # "" = no /karis/object_poses_base
        self.tf_prefix = p("tf_prefix", "karis/")
        self.publish_tf = bool(p("publish_tf", True))
        # "now": outputs stamped with this computer's clock (works when the G1 and the laptop
        # clocks differ); "camera": the image stamps (needs chrony/NTP on both computers)
        self.stamp_source = p("stamp_source", "now")
        self.publish_annotated = bool(p("publish_annotated", True))
        self.annotated_quality = int(p("annotated_jpeg_quality", 80))
        self.show = bool(p("show", False))
        self.show_scale = float(p("show_scale", 2.0))         # window size relative to the image (2 = twice)
        self.models_dir = str(p("models_dir", "") or "")       # "" = <karis_dir>/models
        self.classes_file = str(p("classes_file", "") or "")   # "" = this package's config/classes.yaml
        self.confidence = float(p("confidence", 0.6))         # detector confidence threshold
        self.iou = float(p("iou", 0.6))                       # detector NMS IoU
        self.device = int(p("device", 0))                     # GPU index for YOLO
        self.min_mask_area = int(p("min_mask_area_px", 300))
        self.min_valid_depth = int(p("min_valid_depth_px", 50))
        self.min_depth = float(p("min_depth_m", 0.1))         # depth outside this range is ignored
        self.max_depth = float(p("max_depth_m", 3.0))
        self.redetect_every = int(p("redetect_every", 5))
        self.reregister_iou = float(p("reregister_iou", 0.2))  # drift check: register again below this
        self.settle = bool(p("settle_reregister", True))       # register again when a part comes to rest
        self.settle_move = float(p("settle_move_m", 0.02))
        self.settle_still = float(p("settle_still_s", 0.5))
        self.image_size = int(p("image_size", 0))              # YOLO input size; 0 = the model's training size
        self.max_register = int(p("max_register", 1))          # 0 = all at once every time
        self.backend = p("backend", "isaac")                   # only "isaac" (Isaac helper)
        self.isaac_service = p("isaac_service", "/karis/fp/estimate_pose")
        self.isaac_timeout = float(p("isaac_timeout_s", 25.0))
        self.info_msg = None

        self._load_vision()

        self.cam_K = None
        self.info_frame = ""
        self.colors, self.depths = {}, {}     # stamp_ns -> msg (a few newest)
        self.pending = None                   # newest (colour, depth) pair not processed yet
        self.tracker = None
        self.reset_req = self.detect_req = self.rereg_req = False
        self.n, self.t0 = 0, time.perf_counter()
        self.seen = {}                        # object_id -> time last confirmed by the detector
        self.last_cam = None                  # last ObjectPoseArray in the camera frame
        self.stats = {"frames": 0, "dropped": 0, "t": time.perf_counter(), "ms": []}
        self.tf_warned = 0.0

        ctype, dtype = rc.image_msg_type(self.color_topic), rc.image_msg_type(self.depth_topic)
        self.create_subscription(ctype, self.color_topic, self._on_color, IMAGE_QOS)
        self.create_subscription(dtype, self.depth_topic, self._on_depth, IMAGE_QOS)
        self.create_subscription(CameraInfo, self.info_topic, self._on_info, INFO_QOS)

        self.pub_cam = self.create_publisher(ObjectPoseArray, "/karis/object_poses", POSE_QOS)
        self.pub_base = self.create_publisher(ObjectPoseArray, "/karis/object_poses_base", POSE_QOS)
        self.pub_markers = self.create_publisher(MarkerArray, "/karis/markers", POSE_QOS)
        self.pub_annot = self.create_publisher(CompressedImage, "/karis/viz/annotated/compressed",
                                               IMAGE_QOS)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = TransformBroadcaster(self)

        self.create_service(GetObjects, "/karis/get_objects", self._srv_get_objects)
        self.create_service(Trigger, "/karis/reset", self._srv_reset)
        self.create_service(Trigger, "/karis/detect_now", self._srv_detect)
        self.create_service(Trigger, "/karis/reregister", self._srv_reregister)

        self.get_logger().info(
            f"Waiting for the camera: {self.color_topic} ({ctype.__name__}), "
            f"{self.depth_topic} ({dtype.__name__}), {self.info_topic}")

    # ------------------------------------------------------------ vision setup
    def _load_vision(self):
        from karis_perception import detectors, isaac_backend, tracker, trt_support, vision
        self.vision, self.trk = vision, tracker
        for k, v in {"CONFIDENCE": self.confidence, "IOU": self.iou, "DEVICE": self.device,
                     "MIN_MASK_AREA_PX": self.min_mask_area, "MIN_VALID_DEPTH_PX": self.min_valid_depth,
                     "MIN_DEPTH_M": self.min_depth, "MAX_DEPTH_M": self.max_depth}.items():
            setattr(vision, k, v)
        tracker.REDETECT_EVERY = max(1, self.redetect_every)
        tracker.REREGISTER_IOU = self.reregister_iou
        tracker.SETTLE_REREGISTER = self.settle
        tracker.SETTLE_MOVE_M = self.settle_move
        tracker.SETTLE_STILL_S = self.settle_still
        self.tmp_dir = Path(tempfile.mkdtemp(prefix="karis_ros_"))
        self.get_logger().info(f"KARIS data folder: {self.karis_dir}")
        if self.backend != "isaac":
            raise ValueError(f"backend '{self.backend}' is not available: FoundationPose runs only in "
                             "the Isaac helper (backend isaac)")

        trt_support.enable(detectors)                       # .engine / .trt (TensorRT) models too
        models_dir = Path(self.models_dir) if self.models_dir else self.karis_dir / "models"
        path = detectors.choose_model(models_dir, self.model_choice or None)
        self.model = trt_support.load_detector(detectors, path, self.image_size, self.device, self.iou,
                                               fallback_size=640, log=self.get_logger().info)
        classes = vision.load_classes(self.classes_file or None)
        self.isaac = isaac_backend.IsaacClient(self, self.isaac_service, self.isaac_timeout)
        self.ests, self.fpu, self.glctx = isaac_backend.build_isaac_estimators(
            classes, set(self.model.names.values()), self.isaac, self.get_logger())
        if not self.ests:
            raise RuntimeError("No class in classes.yaml matches the model's classes; nothing to track.")
        self.model.warmup()
        if not self.isaac.wait_for_helper(5.0):
            self.get_logger().warn(f"{self.isaac_service} not available yet: start the Isaac helper "
                                   "(ros2 launch karis_isaac_fp isaac_fp.launch.py)")
        self.class_ids = {v: k for k, v in self.model.names.items()}
        self.get_logger().info(
            f"Model {self.model.model_file} ({self.model.kind}), FoundationPose: {self.backend}, "
            f"meshes for: {sorted(self.ests)}; confidence {vision.CONFIDENCE}, "
            f"detector every {tracker.REDETECT_EVERY} frames")

    # ------------------------------------------------------------ inputs
    def _on_info(self, msg):
        self.info_msg = msg
        K = rc.camera_info_K(msg)
        if self.cam_K is None or not np.allclose(K, self.cam_K):
            self.cam_K = K
            self.info_frame = msg.header.frame_id
            self.tracker = self.trk.PoseTracker(self.model, self.ests, self.fpu, self.glctx, K,
                                                max_register=self.max_register or None)
            self.get_logger().info(f"Camera {msg.width}x{msg.height}: fx={K[0, 0]:.1f} "
                                   f"cx={K[0, 2]:.1f} cy={K[1, 2]:.1f}, frame '{msg.header.frame_id}'")

    @staticmethod
    def _ns(msg):
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def _on_color(self, msg):
        self._add(self.colors, self.depths, msg, color_first=True)

    def _on_depth(self, msg):
        self._add(self.depths, self.colors, msg, color_first=False)

    def _add(self, mine, other, msg, color_first):
        t = self._ns(msg)
        mine[t] = msg
        while len(mine) > 6:
            del mine[min(mine)]
        if not other:
            return
        best = min(other, key=lambda s: abs(s - t))
        if abs(best - t) <= self.max_dt * 1e9:
            pair = (msg, other[best]) if color_first else (other[best], msg)
            if self.pending is not None:
                self.stats["dropped"] += 1          # processing is slower than the camera
            self.pending = pair
            mine.pop(t, None)
            other.pop(best, None)

    # ------------------------------------------------------------ main step
    def process_pending(self):
        if self.pending is None or self.tracker is None:
            return False
        cmsg, dmsg = self.pending
        self.pending = None
        try:
            bgr = rc.color_to_bgr(cmsg)
            depth, _ = rc.depth_to_metres(dmsg, self.depth_scale)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
            return False
        if depth.shape != bgr.shape[:2]:
            self.get_logger().error(f"depth {depth.shape} and colour {bgr.shape[:2]} differ: "
                                    "use the depth ALIGNED to colour", throttle_duration_sec=5.0)
            return False
        depth[(depth < self.min_depth) | (depth > self.max_depth)] = 0.0

        if self.reset_req:
            self.tracker.reset()
            self.seen.clear()
            self.reset_req = False
            self.get_logger().info("Reset: detecting and registering every part again")
        if self.rereg_req:
            self.tracker.request_reregister()
            self.rereg_req = False
            self.get_logger().info("Registering every tracked part again")
        if self.isaac is not None:              # the helper gets the compressed frame
            self.isaac.set_frame(*self._compressed(cmsg, dmsg, bgr, depth))
        t_rel = time.perf_counter() - self.t0
        try:
            tm = self.tracker.step(bgr, depth, self.n, t_rel, force_detect=self.detect_req)
        except RuntimeError as e:                # e.g. the Isaac helper did not answer
            self.get_logger().error(f"frame skipped: {e}", throttle_duration_sec=5.0)
            return False
        self.detect_req = False
        now = time.perf_counter()
        if tm["mode"] != "track":
            for trk in self.tracker.tracks.values():
                if trk["missed"] == 0:
                    self.seen[trk["id"]] = now

        frame = self.camera_frame or cmsg.header.frame_id or self.info_frame or "camera"
        stamp = cmsg.header.stamp if self.stamp_source == "camera" else self.get_clock().now().to_msg()
        arr = self._make_array(stamp, frame, tm, now)
        self.last_cam = arr
        self.pub_cam.publish(arr)
        if self.publish_tf:
            self._publish_tf(arr)
        self._publish_markers(arr)
        if self.target_frame:
            base = self._transform_array(arr, self.target_frame)
            if base is not None:
                self.pub_base.publish(base)
        if self.publish_annotated or self.show:
            self._annotate(bgr, tm, stamp, frame)

        self.n += 1
        self._report(tm)
        return True

    def _compressed(self, cmsg, dmsg, bgr, depth_m):
        """Colour + depth (16-bit PNG in mm, as the helper expects) + camera_info."""
        info = self.info_msg
        c = cmsg if isinstance(cmsg, CompressedImage) else \
            rc.bgr_to_compressed(bgr, cmsg.header.stamp, cmsg.header.frame_id, 95)
        if isinstance(dmsg, CompressedImage) and abs(self.depth_scale - 0.001) < 1e-9:
            d = dmsg                                 # already mm: forward as it came
        else:
            mm = np.clip(depth_m * 1000.0 + 0.5, 0, 65535).astype(np.uint16)
            d = rc.depth_to_compressed(mm, dmsg.header.stamp, dmsg.header.frame_id)
        if d.header.stamp != c.header.stamp:     # one stamp per frame for the helper
            d = CompressedImage(header=c.header, format=d.format, data=d.data)
        return c, d, info

    def _make_array(self, stamp, frame, tm, now):
        arr = ObjectPoseArray()
        arr.header.stamp, arr.header.frame_id = stamp, frame
        arr.pipeline_mode = tm["mode"]
        arr.processing_ms = float(tm["frame_ms"])
        for trk in self.tracker.tracks.values():
            e = self.ests[trk["cls_name"]]
            P = trk["pose"]
            o = ObjectPose()
            o.object_id = rc.slug(trk["id"])
            o.class_name = trk["cls_name"]
            o.class_id = int(trk["cls_id"])
            o.confidence = float(trk.get("conf", 0.0) or 0.0)
            o.mode = trk["mode"]
            o.has_orientation = True
            o.pose = rc.matrix_to_pose(P)
            o.box_pose = rc.matrix_to_pose(P @ np.linalg.inv(e["to_origin"]))
            o.box_size = rc.vec3(e["bbox"][1] * 2)
            iou = trk.get("iou")
            o.fit_iou = float(iou) if iou is not None else -1.0
            o.age_s = float(now - self.seen.get(trk["id"], now))
            arr.objects.append(o)
        counters = {}
        for s in self.tracker.slots:          # classes without a mesh: position only
            if s["centre"] is None:
                continue
            counters[s["cls_name"]] = counters.get(s["cls_name"], 0) + 1
            T = np.eye(4)
            T[:3, 3] = s["centre"]
            o = ObjectPose()
            o.object_id = rc.slug(f"{s['cls_name']}_{counters[s['cls_name']]}")
            o.class_name = s["cls_name"]
            o.class_id = int(s["cls_id"])
            o.confidence = float(s.get("conf", 0.0) or 0.0)
            o.mode = "centre"
            o.has_orientation = False
            o.pose = o.box_pose = rc.matrix_to_pose(T)
            o.fit_iou = -1.0
            arr.objects.append(o)
        return arr

    # ------------------------------------------------------------ outputs
    def _publish_tf(self, arr):
        tfs = []
        for o in arr.objects:
            t = TransformStamped()
            t.header = arr.header
            t.child_frame_id = self.tf_prefix + o.object_id
            t.transform = rc.matrix_to_transform(rc.pose_to_matrix(o.pose))
            tfs.append(t)
        if tfs:
            self.tf_pub.sendTransform(tfs)

    def _publish_markers(self, arr):
        ma = MarkerArray()
        clear = Marker()
        clear.header = arr.header
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        for i, o in enumerate(arr.objects):
            b, g, r = self.vision.class_color(o.class_id)
            box = Marker()
            box.header, box.ns, box.id = arr.header, "karis_parts", i
            box.type = Marker.CUBE if o.has_orientation else Marker.SPHERE
            box.action = Marker.ADD
            box.pose = o.box_pose
            if o.has_orientation:
                box.scale = o.box_size
            else:
                box.scale.x = box.scale.y = box.scale.z = 0.015
            box.color.r, box.color.g, box.color.b, box.color.a = r / 255, g / 255, b / 255, 0.6
            txt = Marker()
            txt.header, txt.ns, txt.id = arr.header, "karis_labels", i
            txt.type, txt.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            txt.pose = rc.matrix_to_pose(rc.pose_to_matrix(o.box_pose))   # a copy
            txt.pose.position.y -= 0.03              # camera frame: -y is up
            txt.scale.z = 0.012
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            txt.text = o.object_id
            ma.markers += [box, txt]
        self.pub_markers.publish(ma)

    def _lookup(self, target, source):
        """4x4 target <- source from TF (latest available; the robot and laptop clocks
        may differ, so the image stamp is not used)."""
        try:
            t = self.tf_buffer.lookup_transform(target, source, Time())
            return rc.transform_to_matrix(t.transform), None
        except Exception as e:                     # tf2 lookup / connectivity / extrapolation
            return None, str(e)

    def _transform_array(self, arr, target):
        if target == arr.header.frame_id:
            return arr
        T, err = self._lookup(target, arr.header.frame_id)
        if T is None:
            if time.time() - self.tf_warned > 30:
                self.tf_warned = time.time()
                self.get_logger().warn(
                    f"No TF {target} <- {arr.header.frame_id} yet ({err.splitlines()[0]}). "
                    "/karis/object_poses_base starts when the robot model runs "
                    "(ros2 launch karis_bringup robot_model.launch.py on the motion side).")
            return None
        out = ObjectPoseArray()
        out.header.stamp, out.header.frame_id = arr.header.stamp, target
        out.pipeline_mode, out.processing_ms = arr.pipeline_mode, arr.processing_ms
        for o in arr.objects:
            n = ObjectPose()
            for f in ("object_id", "class_name", "class_id", "confidence", "mode",
                      "has_orientation", "box_size", "fit_iou", "age_s"):
                setattr(n, f, getattr(o, f))
            n.pose = rc.matrix_to_pose(T @ rc.pose_to_matrix(o.pose))
            n.box_pose = rc.matrix_to_pose(T @ rc.pose_to_matrix(o.box_pose))
            out.objects.append(n)
        return out

    def _annotate(self, bgr, tm, stamp, frame):
        vis = self.tracker.draw(bgr, {"frame": self.n, "t": time.perf_counter() - self.t0,
                                      "mode": tm["mode"], "ms": tm["frame_ms"]})
        fps = self._fps()
        import cv2
        cv2.putText(vis, f"ROS {fps:4.1f} fps", (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 255), 2, cv2.LINE_AA)
        if self.publish_annotated:
            self.pub_annot.publish(rc.bgr_to_compressed(vis, stamp, frame, self.annotated_quality))
        if self.show:
            try:
                if not getattr(self, "_window_open", False):
                    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)      # resizable with the mouse too
                show = vis
                if abs(self.show_scale - 1.0) > 1e-3:
                    show = cv2.resize(vis, None, fx=self.show_scale, fy=self.show_scale,
                                      interpolation=cv2.INTER_LINEAR)
                cv2.imshow(WINDOW, show)
                self._window_open = True
                key = cv2.waitKey(1) & 0xFF
            except cv2.error:
                self.show = False
                self.get_logger().warn("Cannot open a window here; use rqt_image_view instead")
                return
            self._on_key(key)

    def _on_key(self, key):
        if key in (ord("r"), ord("R")):
            self.reset_req = True
        elif key in (ord("d"), ord("D")):
            self.detect_req = True
        elif key in (ord("q"), ord("Q"), 27):
            raise KeyboardInterrupt

    def pump_window(self):
        """Keeps the OpenCV window responsive between processed frames (it only redraws,
        moves and takes keys while cv2.waitKey runs; with slow frames, e.g. the photo test at
        2 frames/s, Windows/WSLg would otherwise show it as an empty, frozen window)."""
        if self.show and getattr(self, "_window_open", False):
            import cv2
            try:
                self._on_key(cv2.waitKey(1) & 0xFF)
            except cv2.error:
                pass

    def _fps(self):
        ms = self.stats["ms"][-20:]
        return 1000.0 / np.mean(ms) if ms else 0.0

    def _report(self, tm):
        st = self.stats
        st["frames"] += 1
        st["ms"].append(tm["frame_ms"])
        el = time.perf_counter() - st["t"]
        if el >= 10.0:
            self.get_logger().info(
                f"{st['frames'] / el:.1f} frames/s processed, {st['dropped']} older frames skipped, "
                f"{len(self.tracker.tracks)} parts tracked, last: {tm['mode']} {tm['frame_ms']:.0f} ms")
            st.update(frames=0, dropped=0, t=time.perf_counter(), ms=st["ms"][-50:])

    # ------------------------------------------------------------ services
    def _srv_reset(self, req, res):
        self.reset_req = True
        res.success, res.message = True, "all parts will be detected and registered again"
        return res

    def _srv_detect(self, req, res):
        self.detect_req = True
        res.success, res.message = True, "detector runs on the next frame"
        return res

    def _srv_reregister(self, req, res):
        self.rereg_req = True
        res.success, res.message = True, "every tracked part is registered again on the next frame"
        return res

    def _srv_get_objects(self, req, res):
        if req.fresh:
            self.process_pending()               # newest camera frame, if one is waiting
        arr = self.last_cam
        if arr is None:
            res.success, res.message = False, "no camera frame processed yet"
            return res
        if req.target_frame and req.target_frame != arr.header.frame_id:
            arr = self._transform_array(arr, req.target_frame)
            if arr is None:
                res.success = False
                res.message = f"no TF from {self.last_cam.header.frame_id} to {req.target_frame}"
                return res
        out = ObjectPoseArray()
        out.header, out.pipeline_mode, out.processing_ms = arr.header, arr.pipeline_mode, arr.processing_ms
        want_id = rc.slug(req.object_id) if req.object_id else ""   # "Rund mutter_1" works too
        out.objects = [o for o in arr.objects
                       if (not req.class_name or o.class_name == req.class_name)
                       and (not want_id or o.object_id == want_id)]
        res.objects = out
        res.success = bool(out.objects)
        res.message = f"{len(out.objects)} part(s)" if out.objects else "no matching part in view"
        return res

    def shutdown(self):
        if self.tracker is not None and self.tracker.time_rows:
            self.tracker.print_summary()
        if self.isaac is not None:
            from karis_perception.isaac_backend import timing_summary
            print(timing_summary(self.isaac))
            self.isaac.shutdown()
        shutil.rmtree(self.tmp_dir, ignore_errors=True)
        if self.show:
            import cv2
            cv2.destroyAllWindows()


def main(args=None):
    from rclpy.executors import SingleThreadedExecutor
    rclpy.init(args=args)
    node = PoseEstimatorNode()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    try:
        while rclpy.ok():
            # take every waiting message (only the newest colour/depth pair is kept) ...
            ex.spin_once(timeout_sec=0.0 if node.pending is not None else 0.05)
            for _ in range(30):
                ex.spin_once(timeout_sec=0.0)
            # ... then process it. Single thread: no locking, and the GPU work
            # (detector, FoundationPose) stays on the thread that created it.
            node.process_pending()
            node.pump_window()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
