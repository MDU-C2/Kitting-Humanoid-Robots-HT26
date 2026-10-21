"""Bridge between /karis/fp/estimate_pose and the Isaac ROS FoundationPose nodes.

Each request is published to /karis_fp/<class>/pose_estimation/* (register) or
/karis_fp/<class>/tracking/* (track) with a unique stamp (frame stamp + k ms), and the
outputs with that stamp are returned. Tracking state is kept per track_id, and frames are
cached by stamp so the client sends the images only once per frame. Isaac's poses refer to
the centred mesh and are converted to the CAD frame with center_m from meshes.yaml.
"""
import array
import copy
import threading
import time
from collections import OrderedDict

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection3DArray
from isaac_ros_tensor_list_interfaces.msg import TensorList

from karis_interfaces.srv import EstimatePose

PUB_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
SUB_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
PNG_MAGIC = b"\x89PNG"
STAMP_STEP_NS = 1_000_000   # request stamp = frame stamp + k ms, k < STAMP_SEQ_N
STAMP_SEQ_N = 50
STAMP_TOL_NS = 400_000      # output/request stamp match tolerance


def quat_to_rot(x, y, z, w):
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n if n > 0 else 0.0
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)]])


def stamp_ns(st):
    return st.sec * 1_000_000_000 + st.nanosec


def ns_to_stamp(ns, like):
    st = copy.copy(like)
    st.sec, st.nanosec = int(ns // 1_000_000_000), int(ns % 1_000_000_000)
    return st


class Waiter:
    """Collects the Detection3DArray and pose TensorList of one request."""

    def __init__(self, stamp):
        self.stamp = stamp
        self.det = None
        self.mat = None
        self.event = threading.Event()

    def offer(self, det=None, mat=None):
        if det is not None:
            self.det = det
        if mat is not None:
            self.mat = mat
        if self.det is not None and self.mat is not None:
            self.event.set()


class ClassPipe:
    """Isaac ROS FoundationPose topics of one part class."""

    def __init__(self, node, name, slug, center, cb):
        self.name, self.slug = name, slug
        self.center = np.asarray(center, dtype=np.float64)   # mesh centre in the CAD frame
        self.lock = threading.Lock()          # one request at a time per class
        self.waiter = None
        self.seq = 0
        self.log = node.get_logger()
        self.seen = {"det": 0, "mat": 0}       # outputs received (any stamp)
        self.unmatched = 0
        ns = f"/karis_fp/{slug}"
        pub = lambda t, n: node.create_publisher(t, f"{ns}/{n}", PUB_QOS)   # noqa: E731
        self.est_pub = {"image": pub(Image, "pose_estimation/image"),
                        "depth": pub(Image, "pose_estimation/depth_image"),
                        "info": pub(CameraInfo, "pose_estimation/camera_info"),
                        "mask": pub(Image, "pose_estimation/segmentation")}
        self.trk_pub = {"image": pub(Image, "tracking/image"),
                        "depth": pub(Image, "tracking/depth_image"),
                        "info": pub(CameraInfo, "tracking/camera_info"),
                        "pose": pub(TensorList, "tracking/pose_input")}
        for kind in ("pose_estimation", "tracking"):
            node.create_subscription(Detection3DArray, f"{ns}/{kind}/output",
                                     lambda m: self._on(det=m), SUB_QOS, callback_group=cb)
            node.create_subscription(TensorList, f"{ns}/{kind}/pose_matrix_output",
                                     lambda m: self._on(mat=m), SUB_QOS, callback_group=cb)

    def _on(self, det=None, mat=None):
        kind = "det" if det is not None else "mat"
        self.seen[kind] += 1
        if self.seen[kind] == 1:
            self.log.info(f"{self.name}: first {'pose' if kind == 'det' else 'pose matrix'} "
                          "output from Isaac FoundationPose")
        w = self.waiter
        msg = det if det is not None else mat
        if w is None:
            return
        # outputs carry the request stamp; others are late answers to timed-out requests
        diff = stamp_ns(msg.header.stamp) - w.stamp
        if abs(diff) <= STAMP_TOL_NS:
            w.offer(det=det, mat=mat)
        else:
            self.unmatched += 1
            self.log.warn(f"{self.name}: Isaac output stamped {diff / 1e6:+.3f} ms from the "
                          "waiting request: dropped (late answer to an earlier request?)",
                          throttle_duration_sec=2.0)

    def diagnose(self, mode):
        """Explanation for a timeout."""
        pubs = self.est_pub if mode == "register" else self.trk_pub
        subs = ", ".join(f"{k} {p.get_subscription_count()}" for k, p in pubs.items())
        txt = (f"Isaac subscribers on the {mode} inputs: {subs}; outputs received for this class "
               f"so far: {self.seen['det']} poses, {self.seen['mat']} matrices"
               + (f" ({self.unmatched} with a non-matching stamp)" if self.unmatched else ""))
        if any(p.get_subscription_count() == 0 for p in pubs.values()):
            txt += ". An input has no subscriber: are the Isaac nodes of this class running?"
        elif not (self.seen["det"] or self.seen["mat"]):
            txt += (". Isaac gets the inputs but answers nothing: look for errors in the "
                    "isaac_fp launch output (TensorRT engine, GPU memory)")
        return txt


class FpBridge(Node):
    def __init__(self):
        super().__init__("karis_fp_bridge")
        p = lambda n, v: self.declare_parameter(n, v).value   # noqa: E731
        meshes_yaml = p("meshes_yaml", "")
        only = [c.strip() for c in p("classes", "").split(",") if c.strip()]
        self.reg_timeout = float(p("register_timeout_s", 10.0))
        self.trk_timeout = float(p("track_timeout_s", 2.0))
        self.cache_n = int(p("cache_frames", 4))
        with open(meshes_yaml) as f:
            meshes = yaml.safe_load(f)["classes"]
        cb = ReentrantCallbackGroup()
        self.pipes = {}
        for name, m in meshes.items():
            if only and name not in only and m["slug"] not in only:
                continue
            if "center_m" not in m:
                raise RuntimeError(f"{meshes_yaml} is from an older make_isaac_meshes: run it again")
            self.pipes[name] = ClassPipe(self, name, m["slug"], m["center_m"], cb)
        self.states = OrderedDict()            # track_id -> last pose TensorList
        self.max_states = 500
        self.frames = OrderedDict()            # frame stamp ns -> (rgb Image, depth Image)
        self.frames_lock = threading.Lock()
        self.create_service(EstimatePose, "/karis/fp/estimate_pose", self._on_estimate_pose, callback_group=cb)
        self.get_logger().info(f"Isaac ROS FoundationPose bridge for {sorted(self.pipes)} on "
                               f"/karis/fp/estimate_pose")

    # ------------------------------------------------------------ frames
    def _frame(self, req):
        key = stamp_ns(req.color.header.stamp)
        with self.frames_lock:
            if key in self.frames and not len(req.color.data):
                return self.frames[key]
        if not len(req.color.data) or not len(req.depth.data):
            return None
        bgr = cv2.imdecode(np.frombuffer(bytes(req.color.data), np.uint8), cv2.IMREAD_COLOR)
        d = bytes(req.depth.data)
        raw = cv2.imdecode(np.frombuffer(d[max(d.find(PNG_MAGIC), 0):], np.uint8), cv2.IMREAD_UNCHANGED)
        if bgr is None or raw is None:
            raise ValueError("cannot decode the colour or depth image")
        depth = raw.astype(np.float32) * 0.001 if raw.dtype == np.uint16 else raw.astype(np.float32)  # mm -> m
        rgb = self._image(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), "rgb8", req.color.header)
        dep = self._image(depth, "32FC1", req.color.header)
        with self.frames_lock:
            self.frames[key] = (rgb, dep)
            while len(self.frames) > self.cache_n:
                self.frames.popitem(last=False)
        return rgb, dep

    @staticmethod
    def _image(arr, encoding, header):
        m = Image()
        m.header = copy.copy(header)
        m.height, m.width = arr.shape[:2]
        m.encoding = encoding
        m.is_bigendian = 0
        m.step = int(arr.strides[0])
        m.data = array.array("B", np.ascontiguousarray(arr).tobytes())
        return m

    @staticmethod
    def _restamp(msg, st):
        """Shallow copy with a new stamp."""
        m = copy.copy(msg)
        m.header = copy.copy(msg.header)
        m.header.stamp = st
        return m

    # ------------------------------------------------------------ service
    def _on_estimate_pose(self, req, res):
        t0 = time.perf_counter()
        res.success = False
        if req.mode == "forget":
            self.states.pop(req.track_id, None)
            res.success, res.message = True, "forgotten"
            return res
        pipe = self.pipes.get(req.class_name)
        if pipe is None:
            res.message = f"unknown class '{req.class_name}' (bridge has {sorted(self.pipes)})"
            return res
        try:
            frame = self._frame(req)
        except ValueError as e:
            res.message = str(e)
            return res
        if frame is None:
            res.message = "frame not cached"
            return res
        rgb, dep = frame
        if req.mode == "track" and req.track_id not in self.states:
            res.message = "no tracking state for this track_id: register first"
            return res
        if req.mode not in ("register", "track"):
            res.message = f"unknown mode '{req.mode}'"
            return res

        with pipe.lock:
            pipe.seq = (pipe.seq + 1) % STAMP_SEQ_N
            ns = stamp_ns(req.color.header.stamp) + pipe.seq * STAMP_STEP_NS
            st = ns_to_stamp(ns, req.color.header.stamp)
            w = pipe.waiter = Waiter(ns)
            info = self._restamp(req.camera_info, st)
            info.header.frame_id = req.camera_info.header.frame_id or rgb.header.frame_id
            if req.mode == "register":
                mask = self._mask(req, rgb, st)
                if mask is None:
                    pipe.waiter = None
                    res.message = "register needs a mask (PNG mono8, same size as the image)"
                    return res
                P = pipe.est_pub
                P["info"].publish(info)
                P["mask"].publish(mask)
                P["depth"].publish(self._restamp(dep, st))
                P["image"].publish(self._restamp(rgb, st))
                timeout = self.reg_timeout
            else:
                P = pipe.trk_pub
                pose_in = copy.copy(self.states[req.track_id])
                pose_in.header = copy.copy(info.header)
                P["pose"].publish(pose_in)
                P["info"].publish(info)
                P["depth"].publish(self._restamp(dep, st))
                P["image"].publish(self._restamp(rgb, st))
                timeout = self.trk_timeout
            ok = w.event.wait(timeout)
            pipe.waiter = None

        if not ok:
            res.message = f"timeout after {timeout:.1f} s ({req.mode}). {pipe.diagnose(req.mode)}"
            self.get_logger().warn(f"{req.class_name}: {res.message}")
            return res
        if not w.det.detections:
            res.message = "Isaac FoundationPose returned no pose"
            return res
        d = w.det.detections[0]
        pose = d.results[0].pose.pose if d.results else d.bbox.center
        T = np.eye(4)
        o = pose.orientation
        T[:3, :3] = quat_to_rot(o.x, o.y, o.z, o.w)
        T[:3, 3] = (pose.position.x, pose.position.y, pose.position.z)
        T[:3, 3] -= T[:3, :3] @ pipe.center         # centred mesh -> CAD frame
        self.states[req.track_id] = w.mat
        self.states.move_to_end(req.track_id)
        while len(self.states) > self.max_states:
            self.states.popitem(last=False)
        res.pose = [float(v) for v in T.reshape(-1)]
        res.success, res.message = True, req.mode
        res.processing_ms = float((time.perf_counter() - t0) * 1000)
        return res

    def _mask(self, req, rgb, st):
        if not len(req.mask.data):
            return None
        m = cv2.imdecode(np.frombuffer(bytes(req.mask.data), np.uint8), cv2.IMREAD_GRAYSCALE)
        if m is None or m.shape != (rgb.height, rgb.width):
            return None
        hdr = copy.copy(rgb.header)
        hdr.stamp = st
        return self._image(((m > 0) * 255).astype(np.uint8), "mono8", hdr)


def main(args=None):
    rclpy.init(args=args)
    node = FpBridge()
    ex = MultiThreadedExecutor(num_threads=8)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
