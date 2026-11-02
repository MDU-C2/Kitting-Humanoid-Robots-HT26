"""
perception_scene_updater: puts the parts that the KARIS perception sees into MoveIt's planning
scene, so the arm plans around them. Runs on the MOTION laptop next to move_group.

    ros2 run perception_scene_updater scene_updater
    ros2 run perception_scene_updater scene_updater --ros-args -p add_table:=true -p table_top_z:=-0.06

Uses only standard ROS 2 / MoveIt messages: it does NOT need the perception's own packages
(karis_interfaces). Input and output:

    /karis/markers        visualization_msgs/MarkerArray   from the perception laptop (over WiFi)
                          per part with a full 6D pose: a CUBE (namespace "karis_parts":
                          pose = the part's bounding-box centre and orientation, scale = box size)
                          and a TEXT (namespace "karis_labels", same id, text = object_id).
                          Poses are in the camera frame "head_camera_color_optical_frame".
                          Tray slots and position-only parts are SPHEREs: not added.
    TF                    pelvis -> ... -> d435_link from THIS laptop's robot model
                          (run_g1.sh: robot_state_publisher), d435_link -> camera frame from the
                          perception laptop (static camera mount). Together: pelvis -> camera.
    /collision_object     moveit_msgs/CollisionObject to move_group (ADD = add or move, REMOVE)
                          one box per part, id = object_id, frame = base_frame (pelvis).
    /karis/task/active_object  std_msgs/String (optional input): the part being picked; it is
                          taken out of the scene, otherwise MoveIt refuses to bring the hand
                          close to it. Publish "" to clear.

The same parts are also available as TF frames "karis/<object_id>" (the part's CAD frame) from
the perception, e.g. `ros2 run tf2_ros tf2_echo pelvis karis/ratblock_chamfer_2`.

Parameters:
    base_frame (pelvis)        MoveIt's planning frame; boxes are published in this frame
    rate (2.0 Hz)              max. scene updates per second
    remove_after_s (3.0)       remove a part not seen for this long
    padding (0.005 m)          added to every box side (safety margin)
    classes ([] = all)         only object_ids starting with these prefixes, e.g. ["ratblock_chamfer"]
    add_table (false)          also add the table as a flat box
    table_top_z (-0.06 m)      height of the table top in base_frame (measure it!)
    table_size ([0.8, 1.2, 0.04] m)    x (depth), y (width), z (thickness)
    table_center_xy ([0.55, 0.0] m)    table centre in base_frame

==============================================================================================
PLAN / TODO for the motion team (fill in with your own code or with an AI assistant)
==============================================================================================
Network (see also run_g1.sh: G1_ROS_EXTRA_INTERFACES / G1_ROS_PEERS):
  - This node and move_group must be on the network where the perception laptop is (team
    WiFi), and the robot model (TF) must reach the perception laptop too. Check:
      ros2 topic hz /karis/markers                      (here: the perception arrives)
      ros2 run tf2_ros tf2_echo pelvis head_camera_color_optical_frame   (camera mount arrives)
  - Clocks of both laptops in sync (TF from two computers is combined).

Scene (this node):
  [x] parts with a full pose -> boxes in the planning scene, follow them, remove stale ones
  [x] optional table box (add_table): MEASURE table_top_z / size / centre on the robot; without
      the table MoveIt may plan through the table top
  [ ] tray as an obstacle: the perception gives the tray slots as positions only (SPHEREs);
      a tray box could be built from the slot positions (min/max x, y; fixed height)
  [ ] attach the part to the hand after grasping (moveit_msgs/AttachedCollisionObject on
      /attached_collision_object, link = the wrist/hand link) so it moves with the hand

Picking (not in this node; in the motion team's code, e.g. move_right_arm.cpp):
  [ ] get the target part's pose in pelvis from TF:  lookup_transform("pelvis", "karis/<id>")
      (CAD frame) or from this node's box (bounding-box centre). Choose the part by its id
      (text of the markers) or by class prefix + closest to the robot.
  [ ] before planning to the part: publish its id on /karis/task/active_object
  [ ] grasp target = part pose + the measured wrist offset and hand orientation
      (move_right_arm.cpp: offset (-0.197, -0.036, +0.1127), quaternion (-0.7087, 0.0589,
      0.0797, 0.6985) for the right hand); approach 10 cm above, then down, close hand, lift
  [ ] left arm: mirror y of the offset and the quaternion's x and z
  [ ] rotate the hand with the part's yaw (from the part's TF) for non-round parts
  [ ] place: slot position from /karis/markers (SPHERE in namespace "karis_parts" whose label
      starts with the slot class, e.g. "ratblock_chamfer_hole_48540193_1")
  [ ] use the pose from ONE moment (do not follow the part during the motion)
==============================================================================================
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

from perception_scene_updater.geometry import compose, pair_boxes_with_labels

MARKER_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                        history=HistoryPolicy.KEEP_LAST, depth=5)   # same as the perception
TABLE_ID = "table"


def _pose_tuple(p):
    return ((p.position.x, p.position.y, p.position.z),
            (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w))


def _to_pose(t):
    (x, y, z), (qx, qy, qz, qw) = t
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
    return p


class SceneUpdater(Node):
    def __init__(self):
        super().__init__("perception_scene_updater")
        p = lambda n, v: self.declare_parameter(n, v).value   # noqa: E731
        self.base_frame = str(p("base_frame", "pelvis"))
        self.period = 1.0 / float(p("rate", 2.0))
        self.remove_after = float(p("remove_after_s", 3.0))
        self.padding = float(p("padding", 0.005))
        self.classes = [c for c in p("classes", [""]) if c]
        self.add_table = bool(p("add_table", False))
        self.table_top_z = float(p("table_top_z", -0.06))
        self.table_size = [float(v) for v in p("table_size", [0.8, 1.2, 0.04])]
        self.table_xy = [float(v) for v in p("table_center_xy", [0.55, 0.0])]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(CollisionObject, "/collision_object", 10)
        self.create_subscription(MarkerArray, "/karis/markers", self.on_markers, MARKER_QOS)
        self.create_subscription(String, "/karis/task/active_object", self.on_active, 10)
        self.in_scene = {}       # object_id -> last time seen
        self.active = ""         # the part being picked: never in the scene
        self.last_pub = 0.0
        self.tf_warned = 0.0
        if self.add_table:
            self.create_timer(2.0, self.publish_table)   # re-send now and then (move_group restarts)
        self.get_logger().info(f"/karis/markers -> MoveIt /collision_object in '{self.base_frame}'"
                               + (f", table top at z={self.table_top_z:+.3f}" if self.add_table else ""))

    # ------------------------------------------------------------------ inputs
    def on_active(self, msg):
        self.active = msg.data
        if self.active in self.in_scene:
            self.remove(self.active)

    def on_markers(self, msg):
        now = time.time()
        if now - self.last_pub < self.period:
            return
        self.last_pub = now
        boxes = pair_boxes_with_labels(msg.markers)
        for oid, m in boxes.items():
            if oid == self.active or (self.classes and not any(oid.startswith(c) for c in self.classes)):
                continue
            pose = self.to_base(m.header.frame_id, m.pose)
            if pose is None:
                return                                   # no TF yet; try again next message
            co = CollisionObject()
            co.header.frame_id = self.base_frame
            co.header.stamp = self.get_clock().now().to_msg()
            co.id = oid
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [m.scale.x + self.padding, m.scale.y + self.padding, m.scale.z + self.padding]
            co.primitives = [box]
            co.primitive_poses = [pose]
            co.operation = CollisionObject.ADD          # ADD again = move it
            self.pub.publish(co)
            self.in_scene[oid] = now
        for oid, t in list(self.in_scene.items()):
            if now - t > self.remove_after:
                self.remove(oid)

    # ------------------------------------------------------------------ helpers
    def to_base(self, frame, pose):
        """Pose in `frame` -> Pose in base_frame, using the latest TF (both laptops' clocks may
        differ, so the marker stamp is not used)."""
        if not frame or frame == self.base_frame:
            return pose
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, frame, Time())
        except Exception as e:                           # no TF yet / not connected
            if time.time() - self.tf_warned > 10:
                self.tf_warned = time.time()
                self.get_logger().warn(f"no TF {self.base_frame} <- {frame}: {str(e).splitlines()[0]} "
                                       "(robot model running? camera mount from the perception arriving?)")
            return None
        tr, rq = t.transform.translation, t.transform.rotation
        t_ab = ((tr.x, tr.y, tr.z), (rq.x, rq.y, rq.z, rq.w))
        return _to_pose(compose(t_ab, _pose_tuple(pose)))

    def remove(self, oid):
        co = CollisionObject()
        co.header.frame_id = self.base_frame
        co.id = oid
        co.operation = CollisionObject.REMOVE
        self.pub.publish(co)
        self.in_scene.pop(oid, None)

    def publish_table(self):
        co = CollisionObject()
        co.header.frame_id = self.base_frame
        co.header.stamp = self.get_clock().now().to_msg()
        co.id = TABLE_ID
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(self.table_size)
        co.primitives = [box]
        co.primitive_poses = [_to_pose(((self.table_xy[0], self.table_xy[1],
                                         self.table_top_z - self.table_size[2] / 2), (0.0, 0.0, 0.0, 1.0)))]
        co.operation = CollisionObject.ADD
        self.pub.publish(co)


def main():
    rclpy.init()
    node = SceneUpdater()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for oid in list(node.in_scene):
            node.remove(oid)                             # leave a clean scene
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
