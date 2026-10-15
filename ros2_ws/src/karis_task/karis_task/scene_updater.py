"""
Puts the parts the perception sees into MoveIt's planning scene as boxes, so the
arm plans around them (and the target part is known to MoveIt by its object_id).

    ros2 run karis_task scene_updater
Subscribes /karis/object_poses_base (poses in the robot frame) and publishes
moveit_msgs/CollisionObject on /collision_object (move_group listens there).
Parts not seen for `remove_after_s` are removed again. Tray slots (no orientation)
are not added. The part being picked (std_msgs/String on /karis/task/active_object,
from pick_server) is taken out of the scene, otherwise MoveIt would refuse to put the
hand on it.

Parameters: rate (2.0 Hz), remove_after_s (3.0), padding (0.005 m added to each box side),
classes ([] = all classes with a pose).
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from karis_interfaces.msg import ObjectPoseArray
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive


class SceneUpdater(Node):
    def __init__(self):
        super().__init__("karis_scene_updater")
        self.period = 1.0 / float(self.declare_parameter("rate", 2.0).value)
        self.remove_after = float(self.declare_parameter("remove_after_s", 3.0).value)
        self.padding = float(self.declare_parameter("padding", 0.005).value)
        self.classes = list(self.declare_parameter("classes", [""]).value)
        self.classes = [c for c in self.classes if c]
        self.pub = self.create_publisher(CollisionObject, "/collision_object", 10)
        self.create_subscription(ObjectPoseArray, "/karis/object_poses_base", self.on_poses, 10)
        self.in_scene = {}     # object_id -> last time seen
        self.active = ""       # the part being picked: never in the scene
        self.create_subscription(String, "/karis/task/active_object", self.on_active, 10)
        self.last_pub = 0.0
        self.get_logger().info("/karis/object_poses_base -> MoveIt /collision_object")

    def on_active(self, msg):
        self.active = msg.data
        if self.active in self.in_scene:
            self._remove(self.active)

    def _remove(self, oid, header=None):
        co = CollisionObject()
        if header is not None:
            co.header = header
        co.id = oid
        co.operation = CollisionObject.REMOVE
        self.pub.publish(co)
        self.in_scene.pop(oid, None)

    def on_poses(self, msg):
        now = time.time()
        if now - self.last_pub < self.period:
            return
        self.last_pub = now
        for o in msg.objects:
            if not o.has_orientation or (self.classes and o.class_name not in self.classes) \
                    or o.object_id == self.active:
                continue
            co = CollisionObject()
            co.header = msg.header
            co.id = o.object_id
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [o.box_size.x + self.padding, o.box_size.y + self.padding,
                              o.box_size.z + self.padding]
            co.primitives = [box]
            co.primitive_poses = [o.box_pose]
            co.operation = CollisionObject.ADD        # ADD again = move it
            self.pub.publish(co)
            self.in_scene[o.object_id] = now
        for oid, t in list(self.in_scene.items()):
            if now - t > self.remove_after:
                self._remove(oid, msg.header)


def main(args=None):
    rclpy.init(args=args)
    node = SceneUpdater()
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
