"""
Prints what the perception sees: checks the ROS 2 link vision -> motion.

    ros2 run karis_task pose_listener                       (robot frame, /karis/object_poses_base)
    ros2 run karis_task pose_listener --ros-args -p topic:=/karis/object_poses   (camera frame)
"""
import math
import time

import rclpy
from rclpy.node import Node
from karis_interfaces.msg import ObjectPoseArray


def rpy_deg(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [math.degrees(a) for a in (roll, pitch, yaw)]


class PoseListener(Node):
    def __init__(self):
        super().__init__("karis_pose_listener")
        topic = self.declare_parameter("topic", "/karis/object_poses_base").value
        self.period = float(self.declare_parameter("print_every_s", 1.0).value)
        self.last_print = 0.0
        self.n = 0
        self.create_subscription(ObjectPoseArray, topic, self.on_poses, 10)
        self.get_logger().info(f"Listening on {topic}")

    def on_poses(self, msg):
        self.n += 1
        if time.time() - self.last_print < self.period:
            return
        self.last_print = time.time()
        lines = [f"{len(msg.objects)} part(s) in '{msg.header.frame_id}', {msg.pipeline_mode}, "
                 f"{msg.processing_ms:.0f} ms ({self.n} msgs since last print)"]
        for o in msg.objects:
            p = o.pose.position
            r = rpy_deg(o.pose.orientation) if o.has_orientation else None
            rot = f"rpy {r[0]:+5.0f} {r[1]:+5.0f} {r[2]:+5.0f} deg" if r else "(position only)"
            lines.append(f"  {o.object_id:<34s} xyz {p.x:+.3f} {p.y:+.3f} {p.z:+.3f} m  {rot}  "
                         f"{o.mode}, conf {o.confidence:.2f}")
        self.get_logger().info("\n".join(lines))
        self.n = 0


def main(args=None):
    rclpy.init(args=args)
    node = PoseListener()
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
