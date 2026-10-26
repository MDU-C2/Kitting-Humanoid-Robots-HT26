#!/usr/bin/env python3
"""G1 robot model on the robot itself: TF pelvis -> waist -> torso_link -> d435_link from the real
waist joint angles. Runs on the G1 PC2 (ROS 2 Foxy), started by `start_head_camera.sh --tf`.
Gives the perception laptop the pelvis frame over WiFi without the motion laptop.

READ ONLY: the waist angles come from g1_waist_reader.py (Unitree's Python SDK, rt/lowstate on
the robot's internal network eth0) through a pipe on stdin; this node only publishes TF.
Nothing is commanded.

    python g1_waist_reader.py | python3 g1_tf_ros2.py      (start_head_camera.sh --tf does this)

    TF (dynamic, publish_rate Hz):  pelvis -> waist_yaw_link -> waist_roll_link -> torso_link
    TF (static):                    torso_link -> d435_link
Same frame names and values as the URDF (g1_29dof_with_inspire_hand_ftp.urdf), so the
perception's camera mount (d435_link -> head_camera_color_optical_frame) attaches to it.
Only the waist is published: that is all the camera needs (the arms are not on the way).

Do NOT run it while the motion laptop runs its robot model (run_g1.sh or observe_g1_tf.sh):
TF would get two sources for the same frames.

Parameters: publish_rate (50.0 Hz).
"""
import math
import sys
import threading

# URDF values (g1_29dof_with_inspire_hand_ftp.urdf)
WAIST_ROLL_XYZ = (-0.0039635, 0.0, 0.044)               # waist_yaw_link -> waist_roll_link
D435_XYZ = (0.0576235, 0.01753, 0.42987)                # torso_link -> d435_link
D435_PITCH = 0.8307767239493009


def axis_quat(axis, angle):
    """Quaternion (x, y, z, w) for a rotation about the x, y or z axis."""
    s, c = math.sin(angle / 2.0), math.cos(angle / 2.0)
    return {"x": (s, 0.0, 0.0, c), "y": (0.0, s, 0.0, c), "z": (0.0, 0.0, s, c)}[axis]


def waist_chain(yaw, roll, pitch):
    """[(parent, child, xyz, quat)] for the moving waist joints."""
    return [("pelvis", "waist_yaw_link", (0.0, 0.0, 0.0), axis_quat("z", yaw)),
            ("waist_yaw_link", "waist_roll_link", WAIST_ROLL_XYZ, axis_quat("x", roll)),
            ("waist_roll_link", "torso_link", (0.0, 0.0, 0.0), axis_quat("y", pitch))]


def main():
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

    def tf(stamp, parent, child, xyz, q):
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, parent, child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = q
        return t

    class G1Tf(Node):
        def __init__(self):
            super().__init__("g1_tf")
            self.declare_parameter("publish_rate", 50.0)
            rate = float(self.get_parameter("publish_rate").value)
            self.q = None                      # (yaw, roll, pitch) from the newest lowstate
            self.n_msgs = 0
            self.tfb = TransformBroadcaster(self)
            self.static = StaticTransformBroadcaster(self)
            self.static.sendTransform(tf(self.get_clock().now().to_msg(), "torso_link", "d435_link",
                                         D435_XYZ, axis_quat("y", D435_PITCH)))
            threading.Thread(target=self.read_stdin, daemon=True).start()
            self.create_timer(1.0 / rate, self.publish)
            self.create_timer(5.0, self.check)
            self.get_logger().info(f"waist angles (stdin) -> TF pelvis -> torso_link -> d435_link at {rate:.0f} Hz")

        def read_stdin(self):
            for line in sys.stdin:
                try:
                    q = tuple(float(v) for v in line.split()[:3])
                except ValueError:
                    continue
                if len(q) == 3:
                    self.q = q
                    self.n_msgs += 1
            self.get_logger().error("The waist reader stopped (end of stdin): no new TF")

        def publish(self):
            if self.q is None:
                return                         # no joint data yet: publish nothing rather than guess
            stamp = self.get_clock().now().to_msg()
            self.tfb.sendTransform([tf(stamp, *link) for link in waist_chain(*self.q)])

        def check(self):
            if self.n_msgs == 0:
                self.get_logger().warn("No waist angles yet (rt/lowstate on eth0): is the robot fully on? "
                                       "(no TF is published until they arrive)")
            else:
                y, r, p = (math.degrees(v) for v in self.q)
                self.get_logger().info(f"lowstate {self.n_msgs / 5.0:.0f} Hz, waist yaw/roll/pitch "
                                       f"{y:+.1f}/{r:+.1f}/{p:+.1f} deg", throttle_duration_sec=60.0)
            self.n_msgs = 0

    rclpy.init()
    node = G1Tf()
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
