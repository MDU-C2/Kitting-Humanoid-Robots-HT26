"""
G1 joint states for the robot model: /lowstate (unitree_hg/LowState, published by
the G1 itself over DDS) -> /joint_states (sensor_msgs/JointState, 29 body joints).
robot_state_publisher then gives the full TF tree pelvis -> ... -> torso_link ->
d435_link -> camera, so object poses can be expressed in the robot's own frames.

Motor index i in LowState is joint i below (Unitree's G1 29-DoF order, the same
order as the joints in g1_description's URDF). The Inspire hand joints are not in
/lowstate; they are published at 0 (read from the URDF) so robot_state_publisher and
MoveIt get a complete robot state.

    ros2 run karis_task g1_state_bridge          (part of robot_model.launch.py)
Parameters: rate (50.0 Hz), lowstate_topic (/lowstate).
"""
import re

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState

G1_29DOF_JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
    "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint",
    "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]


class G1StateBridge(Node):
    def __init__(self):
        super().__init__("g1_state_bridge")
        try:
            from unitree_hg.msg import LowState
        except ImportError as e:
            raise ImportError("unitree_hg messages are missing: build ros2_ws/src/unitree_hg "
                              "(colcon build --packages-select unitree_hg)") from e
        rate = float(self.declare_parameter("rate", 50.0).value)
        topic = self.declare_parameter("lowstate_topic", "/lowstate").value
        self.min_dt = 1.0 / rate
        self.last = None
        self.n = 0
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.extra = self._other_urdf_joints()
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(LowState, topic, self.on_lowstate, qos)
        self.create_timer(5.0, self.check)
        self.get_logger().info(f"{topic} -> /joint_states at up to {rate:.0f} Hz")

    def on_lowstate(self, msg):
        now = self.get_clock().now()
        if self.last is not None and (now - self.last).nanoseconds * 1e-9 < self.min_dt:
            return
        self.last = now
        js = JointState()
        js.header.stamp = now.to_msg()       # laptop clock (the robot's clock may differ)
        n = len(G1_29DOF_JOINTS)
        ms = msg.motor_state
        js.name = G1_29DOF_JOINTS + self.extra
        js.position = [float(ms[i].q) for i in range(n)] + [0.0] * len(self.extra)
        js.velocity = [float(ms[i].dq) for i in range(n)] + [0.0] * len(self.extra)
        js.effort = [float(ms[i].tau_est) for i in range(n)] + [0.0] * len(self.extra)
        self.pub.publish(js)
        self.n += 1

    def _other_urdf_joints(self):
        """Moving joints of the URDF that /lowstate does not have (the Inspire fingers)."""
        try:
            from ament_index_python.packages import get_package_share_directory
            import os
            path = os.path.join(get_package_share_directory("g1_description"), "urdf",
                                "g1_29dof_with_inspire_hand_ftp.urdf")
            with open(path) as f:
                urdf = f.read()
        except Exception as e:                      # g1_description not built: body joints only
            self.get_logger().warn(f"cannot read the G1 URDF ({e}); publishing the 29 body joints only")
            return []
        blocks = re.findall(r'<joint\s+name="([^"]+)"\s+type="(revolute|continuous|prismatic)"(.*?)</joint>',
                            urdf, flags=re.S)
        # mimic joints follow their parent joint in robot_state_publisher: leave them out
        extra = [j for j, _, body in blocks if j not in G1_29DOF_JOINTS and "<mimic" not in body]
        self.get_logger().info(f"{len(extra)} hand joints not in /lowstate are published at 0")
        return extra

    def check(self):
        if self.n == 0:
            self.get_logger().warn("No /lowstate from the G1 yet. Same network, ROS_DOMAIN_ID=0 "
                                   "and RMW_IMPLEMENTATION=rmw_cyclonedds_cpp?",
                                   throttle_duration_sec=30.0)
        self.n = 0


def main(args=None):
    rclpy.init(args=args)
    node = G1StateBridge()
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
