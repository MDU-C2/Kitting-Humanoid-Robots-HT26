"""G1 robot model + TF on the motion side (Humble container):

    ros2 launch karis_bringup robot_model.launch.py               # joints from the real G1 (/lowstate)
    ros2 launch karis_bringup robot_model.launch.py fake_joints:=true rviz:=true   # no robot

TF tree: pelvis -> ... -> torso_link -> d435_link (URDF) -> head_camera_color_optical_frame
(static, below) -> karis/<object_id> (from the perception). So every part's pose is
available in pelvis / torso_link, which is what MoveIt plans in.

The camera mount is nominal (CAD); calibrate it (hand-eye) and put the result in
camera_xyz / camera_rpy.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    g1 = get_package_share_directory("g1_description")
    share = get_package_share_directory("karis_bringup")
    with open(os.path.join(g1, "urdf", "g1_29dof_with_inspire_hand_ftp.urdf")) as f:
        urdf = f.read()
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("fake_joints", default_value="false",
                              description="true = all joints at 0 (no robot needed)"),
        DeclareLaunchArgument("rviz", default_value="false"),
        # colour optical frame relative to the URDF's d435_link (x forward, z up):
        # the colour imager is ~15 mm to the left of the depth origin; optical = z forward, x right
        DeclareLaunchArgument("camera_x", default_value="0.0"),
        DeclareLaunchArgument("camera_y", default_value="0.015"),
        DeclareLaunchArgument("camera_z", default_value="0.0"),
        DeclareLaunchArgument("camera_yaw", default_value="-1.5707963"),
        DeclareLaunchArgument("camera_pitch", default_value="0.0"),
        DeclareLaunchArgument("camera_roll", default_value="-1.5707963"),

        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf}], output="screen"),
        Node(package="karis_task", executable="g1_state_bridge", output="screen",
             condition=UnlessCondition(arg("fake_joints"))),
        Node(package="joint_state_publisher", executable="joint_state_publisher", output="screen",
             condition=IfCondition(arg("fake_joints"))),
        Node(package="tf2_ros", executable="static_transform_publisher", name="head_camera_mount",
             arguments=[arg("camera_x"), arg("camera_y"), arg("camera_z"), arg("camera_yaw"),
                        arg("camera_pitch"), arg("camera_roll"),
                        "d435_link", "head_camera_color_optical_frame"]),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", os.path.join(share, "rviz", "karis.rviz")],
             condition=IfCondition(arg("rviz"))),
    ])
