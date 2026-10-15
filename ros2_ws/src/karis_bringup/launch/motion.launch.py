"""Motion side (Humble container): robot model + vision -> motion nodes.

    ros2 launch karis_bringup motion.launch.py                     # real G1
    ros2 launch karis_bringup motion.launch.py fake_joints:=true rviz:=true

Starts robot_model.launch.py, the MoveIt scene updater, the pose listener and the
/karis/pick action server. MoveIt's move_group itself comes from the G1 MoveIt config
(not in this repo yet); until then use dry_run goals.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("karis_bringup")
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("fake_joints", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("base_frame", default_value="pelvis"),
        DeclareLaunchArgument("planning_group", default_value="right_arm"),
        DeclareLaunchArgument("ee_link", default_value="right_wrist_yaw_link"),
        DeclareLaunchArgument("execute", default_value="false",
                              description="true = MoveIt also executes (moves the real arm!)"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "robot_model.launch.py")),
            launch_arguments={"fake_joints": arg("fake_joints"), "rviz": arg("rviz")}.items()),
        Node(package="karis_task", executable="scene_updater", output="screen"),
        Node(package="karis_task", executable="pose_listener", output="screen",
             parameters=[{"print_every_s": 5.0}]),
        Node(package="karis_task", executable="pick_server", output="screen",
             parameters=[{"base_frame": arg("base_frame"), "planning_group": arg("planning_group"),
                          "ee_link": arg("ee_link"), "execute": arg("execute")}]),
    ])
