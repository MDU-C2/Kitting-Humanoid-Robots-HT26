"""KARIS vision -> motion nodes, run in the team's MoveIt container (ros2_ws/scripts/start.sh):

    ros2 launch karis_bringup motion.launch.py                     # plan only
    ros2 launch karis_bringup motion.launch.py execute:=true       # MoveIt also executes

Starts the MoveIt scene updater, the pose listener and the /karis/pick action server.
The robot model (robot_state_publisher, /joint_states), move_group and the arm controller
come from the team's MoveIt stack (ros2_ws/scripts/run_g1.sh -> g1_moveit_config,
g1_xr_bridge); this launch does not start them. Without move_group, use dry_run goals.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("base_frame", default_value="pelvis"),
        DeclareLaunchArgument("planning_group", default_value="right_arm"),
        DeclareLaunchArgument("ee_link", default_value="right_wrist_yaw_link"),
        DeclareLaunchArgument("execute", default_value="false",
                              description="true = MoveIt also executes (moves the real arm!)"),
        Node(package="karis_task", executable="scene_updater", output="screen"),
        Node(package="karis_task", executable="pose_listener", output="screen",
             parameters=[{"print_every_s": 5.0}]),
        Node(package="karis_task", executable="pick_server", output="screen",
             parameters=[{"base_frame": arg("base_frame"), "planning_group": arg("planning_group"),
                          "ee_link": arg("ee_link"), "execute": arg("execute")}]),
    ])
