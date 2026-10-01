from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import xacro

def generate_launch_description():
    g1_description_share = get_package_share_directory("g1_description")
    g1_control_share = get_package_share_directory("g1_control")

    xacro_file = os.path.join(
        g1_description_share,
        "urdf",
        "g1_29dof_with_inspire_hand_ftp.urdf.xacro",
    )

    controllers_file = os.path.join(
        g1_control_share,
        "config",
        "controllers.yaml",
    )

    robot_description_config = xacro.process_file(xacro_file)

    robot_description = {
        "robot_description": robot_description_config.toxml()
    }

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controllers_file,
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    arm_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
            "--inactive",
        ],
        output="screen",
    )

    return LaunchDescription([
        control_node,
        joint_state_broadcaster_spawner,
        arm_trajectory_controller_spawner,
    ])
