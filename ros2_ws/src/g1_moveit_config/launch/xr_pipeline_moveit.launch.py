from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import yaml
import xacro


def load_yaml(package_name, relative_path):
    package_share = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_share, relative_path)

    with open(absolute_path, "r") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    g1_description_share = get_package_share_directory("g1_description")
    g1_moveit_share = get_package_share_directory("g1_moveit_config")

    # Use exactly the same robot model as g1_control.
    xacro_file = os.path.join(
        g1_description_share,
        "urdf",
        "g1_29dof_with_inspire_hand_ftp.urdf.xacro",
    )

    robot_description = {
        "robot_description": xacro.process_file(xacro_file).toxml()
    }

    # Semantic robot description (planning groups, etc.).
    srdf_file = os.path.join(
        g1_moveit_share,
        "config",
        "g1.srdf",
    )

    with open(srdf_file, "r") as file:
        robot_description_semantic = {
            "robot_description_semantic": file.read()
        }

    # IK configuration.
    robot_description_kinematics = {
        "robot_description_kinematics":
            load_yaml("g1_moveit_config", "config/kinematics.yaml")
    }

    # Conservative velocity and acceleration limits for MoveIt trajectory generation.
    robot_description_planning = {
        "robot_description_planning": {
            "joint_limits":
                load_yaml("g1_moveit_config", "config/joint_limits.yaml")["joint_limits"]
        }
    }

    # OMPL planning pipeline.
    ompl_planning_pipeline_config = load_yaml(
        "g1_moveit_config",
        "config/ompl_planning.yaml",
    )

    planning_pipeline = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl_planning_pipeline_config,
    }

    # MoveIt -> XR SIM trajectory action mapping; not g1_control.
    moveit_controllers = load_yaml(
        "g1_moveit_config",
        "config/moveit_xr_controllers.yaml",
    )

    # Publish the robot TF tree from /joint_states.
    # This uses the same robot_description as MoveIt so the planning
    # model and published TF frames cannot accidentally diverge.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipeline,
            moveit_controllers,
            {
                "trajectory_execution.allowed_execution_duration_scaling": 1.2,
                "trajectory_execution.allowed_goal_duration_margin": 0.5,
                "trajectory_execution.allowed_start_tolerance": 0.01,
                # Disabled for real G1 read-only observation.
                "allow_trajectory_execution": ParameterValue(
                    LaunchConfiguration("enable_execution"), value_type=bool),
            },
        ],
    )

    rviz = Node(
        condition=IfCondition(LaunchConfiguration("with_rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="screen",
        arguments=["-d", os.path.join(g1_moveit_share, "config", "moveit.rviz")],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("enable_execution", default_value="false"),
        DeclareLaunchArgument("with_rviz", default_value="true"),
        robot_state_publisher,
        move_group,
        rviz,
    ])
