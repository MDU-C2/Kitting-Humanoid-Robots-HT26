import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('karis_calibration'),
        'config',
        'apriltag.yaml'
    )

    apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag_detector",
        output="screen",
        parameters=[str(config_file)],
        remappings=[
            ("image_rect", "/g1/head_camera/color/image_raw"),
            ("camera_info", "/g1/head_camera/color/camera_info"),
        ],
    )

    return LaunchDescription([apriltag_node])
