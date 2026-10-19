"""KARIS perception (perception container, ROS 2 Humble):

    ros2 launch karis_bringup perception.launch.py                         # G1 head camera
    ros2 launch karis_bringup perception.launch.py model:=best8s.pt show:=true
    ros2 launch karis_bringup perception.launch.py confidence:=0.6         # detector threshold
    ros2 launch karis_bringup perception.launch.py redetect_every:=5        # detector every 5 frames
    ros2 launch karis_bringup perception.launch.py image_size:=1280        # YOLO size (default: as trained)
    ros2 launch karis_bringup perception.launch.py player:=/home/kimsv/Kitting-Humanoid-Robots-HT26/local_data/videos/x.bag   # no robot
    ros2 launch karis_bringup perception.launch.py player:=/home/kimsv/Kitting-Humanoid-Robots-HT26/local_data/data           # the test photos

Camera mount: also publishes the static TF d435_link -> head_camera_color_optical_frame
(camera_mount:=true, the default). The robot model (robot_state_publisher in the team's MoveIt
stack, or karis_bringup/scripts/observe_g1_tf.sh) publishes pelvis -> ... -> d435_link, so
together TF has pelvis -> camera and the node publishes /karis/object_poses_base. The values
are the nominal colour-imager position relative to the URDF's d435_link (x forward, z up;
optical frame = z forward, x right); correct them with camera_x/y/z/yaw/pitch/roll (m, rad).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BOOL = {"true": True, "false": False}


def _nodes(context):
    get = lambda n: LaunchConfiguration(n).perform(context)   # noqa: E731
    overrides = {}
    for name in ("model", "target_frame", "karis_dir", "backend"):
        if get(name):
            overrides[name] = get(name)
    if get("show"):
        overrides["show"] = BOOL.get(get("show").lower(), False)
    if get("confidence"):
        overrides["confidence"] = float(get("confidence"))
    if get("show_scale"):
        overrides["show_scale"] = float(get("show_scale"))
    if get("redetect_every"):
        overrides["redetect_every"] = int(get("redetect_every"))
    if get("image_size"):
        overrides["image_size"] = int(get("image_size"))
    actions = [Node(package="karis_perception", executable="pose_estimator",
                    name="karis_pose_estimator", output="screen", emulate_tty=True,
                    parameters=[get("params_file"), overrides])]
    if BOOL.get(get("camera_mount").lower(), True):
        actions.append(Node(
            package="tf2_ros", executable="static_transform_publisher", name="karis_camera_mount",
            arguments=["--x", get("camera_x"), "--y", get("camera_y"), "--z", get("camera_z"),
                       "--yaw", get("camera_yaw"), "--pitch", get("camera_pitch"),
                       "--roll", get("camera_roll"),
                       "--frame-id", "d435_link", "--child-frame-id", get("camera_frame")]))
    if get("player"):
        actions.append(Node(package="karis_perception", executable="camera_player",
                            name="karis_camera_player", output="screen",
                            parameters=[{"source": get("player"), "rate": float(get("player_rate")),
                                         "still_s": float(get("player_still_s"))}]))
    return actions


def generate_launch_description():
    share = get_package_share_directory("karis_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=os.path.join(share, "config", "perception.yaml")),
        DeclareLaunchArgument("model", default_value="", description="model file in models/ ('' = last used)"),
        DeclareLaunchArgument("backend", default_value="", description="isaac (the only one; kept for older commands)"),
        DeclareLaunchArgument("target_frame", default_value="", description="'' = value in perception.yaml"),
        DeclareLaunchArgument("show", default_value="", description="true = OpenCV window"),
        DeclareLaunchArgument("confidence", default_value="",
                              description="detector confidence threshold 0..1 ('' = perception.yaml)"),
        DeclareLaunchArgument("show_scale", default_value="",
                              description="window size relative to the image, e.g. 2 ('' = perception.yaml)"),
        DeclareLaunchArgument("redetect_every", default_value="",
                              description="run the detector every N processed frames ('' = perception.yaml)"),
        DeclareLaunchArgument("image_size", default_value="",
                              description="YOLO input size in px ('' = perception.yaml; 0 = the model's training size)"),
        DeclareLaunchArgument("karis_dir", default_value="", description="'' = $KARIS_DIR"),
        DeclareLaunchArgument("player", default_value="",
                              description=".bag file or frame folder to play instead of the robot camera"),
        DeclareLaunchArgument("player_rate", default_value="0.0", description="Hz, 0 = recording speed"),
        DeclareLaunchArgument("player_still_s", default_value="20.0",
                              description="photo folders (data/): seconds per photo"),
        DeclareLaunchArgument("camera_mount", default_value="true",
                              description="publish the static TF d435_link -> camera_frame"),
        DeclareLaunchArgument("camera_frame", default_value="head_camera_color_optical_frame"),
        DeclareLaunchArgument("camera_x", default_value="0.0"),
        DeclareLaunchArgument("camera_y", default_value="0.015",
                              description="the colour imager is ~15 mm left of the depth origin"),
        DeclareLaunchArgument("camera_z", default_value="0.0"),
        DeclareLaunchArgument("camera_yaw", default_value="-1.5707963"),
        DeclareLaunchArgument("camera_pitch", default_value="0.0"),
        DeclareLaunchArgument("camera_roll", default_value="-1.5707963"),
        OpaqueFunction(function=_nodes),
    ])
