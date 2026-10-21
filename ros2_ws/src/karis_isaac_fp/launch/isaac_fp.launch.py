"""Isaac ROS FoundationPose nodes (register + tracking per part class) and fp_bridge.

    ros2 launch karis_isaac_fp isaac_fp.launch.py [classes:="Rund mutter,Ratblock Chamfer"]

Uses about 1.5 GB of GPU memory per class.
"""
import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

TENSORS = {
    "refine_input_tensor_names": ["input_tensor1", "input_tensor2"],
    "refine_input_binding_names": ["input1", "input2"],
    "refine_output_tensor_names": ["output_tensor1", "output_tensor2"],
    "refine_output_binding_names": ["output1", "output2"],
}
SCORE_TENSORS = {
    "score_input_tensor_names": ["input_tensor1", "input_tensor2"],
    "score_input_binding_names": ["input1", "input2"],
    "score_output_tensor_names": ["output_tensor"],
    "score_output_binding_names": ["output1"],
}


def _setup(context):
    get = lambda n: LaunchConfiguration(n).perform(context)   # noqa: E731
    meshes_yaml = get("meshes_yaml")
    models = get("models_dir")
    refine = os.path.join(models, "refine_trt_engine.plan")
    score = os.path.join(models, "score_trt_engine.plan")
    for f in (meshes_yaml, refine, score):
        if not os.path.exists(f):
            raise FileNotFoundError(
                f"{f} is missing. Meshes: run make_isaac_meshes in the perception container; "
                "engines: run /opt/karis/prepare_models.sh in this container.")
    with open(meshes_yaml) as f:
        meshes = yaml.safe_load(f)["classes"]
    only = [c.strip() for c in get("classes").split(",") if c.strip()]
    nodes = []
    for name, m in meshes.items():
        if only and name not in only and m["slug"] not in only:
            continue
        ns = f"/karis_fp/{m['slug']}"
        common = {"mesh_file_path": m["obj"], "min_depth": float(get("min_depth")),
                  "max_depth": float(get("max_depth")),
                  "tf_frame_name": f"isaac_fp/{m['slug']}", "refine_engine_file_path": refine}
        nodes.append(ComposableNode(
            name="foundationpose", namespace=ns, package="isaac_ros_foundationpose",
            plugin="nvidia::isaac_ros::foundationpose::FoundationPoseNode",
            parameters=[{**common, **TENSORS, **SCORE_TENSORS, "score_engine_file_path": score,
                         "refine_iterations": int(get("refine_iterations"))}]))
        nodes.append(ComposableNode(
            name="foundationpose_tracking", namespace=ns, package="isaac_ros_foundationpose",
            plugin="nvidia::isaac_ros::foundationpose::FoundationPoseTrackingNode",
            parameters=[{**common, **TENSORS}]))
    if not nodes:
        raise RuntimeError(f"no classes selected from {meshes_yaml} (classes:='{get('classes')}')")
    return [
        ComposableNodeContainer(name="karis_fp_container", namespace="", package="rclcpp_components",
                                executable="component_container_mt",
                                composable_node_descriptions=nodes, output="screen"),
        Node(package="karis_isaac_fp", executable="fp_bridge", output="screen",
             parameters=[{"meshes_yaml": meshes_yaml, "classes": get("classes")}]),
    ]


def generate_launch_description():
    karis = os.environ.get("KARIS_DIR") or os.path.join(
        os.path.dirname(os.environ.get("KARIS_WS", "/home/kimsv/Kitting-Humanoid-Robots-HT26/ros2_ws")), "local_data")
    return LaunchDescription([
        DeclareLaunchArgument("meshes_yaml", default_value=os.path.join(karis, "meshes_isaac", "meshes.yaml")),
        DeclareLaunchArgument("models_dir", default_value=os.environ.get("KARIS_ISAAC_MODELS", "/models")),
        DeclareLaunchArgument("classes", default_value="", description="comma-separated; '' = all"),
        DeclareLaunchArgument("refine_iterations", default_value="1"),
        DeclareLaunchArgument("min_depth", default_value="0.1"),
        DeclareLaunchArgument("max_depth", default_value="3.0"),
        OpaqueFunction(function=_setup),
    ])
