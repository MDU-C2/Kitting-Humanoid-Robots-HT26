"""Isaac ROS FoundationPose nodes (register + tracking per part class) and fp_bridge.

    ros2 launch karis_isaac_fp isaac_fp.launch.py [classes:="Rund mutter,Ratblock Chamfer"]
    ros2 launch karis_isaac_fp isaac_fp.launch.py lean:=true      (less GPU memory, see below)

Default: about 4.2 GB of GPU memory per class plus about 0.8 GB once. Most of it is the
register node (score engine for 252 pose hypotheses, ~2.3 GB). Memory savers, each optional:
  shared_register:=true   one register node for all classes; the bridge switches its mesh
                          before each registration (-> ~3.4 GB less per extra class)
  track_engine:=<file>    batch-1 refine engine for the tracking nodes (~0.4 GB less per class);
                          'auto' = <models_dir>/refine_trt_engine_batch1.plan
  max_image:=WxH          size the point-cloud pools for images up to WxH instead of 1920x1440
                          (~0.6 GB less per class); larger images then fail
  lean:=true              all three (track_engine auto, max_image 640x480)
"""
import os
import re
import shutil

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
FP_PKG = "isaac_ros_foundationpose"
GRAPHS = ("nitros_foundationpose_node.yaml", "nitros_foundationpose_tracking_node.yaml")
POINTCLOUD_BLOCK = 1920 * 1440 * 3 * 4      # depth_to_pointcloud pool block in Isaac's graphs
TRUE = ("1", "true", "yes", "on")


def _pool_overlay(max_image):
    """Copy of Isaac's FoundationPose config with the point-cloud pools sized for max_image
    ("WxH"), as an ament prefix to put in front of AMENT_PREFIX_PATH. The installed package
    is not changed."""
    from ament_index_python.packages import get_package_share_directory
    w, h = (int(v) for v in max_image.lower().split("x"))
    src = get_package_share_directory(FP_PKG)
    root = f"/tmp/karis_fp_overlay_{w}x{h}"
    share = os.path.join(root, "share", FP_PKG)
    shutil.rmtree(root, ignore_errors=True)
    shutil.copytree(os.path.join(src, "config"), os.path.join(share, "config"))
    shutil.copy(os.path.join(src, "package.xml"), share)
    marker = os.path.join(root, "share", "ament_index", "resource_index", "packages")
    os.makedirs(marker)
    open(os.path.join(marker, FP_PKG), "w").close()
    for g in GRAPHS:
        path = os.path.join(share, "config", g)
        with open(path) as f:
            text = f.read()
        text, n = re.subn(rf"block_size: {POINTCLOUD_BLOCK}\b", f"block_size: {w * h * 3 * 4}", text)
        if n != 1:
            raise RuntimeError(f"{path}: expected 1 point-cloud pool of {POINTCLOUD_BLOCK} bytes, "
                               f"found {n} (other Isaac ROS version?): run without max_image")
        with open(path, "w") as f:
            f.write(text)
    return root


def _setup(context):
    get = lambda n: LaunchConfiguration(n).perform(context)   # noqa: E731
    lean = get("lean").lower() in TRUE
    shared = get("shared_register").lower() in TRUE or lean
    track_engine = get("track_engine") or ("auto" if lean else "")
    max_image = get("max_image") or ("640x480" if lean else "")
    meshes_yaml = get("meshes_yaml")
    models = get("models_dir")
    refine = os.path.join(models, "refine_trt_engine.plan")
    score = os.path.join(models, "score_trt_engine.plan")
    if track_engine == "auto":
        track_engine = os.path.join(models, "refine_trt_engine_batch1.plan")
    track_refine = track_engine or refine
    for f in (meshes_yaml, refine, score, track_refine):
        if not os.path.exists(f):
            raise FileNotFoundError(
                f"{f} is missing. Meshes: run make_isaac_meshes in the perception container; "
                "engines: run /opt/karis/prepare_models.sh in this container.")
    with open(meshes_yaml) as f:
        meshes = yaml.safe_load(f)["classes"]
    only = [c.strip() for c in get("classes").split(",") if c.strip()]
    selected = {n: m for n, m in meshes.items() if not only or n in only or m["slug"] in only}
    if not selected:
        raise RuntimeError(f"no classes selected from {meshes_yaml} (classes:='{get('classes')}')")

    depth = {"min_depth": float(get("min_depth")), "max_depth": float(get("max_depth"))}

    def register_node(ns, mesh, frame):
        return ComposableNode(
            name="foundationpose", namespace=ns, package=FP_PKG,
            plugin="nvidia::isaac_ros::foundationpose::FoundationPoseNode",
            parameters=[{**depth, **TENSORS, **SCORE_TENSORS, "mesh_file_path": mesh,
                         "tf_frame_name": frame, "refine_engine_file_path": refine,
                         "score_engine_file_path": score,
                         "refine_iterations": int(get("refine_iterations"))}])

    nodes = []
    if shared:
        first = next(iter(selected.values()))
        nodes.append(register_node("/karis_fp/shared", first["obj"], "isaac_fp/shared"))
    for name, m in selected.items():
        ns = f"/karis_fp/{m['slug']}"
        if not shared:
            nodes.append(register_node(ns, m["obj"], f"isaac_fp/{m['slug']}"))
        nodes.append(ComposableNode(
            name="foundationpose_tracking", namespace=ns, package=FP_PKG,
            plugin="nvidia::isaac_ros::foundationpose::FoundationPoseTrackingNode",
            parameters=[{**depth, **TENSORS, "mesh_file_path": m["obj"],
                         "tf_frame_name": f"isaac_fp/{m['slug']}",
                         "refine_engine_file_path": track_refine}]))

    env = {}
    if max_image:
        overlay = _pool_overlay(max_image)
        env["AMENT_PREFIX_PATH"] = overlay + os.pathsep + os.environ.get("AMENT_PREFIX_PATH", "")
    print(f"[isaac_fp] classes {sorted(selected)}; register node: "
          f"{'shared' if shared else 'one per class'}; tracking engine: {os.path.basename(track_refine)}; "
          f"point-cloud pools: {max_image or '1920x1440 (Isaac default)'}", flush=True)
    return [
        ComposableNodeContainer(name="karis_fp_container", namespace="", package="rclcpp_components",
                                executable="component_container_mt",
                                composable_node_descriptions=nodes, output="screen",
                                additional_env=env or None),
        Node(package="karis_isaac_fp", executable="fp_bridge", output="screen",
             parameters=[{"meshes_yaml": meshes_yaml, "classes": get("classes"),
                          "shared_register": shared, "max_image": max_image,
                          "mesh_switch_wait_s": float(get("mesh_switch_wait_s"))}]),
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
        DeclareLaunchArgument("lean", default_value="false",
                              description="true = shared_register, track_engine auto, max_image 640x480"),
        DeclareLaunchArgument("shared_register", default_value="false",
                              description="true = one register node for all classes"),
        DeclareLaunchArgument("track_engine", default_value="",
                              description="refine engine for tracking ('' = the register one, auto = batch-1)"),
        DeclareLaunchArgument("max_image", default_value="",
                              description="WxH: point-cloud pools for images up to this size ('' = 1920x1440)"),
        DeclareLaunchArgument("mesh_switch_wait_s", default_value="0.5",
                              description="shared_register: wait after switching the mesh"),
        OpaqueFunction(function=_setup),
    ])
