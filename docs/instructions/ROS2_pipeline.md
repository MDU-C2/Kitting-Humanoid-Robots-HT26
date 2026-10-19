# KARIS over ROS 2: camera → vision → motion

Everything off the robot runs **ROS 2 Humble** in Docker. The G1 keeps its own **Foxy** and only publishes its camera; its joint states (`rt/lowstate`) are read by the team's MoveIt stack with the Unitree SDK.

```
 G1 PC2 (Foxy, no Docker)              Vision computer (Docker, all Humble, host network)                  MoveIt container (team, Humble)
 ─────────────────────────             ───────────────────────────────────────────────────────            ─────────────────────────
 robot/g1_camera_ros2.py      ───▶     perception  (karis/perception:humble)                     ───▶     g1_readonly_observer + relay → /joint_states
  /g1/head_camera/... (compressed)       detector (YOLO / RF-DETR) + KARIS tracker                          robot_state_publisher → TF
 G1 firmware: rt/lowstate ───────┐      FoundationPose (Isaac helper):                                     scene_updater → MoveIt scene
                                  │                                                                          pick_server: /karis/pick
                                  │                 ───── /karis/fp/estimate_pose ─────┐                     → move_group
                                  │      /karis/object_poses, TF karis/<id>, services  │
                                  │                                                    ▼
                                  │    isaac_fp  (karis/isaac_fp:3.2, the helper)      
                                  │      Isaac ROS 3.2 FoundationPose (TensorRT), one register +
                                  │      one tracking node per part class, KARIS bridge
                                  └────────────────────────────────────────────────────────────────▶ (MoveIt container)
```

## Foxy on the robot, Humble everywhere else

ROS 2 distributions can talk to each other when all sides use the same DDS implementation and the messages they share have the same definitions. This is not officially supported, so the setup keeps the shared part small ([discussion](https://discourse.openrobotics.org/t/ros-cross-distribution-communication/27335)):

- **The same DDS everywhere.** Every side uses CycloneDDS 0.10: Unitree's build on the G1 (its own DDS) and Humble's in every container. `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` is set by `karis_ros_env.sh` in the containers and by `start_head_camera.sh` on the robot.
- **Few messages cross Foxy↔Humble.** Only standard camera messages (`sensor_msgs/CompressedImage`, `CameraInfo`) cross the distribution boundary; their definitions did not change between Foxy and Humble. The joint states are read with the Unitree SDK (`unitree_sdk2_python`) in the MoveIt container, not as ROS messages.
- **KARIS messages stay on Humble.** `karis_interfaces` messages travel only between the Humble containers.

## FoundationPose: in the Isaac helper

FoundationPose runs only in the `isaac_fp` helper container: NVIDIA Isaac ROS 3.2
FoundationPose with TensorRT FP16 engines (tracking ~45–70 ms per part on an RTX 3060
laptop GPU). It needs an RTX GPU, NVIDIA driver ≥ 560, and native Linux (on WSL it is
unofficial but works). Each part class takes about 1.5 GB of GPU memory. The perception
image has no FoundationPose of its own (the earlier PyTorch backend was removed).

Only FoundationPose itself is in the helper. Detection, matching detections to tracks, re-detection, drift handling, ids, topics and TF stay the same KARIS code, so motion planning sees no difference.

The helper is called once per part per frame. Images go to it only with the first call of each frame. Isaac's tracking state for every part is kept by the helper.

## The interface between vision and motion (`karis_interfaces`)

| Name | Type | What |
|---|---|---|
| `/karis/object_poses` | `karis_interfaces/ObjectPoseArray` | every part in the current frame, in the camera optical frame |
| `/karis/object_poses_base` | `karis_interfaces/ObjectPoseArray` | the same in `pelvis` (when the robot model runs) |
| TF `head_camera_color_optical_frame → karis/<object_id>` | tf2 | one frame per part, e.g. `karis/rund_mutter_1` |
| `/karis/get_objects` | `karis_interfaces/srv/GetObjects` | ask for parts by class or id, in any TF frame |
| `/karis/reset`, `/karis/detect_now` | `std_srvs/srv/Trigger` | start over / run the detector now |
| `/karis/pick` | `karis_interfaces/action/PickObject` | pick a part (and place it in a slot) |
| `/karis/markers`, `/karis/viz/annotated/compressed` | MarkerArray, CompressedImage | RViz boxes and the live overlay |
| `/karis/fp/estimate_pose` | `karis_interfaces/srv/EstimatePose` | internal: perception ↔ Isaac helper |

Each `ObjectPose` carries:

- `object_id`: lower case with underscores, e.g. `rund_mutter_1`. It stays the same while the part is tracked.
- `class_name` and `confidence`.
- `mode`: `register`, `track` or `centre`.
- `pose`: the frame of the CAD model.
- `box_pose` and `box_size`: the tight bounding box.
- `fit_iou` and `age_s`.

Tray slots (classes without a mesh) have `has_orientation: false`; only their position is valid.

## Packages and files

| Path | Runs in | Contents |
|---|---|---|
| `ros2_ws/src/karis_interfaces` | every container | messages, services, action |
| `ros2_ws/src/karis_perception` | perception | `pose_estimator`, `camera_player`, `bag_reindex`, `make_isaac_meshes` |
| `ros2_ws/src/karis_isaac_fp` | isaac_fp | `isaac_fp.launch.py` (Isaac ROS FoundationPose nodes), `fp_bridge` |
| `ros2_ws/src/karis_task` | MoveIt | `pose_listener`, `scene_updater`, `pick_server` |
| `ros2_ws/src/karis_bringup` | perception, MoveIt | launch files (`perception.launch.py` incl. the camera mount TF, `motion.launch.py`), `config/perception.yaml`, `scripts/observe_g1_tf.sh` (read-only robot model for perception tests) |
| `ros2_ws/src/g1_description`, `g1_moveit_config`, `g1_xr_bridge` | MoveIt | the team's MoveIt stack: URDF, MoveIt config, XR arm controller, read-only joint-state observer |
| `ros2_ws/scripts/`, `ros2_ws/deployment/` | | the team's MoveIt container: `install.sh`, `start.sh`, `run_g1.sh` |
| `ros2_ws/karis_ros_env.sh`, `ros2_ws/build.sh` | every container | environment and build. Each container builds into its own folders (`install_perception/`, `install_isaac/`, `install/`), chosen by `KARIS_ROLE`. |
| `docker/compose.yaml`, `docker/setup_env.sh`, `docker/.env.example`, `docker/compose.wsl.yaml` | vision computer | the perception and Isaac containers |
| `docker/perception/Dockerfile` | | Ubuntu 22.04, Humble, PyTorch 2.4 (for YOLO), Ultralytics ≥ 8.4, RF-DETR (own venv); no FoundationPose |
| `docker/isaac_fp/` | | `build.sh`, `Dockerfile` (on NVIDIA's Isaac ROS 3.2 base image), `prepare_models.sh` (NGC models → TensorRT engines) |
| `robot/g1_camera_ros2.py`, `robot/start_head_camera.sh` | G1 (Foxy) | camera → ROS 2 topics |

All code is in the ROS 2 packages: the detector, tracker and helpers in `karis_perception/karis_perception/`, the part classes with their CAD meshes in `karis_perception/config/classes.yaml` and `karis_perception/meshes/`, and every setting (confidence, detector rate, depth range, ...) in `karis_bringup/config/perception.yaml`. Only local data stays in `local_data/` (`$KARIS_DIR`): the trained models (`models/`), test photos and recordings, and the generated `meshes_isaac/`.

---

## One-time setup

### 1. The vision computer

You need:

- an NVIDIA GPU, with driver ≥ 560 for the Isaac helper (CUDA 12.6);
- Docker Engine with the Docker Compose plugin;
- the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

Native Ubuntu 22.04 is best. The team's Ubuntu PC is a good choice if it has an RTX GPU.

**Network:**
- Be on the robot network: by cable, or on the team router.
- Make the receive buffers bigger, so camera frames are not dropped. Run this on the host, not in a container:
  ```bash
  sudo sysctl -w net.core.rmem_max=33554432 net.core.rmem_default=8388608
  printf 'net.core.rmem_max=33554432\nnet.core.rmem_default=8388608\n' | sudo tee /etc/sysctl.d/10-karis-dds.conf
  ```

**Only on Windows + WSL 2:**

- **Networking.** The G1 must be able to reach the containers, so WSL needs mirrored networking. Put this in `C:\Users\<you>\.wslconfig`:
  ```ini
  [wsl2]
  networkingMode=mirrored
  ```
  Then run in PowerShell as administrator:
  ```powershell
  Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
  wsl --shutdown
  ```
- **Docker.** Use Docker Engine inside WSL, not Docker Desktop.
- **Compose.** Add `-f docker/compose.wsl.yaml` to every compose command.
- **Isaac helper.** Isaac ROS is not officially supported on WSL. If the helper fails there, run it on a Linux PC: the perception container finds it over the network.

### 2. Repo and settings

```bash
cd ~
git clone https://github.com/MDU-C2/Kitting-Humanoid-Robots-HT26.git
cd Kitting-Humanoid-Robots-HT26
bash docker/setup_env.sh                # writes docker/.env (paths of this clone); --robot <ip> for WiFi
```

Nothing in the repo has to be edited by hand: `setup_env.sh` takes the paths from where you
cloned it, and inside the containers `$KARIS_REPO` (the repo) and `$KARIS_DIR` (`local_data/`, the local
data folder) point to it. Models (`local_data/models/`), test data and the Docker images stay on
each computer (not in git).

### 3. Perception container

```bash
docker compose -f docker/compose.yaml build perception        # 10-30 min the first time (downloads)
docker compose -f docker/compose.yaml up -d perception
docker compose -f docker/compose.yaml exec perception bash
# inside:
bash $KARIS_WS/build.sh
python3 -c "import torch, rclpy; print(torch.cuda.is_available())"          # True
```

### 4. Isaac helper

```bash
bash docker/isaac_fp/build.sh                                   # NVIDIA's Isaac ROS 3.2 base + KARIS layer (large)
docker compose -f docker/compose.yaml exec perception \
    bash -ic "ros2 run karis_perception make_isaac_meshes"      # CAD meshes -> textured OBJ in KARIS_DIR/meshes_isaac
docker compose -f docker/compose.yaml --profile isaac up -d isaac_fp
docker compose -f docker/compose.yaml exec isaac_fp bash
# inside:
/opt/karis/prepare_models.sh      # downloads FoundationPose ONNX from NGC, builds TensorRT engines (minutes)
bash $KARIS_WS/build.sh
```
Run `make_isaac_meshes` again after changing `classes.yaml` (and rebuild first). Run `prepare_models.sh --force` again after changing GPU or driver.

### 5. The G1

```bash
scp robot/g1_camera_ros2.py robot/start_head_camera.sh unitree@192.168.123.164:~/
```

### 6. MoveIt container (team image)

On the host: `bash ros2_ws/scripts/install.sh` (builds the image and the MoveIt workspace), then `bash ros2_ws/scripts/start.sh` (opens a shell in the container, host network). Inside, build the KARIS motion nodes once: `bash /workspace/build.sh && source /workspace/karis_ros_env.sh`.

---

## Every day

**G1** (`ssh unitree@192.168.123.164`, choose 1 = Foxy):
```bash
bash start_head_camera.sh
```

**Perception** (`docker compose -f docker/compose.yaml up -d perception`, then `... exec perception bash`):
```bash
ros2 topic hz /g1/head_camera/color/image_raw/compressed       # ~15 Hz: the camera arrives
ros2 launch karis_bringup perception.launch.py model:=best8s.pt        # needs the Isaac helper running
```

**Isaac helper** (start it first; `docker compose -f docker/compose.yaml --profile isaac up -d`, then `... exec isaac_fp bash`):
```bash
ros2 launch karis_isaac_fp isaac_fp.launch.py                      # all classes in meshes.yaml
ros2 launch karis_isaac_fp isaac_fp.launch.py classes:="Rund mutter,Ratblock Chamfer"   # less GPU memory
```

**MoveIt container** (`bash ros2_ws/scripts/start.sh`; each shell: `source /workspace/karis_ros_env.sh`):
```bash
bash /workspace/scripts/run_g1.sh          # the team's MoveIt stack (robot model, move_group, arm control); see its README
ros2 launch karis_bringup motion.launch.py # second shell: scene_updater, pose_listener, pick_server
```
Perception tests without MoveIt/arm control: `bash /workspace/src/karis_bringup/scripts/observe_g1_tf.sh` instead of `run_g1.sh` (read-only robot model; `--fake` = waist straight, e.g. over WiFi). Never both: they publish the same TF.

**Try it** (any container):
```bash
ros2 topic echo /karis/object_poses_base --once
ros2 service call /karis/get_objects karis_interfaces/srv/GetObjects "{class_name: 'Rund mutter', target_frame: 'pelvis', fresh: true}"
ros2 action send_goal /karis/pick karis_interfaces/action/PickObject \
  "{class_name: 'Rund mutter', place_slot: 'Rund mutter HOLE 11432713', dry_run: true}" --feedback
ros2 run rqt_image_view rqt_image_view /karis/viz/annotated        # transport "compressed"
```

**Without the robot:** add `player:=...` to the perception launch, and use `observe_g1_tf.sh --fake` for the robot model. Step-by-step tests: photos [ROS2_image_test_quickstart.md](ROS2_image_test_quickstart.md), video [ROS2_video_test_quickstart.md](ROS2_video_test_quickstart.md).

- **A recording:** `player:=$KARIS_DIR/videos/<name>.bag`, or a frame folder with `timestamps.csv`.
- **The image-test photos:** `player:=$KARIS_DIR/data`. It needs `rgb/`, `depth/` and `cam_K/` subfolders. Each photo is shown as a still camera view for `player_still_s` seconds (default 20). The perception is reset at every new photo, so parts of one photo are not carried into the next. For photos with many parts use e.g. `player_still_s:=40`.

## Moving the arm

`pick_server` sends each pose (pre-grasp, grasp, lift, slot) to MoveIt's `move_group` on `/move_action`, for the group `planning_group` (default `right_arm`) and the link `ee_link` (default `right_wrist_yaw_link`).

- **Needs `move_group`** from the team's MoveIt stack (`g1_moveit_config`, started by `run_g1.sh`). Without it, use `dry_run: true`.
- **`execute:=false`** plans only; **`execute:=true`** moves the real arm.
- **During a pick**, the part being picked is taken out of the planning scene. The other parts stay in as obstacles.
- **Still to do on the robot:**
  - opening and closing the Inspire hand;
  - measuring `grasp_offset_z` and `grasp_rpy_deg` (`ee_link` is the wrist, not the fingertips);
  - checking the camera mount (the `camera_*` arguments of `perception.launch.py`): [ROS2_robot_frame_quickstart.md](ROS2_robot_frame_quickstart.md).

## Troubleshooting

| Problem | Fix |
|---|---|
| `/g1/head_camera/...` topics not visible | All sides must use `ROS_DOMAIN_ID=0`. The line printed by `karis_ros_env.sh` must name the 192.168.123.x interface. Test discovery with `ros2 multicast receive` on one side and `ros2 multicast send` on the other. If multicast does not pass, run `bash docker/setup_env.sh --robot 192.168.123.164` (WiFi: the robot's WiFi address) and recreate the containers. |
| Topics listed but few frames | Receive buffers (setup 1), then the firewall. |
| G1 side errors | See `robot/start_head_camera.sh`: conda env (`CONDA_ENV=...`), `teleimager.service`, interface (`ROBOT_IF=eth0`). |
| "not available: is the Isaac helper running?" | Start `isaac_fp.launch.py`. `ros2 service list` must show `/karis/fp/estimate_pose`. |
| Isaac helper: missing engines or meshes | `/opt/karis/prepare_models.sh` in the helper; `make_isaac_meshes` in perception. |
| Isaac helper: out of GPU memory | About 1.5 GB per class: start fewer classes (`classes:=...`), or run the helper on another PC on the robot network. |
| Isaac helper: timeouts | Check the log of `karis_fp_container`. After a failed registration, Isaac ignores new input of that class for up to 5 s, so the next calls can time out too; they recover on their own. |
| Bridge says "meshes.yaml is from an older make_isaac_meshes" | Run `make_isaac_meshes` again; the meshes are now written centred. |
| No `/karis/object_poses_base` | The robot model is not running (`run_g1.sh` or `observe_g1_tf.sh` in the MoveIt container), so there is no TF `pelvis → camera`. |
| RViz drops `karis/*` frames | Outputs use the vision computer's clock (`stamp_source: now`); keep that unless chrony syncs the G1. |
| `InvalidParameterTypeException` | Humble checks types: write decimals, e.g. `-p approach_height:=0.10`. |
