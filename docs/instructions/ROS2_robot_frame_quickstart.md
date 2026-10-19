# Quickstart: live camera test with the parts' poses in the robot frame (`pelvis`)

The perception computes each part's pose in the camera frame and converts it to the robot's
`pelvis` frame itself (`/karis/object_poses_base`). For that it needs the transform
`pelvis → camera` in TF:

```
pelvis → waist joints (yaw, roll, pitch) → torso_link → d435_link → head_camera_color_optical_frame → karis/<object_id>
└──────────── robot model (team MoveIt container) ────────────┘   └ camera mount (perception launch) ┘   └ perception ┘
```

- **Robot model:** `robot_state_publisher` with the G1 URDF (`g1_description`) and the joint
  angles, in the team's MoveIt container. Either the full MoveIt stack (`run_g1.sh`) or, for
  perception tests, the read-only `observe_g1_tf.sh` (no arm control, no MoveIt).
- **Camera mount** (`d435_link → head_camera_color_optical_frame`): published by
  `perception.launch.py` itself (`camera_mount:=true`, the default).

Only the three waist joints lie between the pelvis and the camera; legs and arms do not change
the camera pose.

One-time setup for the camera and perception: steps 1–5 of
[ROS2_live_camera_quickstart.md](ROS2_live_camera_quickstart.md). For the MoveIt container:
`bash ros2_ws/scripts/install.sh` once on the host (see the root README).

## Terminals

| | Where | Purpose |
|---|---|---|
| **A** | WSL | settings, start the containers |
| **B** | SSH into the G1 | head camera |
| **C** | Isaac container | FoundationPose |
| **D** | perception container | detector + tracker, camera mount TF, overlay window |
| **E** | team MoveIt container (`ros2_ws/scripts/start.sh`) | robot model → TF `pelvis → d435_link` |

## Cable or WiFi

| | Cable | WiFi (team router) |
|---|---|---|
| Camera start (B) | `bash start_head_camera.sh` | `bash start_head_camera.sh --wifi --laptop <laptop IP>` |
| Settings (A) | `bash docker/setup_env.sh --cable` | `bash docker/setup_env.sh --robot <robot WiFi IP>` |
| Robot model (E) | `observe_g1_tf.sh` (real joints) or `run_g1.sh` | `observe_g1_tf.sh --fake` |
| Waist angles | real (`rt/lowstate`) | assumed straight (all joints 0) |

**Why `--fake` over WiFi:** the joint angles (`rt/lowstate`) come from the G1's motor
controller on the robot's internal cable network and do not reach the laptop over WiFi. With
`--fake` the waist is assumed straight. When the G1 stands normally its controller keeps the
waist close to straight, so the error is small; if the waist is turned or bent, the poses are
off by about that angle.

## Every time

### 1. B: camera on the robot

```bash
ssh unitree@192.168.123.164                 # menu: 1 = Foxy
sudo systemctl stop teleimager.service      # frees the camera
bash start_head_camera.sh                   # cable
# WiFi: bash start_head_camera.sh --wifi --laptop <laptop IP>
```

Wait for `Publishing 640x480 at 15 Hz ...` and, every 10 s, `15.0 frames/s ...`.

### 2. A: settings and the perception + Isaac containers

```bash
cd ~/Kitting-Humanoid-Robots-HT26
bash docker/setup_env.sh --cable            # WiFi: bash docker/setup_env.sh --robot <robot WiFi IP>
docker compose -f docker/compose.yaml -f docker/compose.wsl.yaml up -d --force-recreate perception
docker compose -f docker/compose.yaml -f docker/compose.wsl.yaml --profile isaac up -d --force-recreate isaac_fp
```

### 3. E: robot model in the team MoveIt container (new WSL window)

```bash
cd ~/Kitting-Humanoid-Robots-HT26
bash ros2_ws/scripts/start.sh               # opens (or re-opens) a shell in g1_xr_clean_humble
```

Inside the container:

```bash
bash /workspace/build.sh                    # first time and after KARIS code changes (karis_interfaces, karis_task, karis_bringup)
source /workspace/karis_ros_env.sh          # first line: CycloneDDS on <interface> (192.168.123.x)
bash /workspace/src/karis_bringup/scripts/observe_g1_tf.sh          # cable: real joint angles, read-only
# WiFi / no rt/lowstate: bash /workspace/src/karis_bringup/scripts/observe_g1_tf.sh --fake
```

It prints `READ-ONLY G1 rt/lowstate on <interface>` (cable) and `Robot model running`.
Nothing is commanded and the arms are not taken over.

**With MoveIt instead** (for picking): run the team's stack, following its safety procedure,
instead of `observe_g1_tf.sh` (never both: they publish the same TF):

```bash
bash /workspace/scripts/run_g1.sh --check   # read-only DDS check
bash /workspace/scripts/run_g1.sh           # robot model + move_group + supervised arm control
# second shell in E (bash ros2_ws/scripts/start.sh, then source /workspace/karis_ros_env.sh):
ros2 launch karis_bringup motion.launch.py  # scene_updater, pose_listener, pick_server (plan only)
```

### 4. C: Isaac helper (new WSL window)

```bash
cd ~/Kitting-Humanoid-Robots-HT26
docker compose -f docker/compose.yaml exec isaac_fp bash
ros2 launch karis_isaac_fp isaac_fp.launch.py classes:="Ratblock Chamfer"
```

Wait for `Isaac ROS FoundationPose bridge for ['Ratblock Chamfer'] ...` and two `Node was started`.

### 5. D: perception (new WSL window)

```bash
cd ~/Kitting-Humanoid-Robots-HT26
docker compose -f docker/compose.yaml exec perception bash
cd $KARIS_DIR
ros2 launch karis_bringup perception.launch.py model:=YOLO_26s.pt show:=true
```

The log must **not** keep printing `No TF pelvis <- head_camera_color_optical_frame yet`.
If it does, E is not running or does not reach D (see *Troubleshooting*).

### 6. Read the coordinates (a second shell in D)

```bash
cd ~/Kitting-Humanoid-Robots-HT26
docker compose -f docker/compose.yaml exec perception bash
ros2 run tf2_ros tf2_echo pelvis head_camera_color_optical_frame   # the camera in the robot frame; Ctrl+C
ros2 topic echo /karis/object_poses_base                           # all parts in pelvis
ros2 run tf2_ros tf2_echo pelvis karis/<object_id>                 # one part (id from the overlay/topic)
```

`header.frame_id` must be `pelvis`. The `pelvis` frame: origin in the middle of the hips,
**x forward, y to the robot's left, z up**, in metres. A part on a table in front of the robot
shows x ≈ +0.3 … +0.6, a small y, and a negative z (the table is below the pelvis).

| Output | Frame | Used by |
|---|---|---|
| `/karis/object_poses` | camera (`head_camera_color_optical_frame`) | debugging, recordings |
| `/karis/object_poses_base` | `pelvis` | MoveIt (`scene_updater`, `pick_server`) |
| TF `karis/<object_id>` | child of the camera frame | RViz, any TF lookup from `pelvis` |
| `/karis/get_objects` | the frame asked for | `pick_server` (asks for `pelvis`) |

### 7. Check the accuracy with a tape measure

The camera mount is the URDF's nominal (CAD) value, so expect about 1–2 cm error at the table
until it is checked. The tilt matters most: 1° ≈ 1 cm at 0.6 m.

1. Note the part's x, y, z.
2. Slide it exactly 10 cm straight forward: x should grow by ≈ 0.10, y and z stay the same.
3. Slide it 10 cm to the robot's left: y should grow by ≈ 0.10.
4. Height: z should equal (table height − pelvis height above the floor).

A few millimetres is good; 1–2 cm is expected. A **constant** offset is corrected in the
perception launch (D), without code changes, e.g.:

```bash
ros2 launch karis_bringup perception.launch.py model:=YOLO_26s.pt show:=true camera_z:=0.01 camera_pitch:=0.02
```

(`camera_x/y/z` in metres, `camera_yaw/pitch/roll` in radians, relative to the URDF's
`d435_link`; defaults `0 0.015 0 -1.5707963 0 -1.5707963`, so add your correction to those.)
If z changes when the part only moves forward, the tilt is off. Put good values into the
defaults in `ros2_ws/src/karis_bringup/launch/perception.launch.py` once they are measured.

### 8. Record for the motion group (a second shell in D)

```bash
cd $KARIS_DIR && mkdir -p bags
ros2 bag record -o bags/live_pelvis_$(date +%Y%m%d_%H%M) \
    /karis/object_poses_base /karis/object_poses /tf /tf_static /joint_states
```

Ctrl+C to stop. The bag has the poses directly in `pelvis` plus the full TF tree.

### Stop

Ctrl+C in D, C, E and B (the team container itself keeps running; `start.sh` re-opens it).
Then in A:

```bash
cd ~/Kitting-Humanoid-Robots-HT26
docker compose -f docker/compose.yaml --profile isaac down
```

On the robot, if its own software needs the camera: `sudo systemctl start teleimager.service`.

## Troubleshooting

| Problem | Fix |
|---|---|
| D: `No TF pelvis <- head_camera_color_optical_frame yet` | E not running or not reaching D. In D: `ros2 topic list \| grep -E "tf\|joint_states"`. Over WiFi, if nothing arrives from E: in E `export KARIS_DDS_PEERS=<laptop IP>` (its own 192.168.123.x address) before `observe_g1_tf.sh`. |
| D: TF `pelvis → d435_link` exists but not to the camera | `camera_mount:=false` was set, or the images use another frame: `camera_frame:=<frame_id of the images>`. |
| E: `start.sh` says another G1 container is running | The old KARIS MoveIt container: `docker rm -f g1_project_humble` (A), then `start.sh` again. |
| E: `Image missing; run ros2_ws/scripts/install.sh first` | Run it on the host (A), once. It must not run while a G1 container is running. |
| E: `observe_g1_tf.sh` says an observer / robot_state_publisher is already running | `run_g1.sh` (or another copy) already publishes the robot model; use that one. |
| E: no `READ-ONLY ... rt/lowstate` or `No fresh joint states` | Normal over WiFi: use `--fake`. On the cable: robot powered on; `G1_INTERFACE=<cable interface>` if several 192.168.123.x interfaces exist. |
| `/karis/object_poses_base` empty but `/karis/object_poses` has parts | Same as the first row (no TF). |
| Poses wrong after the robot turned/bent its waist | Running with `--fake`: use the cable for real waist angles. |
| No camera topics, Isaac or perception errors | See *Troubleshooting* in [ROS2_live_camera_quickstart.md](ROS2_live_camera_quickstart.md). |
