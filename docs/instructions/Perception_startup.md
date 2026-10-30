# Quickstart: test that the perception gives the part's pose in the robot frame (`pelvis`)

Runs the perception live on the G1 over WiFi: the head camera → the perception (YOLO + Isaac ROS FoundationPose) → the part's pose in robot's frame `pelvis`.

The robot itself publishes its robot model (TF `pelvis` → waist → `torso_link` → `d435_link`, from the real waist joints, read only): `start_head_camera.sh --tf`. The perception adds the camera mount (`d435_link` → `head_camera_color_optical_frame`) and so gets `/karis/object_poses_base` and the green `pelvis` lines in its window.

| What you check | Where |
|---|---|
| The robot's TF arrives | `tf2_echo pelvis head_camera_color_optical_frame` ≈ `[0.054, 0.033, 0.474]` |
| The part in `pelvis` | the window (`pelvis: from TF`, green `pelvis xyz`) and `/karis/object_poses_base` |
| It is right | a tape measure from the robot to the part |

## How to open each terminal

Every terminal below starts the same way:

- If the tab is **Ubuntu** (prompt `$USER@LAPTOP-PQVFOCEA:~$`): continue with the commands.
- If the tab is **PowerShell** (prompt `PS C:\Users\@User>`): type `wsl` first.

`ros2` only exists **inside the containers**, never in plain WSL.

| Terminal | What runs there | Prompt when ready |
|---|---|---|
| **WSL** | settings, starting/stopping containers | `$User@LAPTOP-PQVFOCEA:~/Kitting-Humanoid-Robots-HT26$` |
| **Robot** | the camera + the robot model TF on the G1 | `unitree@ubuntu:~$` |
| **Isaac** | the Isaac FoundationPose helper | `root@LAPTOP-PQVFOCEA:.../ros2_ws#` |
| **Perception** | the perception (pose estimator) | `root@LAPTOP-PQVFOCEA:.../local_data#` |
| **Check** | the checks | `root@LAPTOP-PQVFOCEA:.../local_data#` |

## Addresses

| | WiFi |
|---|---|
| Robot | `192.168.123.213` (wlan0) |
| Robot | `192.168.123.164` (eth0) |
| Laptop | `192.168.123.184` (WSL interface `eth1`) |

---

## 1. WSL terminal: network check, settings and containers

New tab (wsl):

```bash
cd ~/Kitting-Humanoid-Robots-HT26
git pull --no-rebase
ip -4 -o addr | grep 192.168.123                                # eth1 with 192.168.123.184
ping -c 3 192.168.123.213                                       # the robot answers over WiFi
bash docker/setup_env.sh --robot 192.168.123.213 --if eth1
docker compose -f docker/compose.yaml -f docker/compose.wsl.yaml up -d --force-recreate perception
docker compose -f docker/compose.yaml -f docker/compose.wsl.yaml --profile isaac up -d --force-recreate isaac_fp
```

After this, always open **new** container terminals.

## 2. Robot terminal: start the camera and the robot model TF

New tab (wsl):

```bash
ssh unitree@192.168.123.213                                     # the robot's ip over wlan0
```

Password, then at the menu type `1` (Foxy). On the robot (prompt `unitree@ubuntu:~$`):

```bash
sudo systemctl stop teleimager.service                  # free the camera if not already
unset CYCLONEDDS_URI                                    # the robot's own setting would override the script's
bash start_head_camera.sh --wifi --laptop 192.168.123.184 --tf
```

It must print:
- `... interface=wlan0 (192.168.123.213/24)  laptop peers: 192.168.123.184 ...`
- `Robot model TF: pelvis -> torso_link -> d435_link at 50 Hz (read only, rt/lowstate via eth0, SDK python .../motiontest/bin/python)`
- `# reading rt/lowstate on eth0 (read only)`
- `waist angles (stdin) -> TF pelvis -> torso_link -> d435_link at 50 Hz`
- `Publishing 640x480 at 15 Hz ...`, and every 10 s `15.0 frames/s`
- after a few seconds `lowstate 100 Hz, waist yaw/roll/pitch +0.0/+0.0/+0.0 deg` (about)

## 3. Isaac terminal: start the Isaac helper

New tab (wsl):

```bash
cd ~/Kitting-Humanoid-Robots-HT26
docker compose -f docker/compose.yaml exec isaac_fp bash
```

Inside (prompt `root@LAPTOP-PQVFOCEA:...#`). The first line must say: `CycloneDDS on eth1 (192.168.123.184/24)`.

```bash
bash $KARIS_WS/build.sh && source $KARIS_WS/karis_ros_env.sh        # only after a git pull / code change
ros2 launch karis_isaac_fp isaac_fp.launch.py classes:="Ratblock Chamfer" refine_iterations:=3 max_depth:=2
```

Wait for `Isaac ROS FoundationPose bridge for ['Ratblock Chamfer'] ...` and two `Node was started`.
Leave this terminal running.

## 4. Perception terminal: check the camera, then start

New tab (ws`):

```bash
cd ~/Kitting-Humanoid-Robots-HT26
docker compose -f docker/compose.yaml exec perception bash
```

Inside (prompt `root@LAPTOP-PQVFOCEA:.../local_data#`). The first line must sa: `CycloneDDS on eth1 (192.168.123.184/24)`.

```bash
bash $KARIS_WS/build.sh && source $KARIS_WS/karis_ros_env.sh    # only after a git pull / code change
ros2 daemon stop
ros2 launch karis_bringup perception.launch.py model:=YOLO_26s.pt show:=true confidence:=0.75
```

The window will show ratblock_chamfer with box and axes, and on the right **`pelvis: from TF`** with the green `pelvis xyz/rpy` of the
part. Leave this terminal running.

## 5. Check terminal: TF and the part in pelvis

New tab (wsl):

```bash
cd ~/Kitting-Humanoid-Robots-HT26
docker compose -f docker/compose.yaml exec perception bash
```

Inside (prompt `root@LAPTOP-PQVFOCEA:.../local_data#`):

```bash
ros2 run tf2_ros tf2_echo pelvis d435_link                         # ~[0.054, 0.018, 0.474] (from the robot); Ctrl+C
ros2 run tf2_ros tf2_echo pelvis head_camera_color_optical_frame   # ~[0.054, 0.033, 0.474]; Ctrl+C
ros2 topic hz /tf                                                  # ~50 Hz; Ctrl+C
ros2 topic echo /karis/object_poses_base --once                    # the block in pelvis
```

The values above are for a straight waist. In `/karis/object_poses_base` look at`header.frame_id: pelvis` and the block's `pose.position`.

## 6. Stop

1. Ctrl+C in **Perception**, **Isaac** and **Robot** (the robot stops the camera and the TF together).
2. **WSL** terminal:

```bash
cd ~/Kitting-Humanoid-Robots-HT26
docker compose -f docker/compose.yaml --profile isaac down
```



## Other way: pelvis from the motion laptop instead

If the MoveIt team runs their stack (`run_g1.sh`, with ROS 2 also on WiFi:
`G1_ROS_EXTRA_INTERFACES` / `G1_ROS_PEERS=192.168.123.184`) or `observe_g1_tf.sh`, their robot
model publishes `pelvis`. Then in step 2 start the camera **without** `--tf`:

```bash
bash start_head_camera.sh --wifi --laptop 192.168.123.184
```

and add the motion laptop's WiFi address as a peer in step 1:
`bash docker/setup_env.sh --robot "192.168.123.213 <motion laptop IP>" --if eth1`.
Steps 3–6 are the same.

## Messages you can ignore

| Message | Why |
|---|---|
| `No TF pelvis <- head_camera_color_optical_frame yet` in the first seconds | The robot's TF had not arrived yet. |
| `... not part of the same tree ...` once at start-up | The camera mount started a moment before the robot's TF arrived. |
| `unknown class '...' (bridge has [...]) -> ... position only` | The Isaac helper was started without that class (`classes:=`). |
| `Clock skew detected` during `build.sh` | WSL's clock differs slightly from Windows'; harmless. |
| GXF warnings `Using unregistered parameter` / `min_size ... deprecated` | From NVIDIA's graph files. |

## Troubleshooting

| Problem | Fix |
|---|---|
| `ros2: command not found` | You are in plain WSL: use a container terminal. |
| Robot: `--tf needs a Python with unitree_sdk2py` | Give one: `--sdk-python ~/miniconda3/envs/tv/bin/python` (or `inspire`, `~/motiontest/bin/python`). |
| Robot: `--tf needs the unitree_hg messages` | The old script: copy the new files (`robot_tf_sdk.patch`, then the `scp` at the top). |
| Robot: `No waist angles yet` every 5 s | The SDK reader gets no `rt/lowstate` on `eth0`. Robot fully on (not only the PC)? Test on the robot: `~/motiontest/bin/python ~/g1_waist_reader.py` must print three numbers per line (Ctrl+C). If it prints an error, try another `--sdk-python`. |
| Robot: `unrecognized option --tf` / old messages | The new `start_head_camera.sh` is not on the robot: the `scp` at the top. |
| Window says `pelvis: no TF` | `tf2_echo pelvis d435_link` (step 5). Nothing → the robot TF does not arrive: robot terminal says `lowstate ... Hz`? Camera images arrive but TF not → `ros2 daemon stop`, then try again. |
| `tf2_echo pelvis d435_link` works, but not `... head_camera_color_optical_frame` | The camera mount comes from the perception launch: is step 4 running? |
| TF arrives but jumps between two values | Someone also runs a robot model (`run_g1.sh` / `observe_g1_tf.sh`): stop `--tf` (or theirs). |
| No camera topics over WiFi | `unset CYCLONEDDS_URI` on the robot before the script; `--laptop 192.168.123.184` correct; `ping 192.168.123.213` from WSL. |
| `ssh unitree@192.168.123.213` does not work | Laptop on the team WiFi (`.184`)? Robot on? |
| Robot: `Device or resource busy` | `sudo systemctl stop teleimager.service`. |
| A terminal shows nothing / the program seems stuck, title says "Select"/"Mark" | Copy/select mode: press `Esc` (or `q`) in that tab. |
| OpenCV window does not appear | Test in WSL: `xeyes` (from `sudo apt install x11-apps`). No window → PowerShell `wsl --shutdown`, open WSL again, start everything again. Window appears → recreate the perception container (`up -d --force-recreate perception`). |
