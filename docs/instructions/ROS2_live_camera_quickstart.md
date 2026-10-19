# Quickstart: run the ROS 2 perception on the live G1 head camera

The G1 streams its head camera (D435i) as ROS 2 topics; the laptop runs YOLO and
FoundationPose on them and publishes the parts' 6D poses. Same perception as the photo and
video tests ([ROS2_image_test_quickstart.md](ROS2_image_test_quickstart.md),
[ROS2_video_test_quickstart.md](ROS2_video_test_quickstart.md)), but without `player:=`:
the camera topics come from the robot.

```
G1 (Foxy)                          laptop, WSL + Docker (Humble)
g1_camera_ros2  ── cable/WiFi ──▶  karis_pose_estimator  ── /karis/fp/estimate_pose ──▶  Isaac helper
 /g1/head_camera/...  ~15 Hz        → /karis/object_poses, TF karis/<part>, overlay window
```

Tested on the G1 over WiFi (team router), 2026-10-09.

**Terminals:**

| | Where | Prompt |
|---|---|---|
| **A** | WSL (Windows: open "Ubuntu-24.04") | `you@LAPTOP...$` |
| **B** | SSH into the G1 | `unitree@ubuntu:~$` |
| **C** | Isaac container (only for Isaac) | `root@LAPTOP...#` |
| **D** | perception container | `root@LAPTOP...#` |

## Cable or WiFi

| | Cable | WiFi (team router) |
|---|---|---|
| Laptop address | set by hand, e.g. `192.168.123.222` (step 1) | from the router, e.g. `192.168.123.184` |
| Robot interface | `eth0` (`192.168.123.164`), the default | `wlan0` (e.g. `192.168.123.213`, from the router) |
| Camera start (B) | `bash start_head_camera.sh` | `bash start_head_camera.sh --wifi --laptop <laptop address>` |
| Settings (A) | `bash docker/setup_env.sh --cable` | `bash docker/setup_env.sh --robot <robot WiFi address>` |
| Frame rate | steady 15 Hz | lower and less steady; fine for testing |

**Why `--wifi` and `--laptop` over WiFi:** the robot has two addresses on `192.168.123.x`: `eth0`
(its internal cable network) and `wlan0` (the router's WiFi). The camera script ties ROS 2
to `eth0` by default, which the laptop cannot reach over WiFi: `--wifi` uses `wlan0`. WiFi often
blocks multicast, so both sides name each other directly: the laptop names the robot
(`setup_env.sh --robot`), the robot names the laptop (`--laptop`). Addresses (the router can
change them after a while): robot `ip -4 -o addr | grep wlan0` (B), laptop
`ip -4 addr | grep 192.168.123` (A).

## One-time setup

### 1. Laptop network (Windows)

- **Cable:** connect it to the G1 (directly or through its switch). *Settings → Network &
  internet → Ethernet → IP assignment → Edit → Manual*, IPv4: address `192.168.123.222`,
  mask `255.255.255.0`, no gateway.
- **WiFi:** join the team router's WiFi.

Set that network to **Private** (*… → your network → Network profile type → Private*).

### 2. WSL shares Windows' network (Windows)

`%USERPROFILE%\.wslconfig` (create it if missing):

```ini
[wsl2]
networkingMode=mirrored
kernelCommandLine = sysctl.net.core.rmem_max=33554432 sysctl.net.core.rmem_default=8388608
```

### 3. Firewall: let ROS 2 in (Windows)

ROS 2 finds the robot with multicast, then sends the data to the laptop's own address on
UDP ports 7400–7700. Windows blocks those by default. PowerShell **as administrator**:

```powershell
Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
New-NetFirewallHyperVRule -Name ROS2_DDS -DisplayName "ROS 2 DDS" -Direction Inbound -VMCreatorId '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -Protocol UDP -LocalPorts 7400-7700
New-NetFirewallRule -DisplayName "ROS 2 DDS (UDP 7400-7700)" -Direction Inbound -Protocol UDP -LocalPort 7400-7700 -Action Allow
wsl --shutdown
```

(If a rule already exists, PowerShell says so; that is fine.)

**Check (A),** after opening WSL again:

```bash
ip -4 addr | grep 192.168.123          # the laptop's address is listed
ping -c 3 192.168.123.164              # the robot answers (cable: <1 ms, WiFi: ~10-20 ms)
cat /proc/sys/net/core/rmem_max        # 33554432
```

### 4. Settings (A)

Writes `docker/.env` for this computer (paths from where the repo is; nothing to edit):

```bash
cd ~/Kitting-Humanoid-Robots-HT26
bash docker/setup_env.sh --robot 192.168.123.213   # WiFi: the robot's WiFi address
bash docker/setup_env.sh --cable                   # cable
```

### 5. Camera script on the robot (A, again after every change to `robot/`)

```bash
cd ~/Kitting-Humanoid-Robots-HT26
scp robot/g1_camera_ros2.py robot/start_head_camera.sh unitree@192.168.123.164:~/
```

## Every time

### Before you start

- Laptop on the team router's WiFi (or the cable).
- `ROS_DOMAIN_ID` must be 0 everywhere (the robot uses 0). If you set another one for a
  test, open new shells or `export ROS_DOMAIN_ID=0`.
- GPU memory not nearly full (`nvidia-smi` in A; close GPU-heavy Windows apps). A full GPU
  makes tracking many times slower.

### B: start the camera on the robot

```bash
ssh unitree@192.168.123.164          # at the menu: 1 = Foxy
sudo systemctl stop teleimager.service     # frees the camera (only one program can use it)
bash start_head_camera.sh --wifi --laptop 192.168.123.184   # WiFi (your laptop's address); cable: bash start_head_camera.sh
```

The first line must say `interface=wlan0` (WiFi) or `interface=eth0` (cable), then
`Publishing 640x480 at 15 Hz ...` and every 10 s `15.0 frames/s, 2.4 MB/s compressed`.
A first `Device or resource busy` warning followed by `Resetting the camera...` is fine.

Over WiFi, a lower rate is steadier: `bash start_head_camera.sh --wifi --laptop 192.168.123.184 --ros-args -p publish_rate:=10.0`.
All options: `bash start_head_camera.sh --help`.

### A: start the containers

```bash
cd ~/Kitting-Humanoid-Robots-HT26
docker compose -f docker/compose.yaml -f docker/compose.wsl.yaml up -d perception
docker compose -f docker/compose.yaml -f docker/compose.wsl.yaml --profile isaac up -d isaac_fp   # only for Isaac
```

After changing `docker/.env`, add `--force-recreate` so the containers pick it up.

### D: check that the camera arrives

```bash
cd ~/Kitting-Humanoid-Robots-HT26
docker compose -f docker/compose.yaml exec perception bash
```

The first line printed must name the robot network, e.g.
`CycloneDDS on eth1 (192.168.123.184/24)` (`none on 192.168.123.x found` = steps 1–2 not working). Then:

```bash
ros2 daemon stop
ros2 topic list --no-daemon --spin-time 5 | grep head_camera
ros2 topic hz /g1/head_camera/color/image_raw/compressed      # cable ~15 Hz, WiFi up to 15 Hz
```

No topics: see *Troubleshooting*.

### C: Isaac helper

```bash
cd ~/Kitting-Humanoid-Robots-HT26
docker compose -f docker/compose.yaml exec isaac_fp bash
ros2 launch karis_isaac_fp isaac_fp.launch.py classes:="Ratblock Chamfer"
```

Wait for `Isaac ROS FoundationPose bridge for ['Ratblock Chamfer'] ...` and two
`Node was started`. On the 6 GB laptop GPU one class fits; other classes get their position only.

### D: start the perception (no `player:=`)

```bash
cd $KARIS_DIR
ros2 launch karis_bringup perception.launch.py model:=YOLO_26s.pt show:=true
```

- Start C first: every pose comes from the Isaac helper.
- `model:=` is any file in `$KARIS_DIR/models` (`.pt`/`.engine` YOLO, `.pth`/`.trt`
  RF-DETR); without it, the model used last.
- **Detector every N frames:** 5 by default; for one run `redetect_every:=10`.
- **Detector confidence:** 0.6 by default (`confidence` in
  `ros2_ws/src/karis_bringup/config/perception.yaml`). For one run: add `confidence:=0.4`
  (or any 0–1). Higher = fewer false detections but parts are found less easily. The start
  line of the log shows the value in use: `... confidence 0.6, register 3 / track 1 ...`.

### What you should see

- D: `YOLO input size 640 px from the model's training size` (1280 for a model trained at 1280),
  `Camera 640x480: fx=... frame 'head_camera_color_optical_frame'`, then every ~10 s
  `X frames/s processed, Y older frames skipped, N parts tracked, last: track .. ms`.
- C: `first pose output from Isaac FoundationPose` when a part is registered.
- The OpenCV window: the live image with boxes and axes on the parts.
- Poses (any container): `ros2 topic echo /karis/object_poses --once`.

"Older frames skipped" is normal live: the camera sends ~15 frames/s and the perception
always works on the newest one.

### Optional: poses in the robot frame

`/karis/object_poses_base` (in `pelvis`) needs the robot model in the team's MoveIt container:
see [ROS2_robot_frame_quickstart.md](ROS2_robot_frame_quickstart.md).

### Stop

Ctrl+C in D, C and B. Then in A:

```bash
cd ~/Kitting-Humanoid-Robots-HT26
docker compose -f docker/compose.yaml --profile isaac down
```

On the robot, start the camera service again if the robot's own software needs it:
`sudo systemctl start teleimager.service`.

## Troubleshooting

Work from the top: each test narrows it down.

**1. Does the robot answer at all?** A: `ping -c 3 192.168.123.164`. No: cable / WiFi / step 1–2.

**2. Does multicast pass?** D: `ros2 multicast receive`; B (second SSH window):
`source /opt/ros/foxy/setup.bash && ros2 multicast send`. D must print
`Received from 192.168.123.x: 'Hello World!'`. **Note the sender address:** if it is the
robot's WiFi address (not `.164`), the robot talks to you over WiFi → start the camera with
`--wifi --laptop <laptop address>` and run `bash docker/setup_env.sh --robot <that address>` (A).

**3. Do direct messages reach the laptop (firewall)?**
D: `python3 -c "import socket;s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.bind(('0.0.0.0',7650));print('waiting');print(s.recvfrom(100))"`,
B: `echo hello | nc -u -w1 192.168.123.184 7650` (the laptop's address). D must print
`b'hello\n'`. If it keeps waiting: step 3 of the setup (firewall), `wsl --shutdown`,
recreate the containers.

**4. Topics?** D: `ros2 daemon stop; ros2 topic list --no-daemon --spin-time 5 | grep head_camera`.

| Problem | Fix |
|---|---|
| Container prints `CycloneDDS on none on 192.168.123.x found` | WSL does not see the laptop's robot-network address: setup steps 1–2. Open a **new** container shell afterwards. |
| No `/g1/head_camera/...` topics, multicast works | Firewall (test 3), wrong robot interface (`ROBOT_IF`, test 2), or the robot's address not set (`bash docker/setup_env.sh --robot <address>`, then `up -d --force-recreate` and new shells; check with `echo $CYCLONEDDS_URI`). |
| Still nothing | Start the camera with the laptop named: `bash start_head_camera.sh --wifi --laptop <laptop address>` (B); its first line must list `laptop peers: <address>`. In an SSH shell where you exported `CYCLONEDDS_URI` by hand: `unset CYCLONEDDS_URI` first. |
| Topics listed, but `ros2 topic hz` shows nothing | Firewall (test 3) or small buffers (`cat /proc/sys/net/core/rmem_max` = 33554432). |
| Low or uneven rate | WiFi: `--ros-args -p publish_rate:=10.0` on the robot, laptop close to the router (5 GHz), or use the cable. |
| B: `Device or resource busy` and no frames after the reset | `sudo systemctl stop teleimager.service`, or close other camera programs. |
| B: cannot import `pyrealsense2`, `cv2` or `rclpy` | `CONDA_ENV=<env with them> bash start_head_camera.sh` (e.g. `teleimager`). |
| `ros2: command not found` in A | ROS is only inside the containers: use D. |
| D: `Waiting for the camera: ...` forever | Same as "no topics" above. `ROS_DOMAIN_ID` must be 0 on both sides (default). |
| Too many / too few detections | `confidence:=…` (higher = fewer). |
| `/karis/fp/estimate_pose is not available` | The Isaac helper in C is not running or died at start. |
| Isaac times out (`... answers nothing`) | GPU memory full (`nvidia-smi` in A): fewer classes, or the helper on a bigger GPU. |
| `serdata.cpp ... invalid data size` / `string data is not null-terminated` | Another ROS 2 program on the network (domain 0, other ROS version) sends messages these cannot decode. Harmless if poses come through. For tests without the robot: `export ROS_DOMAIN_ID=42` in C and D. |
| Tracking much slower than ~70 ms per part | GPU memory full (`nvidia-smi`): close GPU apps in Windows, restart the containers. |
| `unknown class '...' (bridge has [...])` + one `frame skipped` | That class is not in the helper's `classes:=`: it gets its position only. Expected with one class. |
| `No TF pelvis <- head_camera_color_optical_frame yet` | The robot model is not running ([ROS2_robot_frame_quickstart.md](ROS2_robot_frame_quickstart.md)); `/karis/object_poses` (camera frame) still works. |
| OpenCV window empty or frozen, cannot be opened | The WSL display link is stuck: Windows PowerShell `wsl --shutdown`, open WSL again, start the containers again. Or watch `ros2 run rqt_image_view rqt_image_view /karis/viz/annotated/compressed`. |
