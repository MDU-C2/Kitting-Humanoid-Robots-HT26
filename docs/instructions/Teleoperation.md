# G1 Teleoperation – Quick Start (xr_teleoperate)

Teleoperate the Unitree G1 with Inspire FTP hands using a Meta Quest 3. Both the G1 and the PC must be on the same network for teleoperation to work. Go to [Robot_connection](Robot_connection.md) for instructions to connect the G1 to a wireless network. Everything runs on the robot (PC2).

## Setup at a glance

| Device | Address | Role |
|---|---|---|
| PC2 (robot, `unitree@ubuntu`) | cable `eth0` = `192.168.123.164`, WiFi `wlan0` (VR-Wifi) = DHCP, e.g. `.213` | Runs camera server, hand driver **and teleop** |
| Motion computer | `192.168.123.161` | Robot's own controller (teleop talks to it over DDS) |
| Inspire FTP hands | `.210` (left), `.211` (right) | Only reachable over PC2's cable |
| Quest 3 | WiFi on VR-Wifi | Headset, Meta Quest Browser |
| Laptop | any | Only for SSH: `ssh unitree@192.168.123.164` |

Conda envs on PC2: `teleimager` (camera), `inspire` (hand driver), `tv` (teleop).

---

## Start a session

**0. Robot:** power on → damping → locked standing → **R1+X** (regular mode). Keep it in the gantry with slack.
Open **three SSH sessions** to PC2 (terminals A, B, C).

**1. Routes (once per boot, any terminal)**
PC2's WiFi and cable share `192.168.123.x` and WiFi has priority, so pin robot traffic to the cable:
```bash
sudo systemctl stop teleimager.service
sudo ip route replace 192.168.123.161/32 dev eth0 src 192.168.123.164
sudo ip route replace 192.168.123.210/32 dev eth0 src 192.168.123.164
sudo ip route replace 192.168.123.211/32 dev eth0 src 192.168.123.164
```

**2. Terminal A – camera server**
Make sure the ROS camera driver is not running and no other teleimager is running (`ps aux | grep teleimager`).
```bash
conda activate teleimager
cd ~/teleimager
teleimager-server --rs
```
Wait for `Running... Press Ctrl+C to exit.`

**3. Terminal B – hand driver**
```bash
conda activate inspire
cd ~/inspire_hand_ws
python inspire_hand_sdk/example/Headless_driver_double.py
```
Both hands should report ~20 Hz, no "Retrying".

**4. Terminal C – teleop**
```bash
conda activate tv
cd ~/xr_teleoperate/teleop
python teleop_hand_and_arm.py --arm=G1_29 --ee=inspire_ftp --input-mode=hand --network-interface=eth0 --img-server-ip=192.168.123.164 --motion --display-mode=pass-through
```
Wait for `Press [r] to start...` – **do not press r yet.**

**5. Quest 3 (Meta Quest Browser)**
1. First time on a new headset: `https://192.168.123.164:8012` → Advanced → Proceed (accept certificate).
2. `https://192.168.123.164:8012/?ws=wss://192.168.123.164:8012` → **Virtual Reality or Pass-through**.
3. Put the controllers down (hand tracking).

**6. Start**
1. Terminal C shows `websocket is connected`.
2. Hold your arms in the robot's start pose (arms down, elbows slightly bent).
3. Press **r** in terminal C.

## Stop

1. Arms back to the start pose → press **q** in terminal C (arms return home within ~5 s).
2. Ctrl+C terminal B, then terminal A.

---

## Options

| Flag | Effect |
|---|---|
| `--display-mode=immersive` | (default) robot camera fills your view |
| `--display-mode=ego` | room in passthrough + small robot-camera window |
| `--display-mode=pass-through` | room only, **no robot camera** |
| `--record` | record episodes: **s** start / **s** stop+save (data in `teleop/utils/data/`) |
| `--motion` | robot standing on its own balance controller (see below) |

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| Arms lift and don't follow after **r** | Quest not connected – wait for `websocket is connected` before pressing r |
| Quest page doesn't load | Teleop running? `ss -tlnp \| grep 8012` on PC2. Accept the certificate (step 5.1). Check router has no client/AP isolation |
| No camera image in headset | `--display-mode=pass-through` shows no camera. Otherwise check terminal A is running |
| teleimager: `set_xu ... Resource temporarily unavailable` | Another teleimager already holds the camera (`ps aux \| grep teleimager`, `systemctl status teleimager.service`) |
| teleimager: `Error updating frame` then stops | Retry; reset camera: `python -c "import pyrealsense2 as rs; [d.hardware_reset() for d in rs.context().devices]"` |
| Hand driver: `No route to host` / Retrying | Redo step 1 (routes are lost on every reboot) |
| `Failed to subscribe dds` | Routes missing (step 1), wrong `--network-interface` (must be `eth0` on PC2) |
| `teleimager-server: command not found` | `conda activate teleimager` first |
| Thumb doesn't bend enough | Edit thumb ranges in `~/xr_teleoperate/teleop/robot_control/robot_hand_inspire.py` (class `Inspire_Controller_FTP`, `idx == 4` max `0.5` → e.g. `0.3`) |

---

## Notes for whoever sets this up again

- **Network:** PC2 `wlan0` (VR-Wifi) has higher priority than `eth0` and both are on `192.168.123.x` – hence the routes in step 1. Moving the router to its own subnet (e.g. `192.168.50.x`) would remove the need for them.
- **PC2 `tv` env** (Python 3.10, conda pinocchio 3.1.0): installed `teleimager` client (`--no-deps` + pyyaml pyzmq), `televuer`, `params-proto<3`, `sshkeyboard`, `logging-mp`, `opencv-python==4.10.0.84`, `meshcat`, `unitree_sdk2_python`, `inspire_sdkpy` (`--no-deps`) + `pymodbus==3.6.9`.
  `dex-retargeting` installed with `--no-deps` + `nlopt<2.8` from **conda-forge** (pip build fails on the Jetson) + torch 2.3.0, pytransform3d, trimesh, anytree, lxml. Don't let pip install `pin` – it would replace conda pinocchio 3.1.0.
- **PC2 `inspire` env:** `inspire_sdkpy/__init__.py` patched so the PyQt GUI import is optional (PyQt5 can't be built on the Jetson). Original in `__init__.py.bak`. Keep `numpy<2`.
- `CYCLONEDDS_HOME=/home/unitree/cyclonedds/install` (in `~/.bashrc`).
- `teleimager.service` exists but is disabled – keep it that way, or it blocks the ROS perception camera on every boot.
- `numpy` must stay `<2` in all three envs.
- Upstream: https://github.com/unitreerobotics/xr_teleoperate
