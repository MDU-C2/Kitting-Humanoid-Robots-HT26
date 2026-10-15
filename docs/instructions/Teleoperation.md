# G1 Teleoperation – Quick Start (xr_teleoperate)

Teleoperate the Unitree G1 with Inspire FTP hands using a Meta Quest 3. Both the G1 and the PC needs to be in the same network for teleoperation to work. Go to [Robot_connection](Robot_connection.md) for instructions to connect the G1 to a wireless network.

## Setup at a glance

| Machine | Address | Role |
|---|---|---|
| PC2 (robot, `unitree@ubuntu`) | `192.168.123.164` | Camera server (teleimager) + hand driver |
| Ubuntu PC (`marc@Marc`) | cable `enx50a03000b0dc` = `192.168.123.222`, WiFi `wlp0s20f3` = `192.168.123.181` | Runs teleop |
| Inspire FTP hands | `.210` (left), `.211` (right) | Only reachable from PC2 |
| Quest 3 | `192.168.123.136` (WiFi) | Headset, Meta Quest Browser |

Conda envs: PC2 → `teleimager`, `inspire`. Ubuntu PC → `tv`.

---

## Start a session

**1. PC2 – camera server (terminal A)**
Make sure the ROS camera driver (`start_head_camera.sh`) is **not** running – only one program can open the camera.
```bash
ssh unitree@192.168.123.164
sudo systemctl stop teleimager.service 
conda activate teleimager
cd ~/teleimager
teleimager-server --rs
```
Wait for `Running... Press Ctrl+C to exit.`

**2. PC2 – hand driver (terminal B)**
```bash
ssh unitree@192.168.123.164
sudo ip route replace 192.168.123.210/32 dev eth0 src 192.168.123.164
sudo ip route replace 192.168.123.211/32 dev eth0 src 192.168.123.164
conda activate inspire
cd ~/inspire_hand_ws
python inspire_hand_sdk/example/Headless_driver_double.py
```
Should print ~20 Hz for both hands, no "Retrying".

**3. Ubuntu PC – route to the Quest (once per boot)**
```bash
sudo ip route replace 192.168.123.136/32 dev wlp0s20f3 src 192.168.123.181
```
Keep the Ubuntu PC's WiFi **on** (the Quest is only reachable over WiFi).

**4. Ubuntu PC – teleop (terminal C)**
```bash
conda activate tv
cd ~/xr_teleoperate/teleop
python teleop_hand_and_arm.py --arm=G1_29 --ee=inspire_ftp --input-mode=hand \
  --network-interface=enx50a03000b0dc --img-server-ip=192.168.123.164
```
Wait for `Press [r] to start...` – **do not press r yet.**
Teleop reads the keyboard system-wide: don't type in other windows while it runs.

**5. Quest 3 (Meta Quest Browser)**
1. `https://192.168.123.164:60001` → Advanced → Proceed → **start** (camera check, first time).
2. `https://192.168.123.181:8012` → Advanced → Proceed (accept certificate, first time).
3. `https://192.168.123.181:8012/?ws=wss://192.168.123.181:8012` → **Virtual Reality**.
4. Put the controllers down (hand tracking).

**6. Start**
1. Terminal C shows `websocket is connected`.
2. Hold your arms in the robot's start pose (arms down, elbows slightly bent).
3. Press **r** in terminal C.

## Stop

1. Arms back to the start pose → press **q** in terminal C (arms return home within ~5 s).
2. Ctrl+C terminal B, then terminal A.
3. Restart the ROS camera driver if going back to perception.

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| Arms lift and don't follow after **r** | Quest not connected – look for `websocket is connected` before pressing r |
| Quest page reloads / can't reach `:8012` | Redo step 3 (Quest route); check `ip route get 192.168.123.136` says `dev wlp0s20f3`. If the Quest's IP changed, update it |
| Hand driver: `No route to host` / Retrying | Redo the two `ip route` lines in step 2 (lost on every reboot) |
| Black image in headset | teleimager not running, ROS camera still holding the camera, or redo Quest step 5.1 |
| `Failed to subscribe dds` | Wrong `--network-interface`, or robot not powered/reachable (`ping 192.168.123.164`) |
