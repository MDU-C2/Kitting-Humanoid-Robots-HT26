# Perception on the Eskilstuna PC (RTX 5090) over the Zenoh bridge

Runs the perception and the Isaac FoundationPose helper on the PC in Eskilstuna instead of the
laptop. The robot's camera and TF reach the PC over the existing Zenoh bridge (see
`~/G1_ZENOH_STREAMING_GUIDE.md` on the PC):

```
robot (Västerås) --> Windows laptop on the robot WiFi (start_zenoh.ps1) --VPN--> PC: zenoh bridge --> ROS 2 domain 0 on the PC
```

- **Part A** (once): get everything onto the PC.
- **Part B** (every time): start the camera path, the helper and the perception.

| What you check | Where |
|---|---|
| The camera arrives on the PC | `ros2 topic hz /g1/head_camera/color/camera_info` ≈ 15 Hz on the PC |
| The Isaac helper runs on the 5090 | `Isaac ROS FoundationPose bridge for [...]`, `Node was started` per node, no `Unsupported SM` |
| The parts get a 6D pose | `/karis/object_poses`: `mode: track`, `fit_iou` about 0.5-0.8 |

Good to know:
- The RTX 5090 (Blackwell) needs the Isaac image with TensorRT 10.13,
  `karis/isaac_fp:3.2-trt10.13` (the default in `docker/compose.yaml`). It is built on the laptop
  (`Isaac_TensorRT10_Blackwell.md`, step 1) and copied to the PC (A4).
- The bridge forwards only the camera, `/tf`, `/tf_static`, `/g1/joint_states` to the PC (and the
  arm action back). **`/karis/...` stays on the PC**: whatever uses the poses must run on the PC too.
- The robot's `start_cam_both.sh` serves **both** the laptop (`.184`) and the Zenoh bridge
  (`.171`) at once, so the same robot start works for `Perception_startup.md` and for this page.
  Only one camera program can run on the robot: do not start `start_head_camera.sh` next to it.
- The PC is shared: do not stop or remove other people's containers, images or tmux sessions.

## Terminals

| Terminal | Where | Prompt when ready |
|---|---|---|
| **WSL** | the laptop | `$USER@LAPTOP-PQVFOCEA:~/Kitting-Humanoid-Robots-HT26$` |
| **Zenoh-Win** | PowerShell on the Windows laptop in Västerås (robot WiFi + VPN) | `PS C:\...\zenoh>` |
| **Robot** | ssh to the G1 | `unitree@ubuntu:~$` |
| **PC** | ssh to the PC | `xrs01@XRS01-PC01:~$` |
| **Isaac** | the Isaac container on the PC | `root@XRS01-PC01:.../ros2_ws#` |
| **Perception** | the perception container on the PC | `root@XRS01-PC01:.../local_data#` |
| **Check** | a second perception terminal on the PC | `root@XRS01-PC01:.../local_data#` |

## Addresses

| | |
|---|---|
| PC (Eskilstuna, over VPN) | `172.17.10.215`, user `xrs01`, repo `~/perception_pipeline` (branch `perception-pipeline`) |
| Robot | `192.168.123.164` (eth0), `192.168.123.213` (wlan0) |
| Zenoh on the PC | `tcp/172.17.10.215:7447` |

---

# Part A: setup (once)

## A1. WSL: ssh to the PC

In WSL, `172.17.x.x` goes to Docker's own `docker0` network ("No route to host"). Route only the
PC through the VPN interface (the one with `172.16.50.x`):

```bash
ip -br addr | grep 172.16.50                                  # the VPN interface, e.g. eth2
sudo ip route add 172.17.10.215/32 dev eth2                   # until the next WSL restart
ssh.exe xrs01@172.17.10.215                                       # → PC terminal
```

For a plain terminal `ssh.exe xrs01@172.17.10.215` (Windows' ssh) also works, but not for
copying data (A4): it stalls on large data streams.

## A2. PC terminal: the repo and its settings

```bash
cd ~/perception_pipeline 2>/dev/null || git clone -b perception-pipeline https://github.com/MDU-C2/Kitting-Humanoid-Robots-HT26.git ~/perception_pipeline
cd ~/perception_pipeline && git pull --no-rebase
bash docker/setup_env.sh                                       # writes docker/.env (paths, domain 0)
grep -E '^(KARIS_REPO|KARIS_DIR|ROS_DOMAIN_ID|KARIS_NET_IF|ISAAC_IMAGE)=' docker/.env
```

`ISAAC_IMAGE` must be missing or `karis/isaac_fp:3.2-trt10.13` (the default). `KARIS_NET_IF` must be empty on the PC: it is not on the robot network, the camera comes from the
Zenoh bridge on the PC itself.

## A3. PC terminal: disk space

```bash
df -h / | tail -1                     # Avail: need about 60 GB for the Isaac image
docker images                         # karis/perception:humble there? (A5)
docker system df                      # Build Cache: RECLAIMABLE
```

If there is not enough room, free it only with the PC owner's OK:

```bash
docker builder prune -f               # unused Docker build cache
pip cache purge; conda clean -a -y    # pip/conda download caches
```

Never `docker system prune -a` / `docker image prune -a` (deletes other people's images), and never
`~/.cache/huggingface/token` or `~/.cache/huggingface/lerobot/` (login and recorded datasets).
Other caches (`~/.cache/huggingface/hub/models--...`, `~/.cache/ov`) are downloaded or rebuilt
again when needed: ask whoever uses them first.

## A4. WSL: copy the Isaac image to the PC

The image must exist on the laptop (`Isaac_TensorRT10_Blackwell.md`, step 1). It is streamed
straight into Docker on the PC, no file on the PC's disk: 20 GB over the VPN, about 25 min at
15 MB/s. Needs the route from A1 (Linux `ssh`, not `ssh.exe`):

```bash
docker save karis/isaac_fp:3.2-trt10.13 | gzip -1 | ssh xrs01@172.17.10.215 docker load
```

Password once, then no output until `Loaded image: karis/isaac_fp:3.2-trt10.13`. Keep the laptop
awake. If the VPN drops (`Broken pipe`, `unexpected EOF`), run it again. The camera stream shares
the link: expect fewer frames/s meanwhile.

Check in the **PC** terminal: `docker images karis/isaac_fp` → `3.2-trt10.13`.

## A5. The perception image and the detector models

**PC** terminal: is `karis/perception:humble` there (A3)? If not, build it (30-60 min):

```bash
cd ~/perception_pipeline && docker compose -f docker/compose.yaml build perception
```

**WSL**: copy the detector models (about 0.7 GB):

```bash
cd ~/Kitting-Humanoid-Robots-HT26/local_data/models
scp rfdetr_seg_nano.pth rfdetr_seg_small.pth rfdetr_seg_medium.pth xrs01@172.17.10.215:perception_pipeline/local_data/models/
```

Use RF-DETR (`.pth`) on the PC: the perception image's own PyTorch is too old for the 5090, so YOLO
`.pt` models do not run there; RF-DETR has its own, newer PyTorch.

## A6. PC: containers, meshes and engines

**PC** terminal:

```bash
cd ~/perception_pipeline
mkdir -p docker/isaac_fp/models
docker compose -f docker/compose.yaml up -d perception
docker compose -f docker/compose.yaml --profile isaac up -d isaac_fp      # no compose.wsl.yaml on the PC
docker ps --format '{{.Names}}  {{.Image}}'                               # karis_isaac_fp  karis/isaac_fp:3.2-trt10.13
```

**Perception** terminal (`docker compose -f docker/compose.yaml exec perception bash`):

```bash
bash $KARIS_WS/build.sh && source $KARIS_WS/karis_ros_env.sh
ros2 run karis_perception make_isaac_meshes          # meshes for the Isaac helper → local_data/meshes_isaac/
```

**Isaac** terminal (`docker compose -f docker/compose.yaml exec isaac_fp bash`):

```bash
bash $KARIS_WS/build.sh && source $KARIS_WS/karis_ros_env.sh
bash $KARIS_REPO/docker/isaac_fp/prepare_models.sh   # downloads the 2 ONNX files (~130 MB), builds 3 engines for the 5090
ls /models/*.plan                                    # refine, refine_batch1, score
```

`prepare_models.sh` must print no `Unsupported SM`; if it does, the container runs the old image
(see Troubleshooting). Use the repo's `prepare_models.sh`, not the older copy in `/opt/karis/`.

After a `git pull` with code changes: `build.sh` again in both containers. After a change of
`classes.yaml` or the meshes: `make_isaac_meshes` again.

---

# Part B: run (every time)

## B1. PC terminal: the Zenoh bridge

```bash
tmux ls                                                       # zenoh_eskilstuna: ... = it runs, go to B2
```

If `zenoh_eskilstuna` is missing, start only the bridge:

```bash
tmux new-session -d -s zenoh_eskilstuna "export ROS_DISTRO=humble; zenoh-bridge-ros2dds -c ~/zenoh_ws/zenoh_eskilstuna.json5; bash"
```

(`~/start_g1_remote.sh` also starts it, but it also restarts the `g1_control` container and a web
stream: not needed for the perception.)

## B2. Zenoh-Win terminal: the robot side

On the Windows laptop in Västerås that has the Zenoh folder (in the guide `C:\Users\gusta\zenoh`),
with the **VPN on** and the laptop on the robot WiFi. PowerShell, not WSL:

```powershell
cd C:\Users\gusta\zenoh
.\start_zenoh.ps1
```

Leave it running.

## B3. Robot terminal: the camera

```bash
ssh unitree@192.168.123.213                    # the robot over WiFi (or .164 over its cable)
```

Password, then at the menu type `1` (Foxy). On the robot:

```bash
bash ~/start_cam_both.sh
```

`start_cam_both.sh` stops old camera/TF processes, sends to the laptop (`.184`) and the Windows
Zenoh laptop (`.171`) over `wlan0`, starts `/g1/joint_states` (tmux `g1_joints`) and runs
`~/start_head_camera.sh ... --tf` (camera + robot model TF `pelvis`, which the bridge forwards to
the PC). It must print `Publishing 640x480 at 15 Hz ...` and every 10 s `15.0 frames/s`.
Not while the motion team's robot model runs (two TF sources): see "Other way" in
`Perception_startup.md`. Leave it running.

## B4. PC terminal: does the camera arrive?

```bash
docker run --rm --network host karis/perception:humble bash -c 'source /opt/ros/humble/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0; timeout 8 ros2 topic list | grep -E "g1|tf"; timeout 6 ros2 topic hz /g1/head_camera/color/camera_info 2>&1 | grep average'
```

Must list the three `/g1/head_camera/...` topics, `/tf`, `/tf_static`, and `average rate: ~15`.

## B5. Isaac terminal: the Isaac helper

PC terminal:

```bash
cd ~/perception_pipeline
docker compose -f docker/compose.yaml --profile isaac up -d isaac_fp       # if not running (docker ps)
docker compose -f docker/compose.yaml exec isaac_fp bash
```

Inside:

```bash
source $KARIS_WS/karis_ros_env.sh
ros2 launch karis_isaac_fp isaac_fp.launch.py refine_iterations:=5 max_depth:=2
```

All classes fit on the 5090: no `classes:=`, no `lean:=true`. Wait for
`Isaac ROS FoundationPose bridge for ['Ihalig bojd kvadrat', 'Ratblock Chamfer', 'Rund mutter', 'bricka'] ...`
and `Node was started` for each of the 8 nodes. Leave it running.

## B6. Perception terminal: the perception

PC terminal:

```bash
cd ~/perception_pipeline
docker compose -f docker/compose.yaml up -d perception                    # if not running
docker compose -f docker/compose.yaml exec perception bash
```

Inside (the first line says `CycloneDDS on none on 192.168.123.x found`: expected on the PC):

```bash
source $KARIS_WS/karis_ros_env.sh
ros2 launch karis_bringup perception.launch.py model:=rfdetr_seg_small.pth confidence:=0.5 web_port:=8081
```

No `show:=true` over ssh (no display on your screen). Instead, **watch it in a browser** on any
computer on the VPN: **http://172.17.10.215:8081/** shows the annotated image (boxes, axes, poses),
the detector's masks (class colour, class and confidence; updated on detection frames, every 5th
frame: `redetect_every:=1` in the launch for every frame) and the raw camera. (Port 8080 is the PC's other web stream, `stream_live.py`.) The image is only
drawn while a browser shows it. Anyone on the VPN who knows the address can watch.

## B7. Check terminal

PC terminal: `docker compose -f docker/compose.yaml exec perception bash`, then:

```bash
ros2 topic hz /karis/object_poses                                            # Ctrl+C
ros2 topic echo --no-daemon --once /karis/object_poses | grep -E "object_id|mode:|fit_iou|processing_ms"
ros2 topic echo --no-daemon --once /karis/object_poses_base | grep -E "frame_id|object_id"   # pelvis, from the robot's TF
```

## B8. Stop

1. Ctrl+C in **Perception**, **Isaac** and **Robot**; Ctrl+C or close **Zenoh-Win** if nobody else uses the stream.
2. Leave the PC's `zenoh_eskilstuna` tmux session running (others use it).
3. Optional, PC terminal (frees the GPU for others): `docker compose -f docker/compose.yaml --profile isaac stop isaac_fp`.

## Messages you can ignore

| Message | Why |
|---|---|
| `CycloneDDS on none on 192.168.123.x found: DDS picks one` | The PC is not on the robot network; the camera comes from the bridge on the PC itself. |
| `small network receive buffers` | A note about the PC's kernel settings; only matters if frames get dropped. |
| `TRT WARNING: The logger passed into createInferRuntime differs ...` | Several TensorRT engines in one process share one logger. |
| GXF `Using unregistered parameter` / `'min_size' ... deprecated` | From NVIDIA's graph files. |
| `sm_120 is not compatible with the current PyTorch installation` | The perception image's own PyTorch; RF-DETR uses its own, newer one. Only YOLO `.pt` is affected. |
| In the first ~minute after starting the helper: `timeout after 10.0 s (register)` a few times, then `Isaac output stamped -50000 ms ... dropped` | The first registration takes ~50 s (CUDA code prepared for the 5090 once per helper start). Afterwards a registration takes ~0.2 s. |

## Troubleshooting

| Problem | Fix |
|---|---|
| `ssh: connect to host 172.17.10.215 ... No route to host` from WSL | A1: the route, or `ssh.exe`. |
| A4 sends nothing (stuck at 0 MB) | Waiting for the password, or `ssh.exe` was used: Linux `ssh` with the route from A1. |
| `docker load` stops / `unexpected EOF` | The VPN dropped during the copy: run A4 again. |
| `no space left on device` on the PC | A3: free more space first. |
| `Unsupported SM: 0xc00` in the Isaac terminal | The old image: remove an `ISAAC_IMAGE=karis/isaac_fp:3.2` line from `docker/.env` (or `git pull` for the new default), then `docker compose -f docker/compose.yaml --profile isaac up -d --force-recreate isaac_fp`. |
| Helper: `deserialize` / `engine ... incompatible` | Engines from another TensorRT version or GPU in `/models`: `rm /models/*.plan`, run `prepare_models.sh` again. |
| B4 shows no `/g1/...` topics | Check backwards: `tmux ls` on the PC (bridge), `start_zenoh.ps1` on the Windows laptop (VPN on?), `start_cam_both.sh` on the robot. |
| Topics listed but no rate | The robot script runs but sends nothing: robot terminal for errors; `Device or resource busy` → another camera program runs (`start_head_camera.sh`, `teleimager.service`). |
| `could not connect to display :0` → `process has died` | `show:=true` over ssh: there is no screen. Start without `show:=true`, with `web_port:=8081`, and use the browser. |
| Browser: `http://172.17.10.215:8081/` does not load | Perception started without `web_port:=8081`? VPN on? `curl -s localhost:8081/status` on the PC works but not from outside → the PC's firewall blocks 8081 (ask the PC owner: `sudo ufw allow 8081/tcp`). |
| Browser: page loads, image stays black / "no image yet" | The perception gets no camera (B4) or has not processed a frame yet. `http://172.17.10.215:8081/status` shows frames/s per topic. |
| `/karis/fp/estimate_pose is not available` in the perception | The Isaac helper (B5) is not running yet; without it the perception publishes nothing. |
| `unknown class '...'` | The helper was started with `classes:=` without that class. |
| `ros2 topic echo` fails with `!rclpy.ok()` | `ros2 daemon stop`, or use `--no-daemon`. |
| Poses arrive slowly / gaps | The camera crosses WiFi + VPN; large transfers on the same link slow it down. |
| Nobody in Västerås sees `/karis/...` | Expected: the bridge's allow list only forwards the camera, TF and joints. Adding topics is a change in the bridge config (`~/zenoh_ws/zenoh_eskilstuna.json5`): ask its owner. |
