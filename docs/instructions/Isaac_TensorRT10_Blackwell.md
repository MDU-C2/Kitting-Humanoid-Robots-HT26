# Isaac helper on an RTX 50xx (Blackwell): image with TensorRT 10.13

The Isaac helper image `karis/isaac_fp:3.2` (Isaac ROS 3.2) has **TensorRT 10.3**, which cannot
build engines for Blackwell GPUs (RTX 5090 on the Eskilstuna PC):

```
IBuilder::buildSerializedNetwork: Error Code 1: Internal Error (Unsupported SM: 0xc00)
```

`karis/isaac_fp:3.2-trt10.13` is the same image with the TensorRT runtime upgraded to **10.13.3.9**
(CUDA stays 12.6). Tested: 10.13 builds engines on the RTX 5090, and the helper registers and
tracks with it on the laptop (RTX 3060).

| | Image | Engines (`/models`) |
|---|---|---|
| Laptop, as before | `karis/isaac_fp:3.2` | `docker/isaac_fp/models/` (TensorRT 10.3) |
| Laptop, test of the new image | `karis/isaac_fp:3.2-trt10.13` | `docker/isaac_fp/models_trt10/` (TensorRT 10.13) |
| PC (RTX 5090) | `karis/isaac_fp:3.2-trt10.13` | `docker/isaac_fp/models/` on the PC (TensorRT 10.13) |

Engines only work with the TensorRT version and the GPU they were built for: never copy `.plan`
files between the two images or between computers. Only the `.onnx` files may be copied.

## Network interface: leave it empty

With the VPN on, WSL renames its interfaces (the robot network has been both `eth1` and `eth2`).
Pin it by address, not by name: leave `KARIS_NET_IF` empty and `karis_ros_env.sh` picks the
interface with `192.168.123.x` itself. In a container terminal:

```bash
export KARIS_NET_IF= && source $KARIS_WS/karis_ros_env.sh     # must say CycloneDDS on ethX (192.168.123.184/24)
```

Permanently (WSL terminal, then recreate the containers): `bash docker/setup_env.sh --if ""`.
On the PC (no robot network) it is empty already: the camera arrives over the Zenoh bridge on the PC.

---

## 1. Build the image (laptop, once)

WSL terminal. Downloads about 1.25 GB from NVIDIA's CUDA repo; the image gets about 5 GB larger.

```bash
cd ~/Kitting-Humanoid-Robots-HT26
docker build --network host -f docker/isaac_fp/Dockerfile.trt10 -t karis/isaac_fp:3.2-trt10.13 docker/isaac_fp
docker run --rm --entrypoint /usr/src/tensorrt/bin/trtexec karis/isaac_fp:3.2-trt10.13 --version   # TensorRT v101303
```

## 2. Test on the laptop

Prerequisite: the robot camera runs (step 2 in `Perception_startup.md`). Only one Isaac helper at
a time: Ctrl+C the usual one first (its container can stay).

WSL terminal, the test container (same settings as `karis_isaac_fp`, other image and engine folder):

```bash
R=$HOME/Kitting-Humanoid-Robots-HT26
mkdir -p $R/docker/isaac_fp/models_trt10 && cp -n $R/docker/isaac_fp/models/*.onnx $R/docker/isaac_fp/models_trt10/
docker run -dit --name karis_isaac_fp_trt10 --gpus all --network host --ipc host \
  --device /dev/dxg -v /usr/lib/wsl:/usr/lib/wsl:ro -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $R:$R -v $R/docker/isaac_fp/models_trt10:/models -w $R/ros2_ws \
  -e KARIS_REPO=$R -e KARIS_WS=$R/ros2_ws -e KARIS_DIR=$R/local_data -e KARIS_ROLE=isaac \
  -e KARIS_ISAAC_MODELS=/models -e ROS_DOMAIN_ID=0 -e KARIS_NET_IF= \
  -e KARIS_DDS_PEERS=192.168.123.213 -e DISPLAY=:0 -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e GIT_CONFIG_COUNT=1 -e GIT_CONFIG_KEY_0=safe.directory -e GIT_CONFIG_VALUE_0='*' \
  karis/isaac_fp:3.2-trt10.13 bash
```

(It exists already: `docker start karis_isaac_fp_trt10`.)

Isaac-TRT10 terminal (new tab, wsl): `docker exec -it karis_isaac_fp_trt10 bash`, then:

```bash
export KARIS_NET_IF= && source $KARIS_WS/karis_ros_env.sh
bash $KARIS_REPO/docker/isaac_fp/prepare_models.sh      # first time: 3 engines, a few minutes
ls /models/*.plan                                        # refine, refine_batch1, score
ros2 launch karis_isaac_fp isaac_fp.launch.py classes:="Ratblock Chamfer" refine_iterations:=3 max_depth:=2
```

Use the repo's `prepare_models.sh`, not `/opt/karis/prepare_models.sh`: the copy in the image is
older and does not build the batch-1 engine (needed for `lean:=true`).

Perception terminal (usual container):

```bash
export KARIS_NET_IF= && source $KARIS_WS/karis_ros_env.sh
ros2 launch karis_bringup perception.launch.py model:=rfdetr_seg_small.pth show:=true confidence:=0.75
```

Works when the window shows the Ratblock with box and axes, `ros2 topic echo --no-daemon --once /karis/object_poses`
gives `mode: track` with `fit_iou` about 0.5-0.7, and the summary at Ctrl+C looks like the old image's.

Back to the usual helper: Ctrl+C in Isaac-TRT10, `docker stop karis_isaac_fp_trt10`, then the
usual helper (with `export KARIS_NET_IF=` first).

## 3. Disk space on the PC

The image is about 60 GB. Check on the PC (`ssh.exe xrs01@172.17.10.215` from WSL, see below):

```bash
df -h /                 # Avail
docker system df        # Build Cache: RECLAIMABLE
```

Make room only with the PC owner's OK; the PC is shared:

```bash
docker builder prune -f                                # unused build cache (about 26 GB in Nov 2026)
pip cache purge; conda clean -a -y                     # pip/conda download caches
```

Never `docker system prune -a` or `docker image prune -a`: it deletes other people's images.
Aim for **at least 85 GB free**: Docker on the PC stores an image both packed and unpacked
(`docker images` there shows `DISK USAGE` and `CONTENT SIZE`).

## 4. Copy the image to the PC

From **WSL on the laptop**. Use `ssh.exe` (Windows' ssh), not `ssh`: in WSL, `172.17.x.x` goes to
Docker's `docker0` network instead of the VPN ("No route to host").

The image is streamed straight into Docker on the PC, without a file on the PC's full disk
(about 20-25 GB over the VPN; if the VPN drops, start it again):

```bash
docker save karis/isaac_fp:3.2-trt10.13 | gzip -1 | ssh.exe xrs01@172.17.10.215 docker load
```

Check on the PC: `docker images karis/isaac_fp` shows `3.2-trt10.13`.

## 5. Start on the PC

On the PC, in the repo (`~/perception_pipeline`):

```bash
cd ~/perception_pipeline && git pull --no-rebase
grep -q '^ISAAC_IMAGE=' docker/.env || echo 'ISAAC_IMAGE=karis/isaac_fp:3.2-trt10.13' >> docker/.env
mkdir -p docker/isaac_fp/models
docker compose -f docker/compose.yaml --profile isaac up -d isaac_fp      # no compose.wsl.yaml on the PC
docker compose -f docker/compose.yaml exec isaac_fp bash
```

Inside (prompt `root@XRS01-PC01:.../ros2_ws#`):

```bash
bash $KARIS_WS/build.sh && source $KARIS_WS/karis_ros_env.sh
bash $KARIS_REPO/docker/isaac_fp/prepare_models.sh      # downloads the 2 ONNX files from NVIDIA, builds 3 engines for the 5090
ros2 launch karis_isaac_fp isaac_fp.launch.py refine_iterations:=3 max_depth:=2
```

With 32 GB on the RTX 5090 all classes fit: no `classes:=` and no `lean:=true` needed.

Then the perception in the PC's perception container, as usual
(`ros2 launch karis_bringup perception.launch.py model:=rfdetr_seg_small.pth confidence:=0.75`).
No `show:=true` over ssh (no display); check with `ros2 topic hz /karis/object_poses`.

## Messages you can ignore

| Message | Why |
|---|---|
| GXF warnings `Using unregistered parameter` | From NVIDIA's graph files, same as with 10.3. |
| `CycloneDDS on none on 192.168.123.x found` on the PC | Expected: the camera comes from the Zenoh bridge on the PC itself. |

## Troubleshooting

| Problem | Fix |
|---|---|
| `Unsupported SM: 0xc00` | The old image (TensorRT 10.3) on a Blackwell GPU: use `3.2-trt10.13` (`ISAAC_IMAGE` in `docker/.env`, then `up -d --force-recreate isaac_fp`). |
| Helper: `deserialize` / `engine ... incompatible` | Engines from another TensorRT version or GPU in `/models`: `rm /models/*.plan`, run `prepare_models.sh` again. |
| `prepare_models.sh`: `CUDA driver version is insufficient` (laptop) | Non-interactive shell in WSL: use `docker exec -it ... bash`. |
| `CycloneDDS on ethX (172.16.x.x)` or another non-192.168.123 address | Pinned to the VPN interface: `export KARIS_NET_IF=` and source again. |
| `ros2 topic echo` fails with `!rclpy.ok()` | The ros2 daemon still has the old interface: `ros2 daemon stop`, or `--no-daemon`. |
| `ssh: connect to host 172.17.10.215 ... No route to host` | Use `ssh.exe` from WSL (see step 4). |
| `docker load` stops / `unexpected EOF` | The VPN dropped during the copy: run the step 4 command again. |
| `no space left on device` on the PC | Step 3: free more space first. |
