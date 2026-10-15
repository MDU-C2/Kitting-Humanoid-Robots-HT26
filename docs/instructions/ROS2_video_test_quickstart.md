# Quickstart: test the ROS 2 perception on a recorded video (.bag)

Plays a RealSense recording (`.bag`, colour + depth) as if it were the G1 head camera, and
runs the whole KARIS perception on it through ROS 2: YOLO detection and FoundationPose 6D
pose tracking with **Isaac ROS FoundationPose (TensorRT)** in the Isaac helper.

Unlike the photo test ([ROS2_image_test_quickstart.md](ROS2_image_test_quickstart.md)),
the scene moves, so this tests tracking for real: parts are registered once, then followed
from frame to frame.

This assumes the one-time setup in [ROS2_pipeline.md](ROS2_pipeline.md) is done (both
images built, `make_isaac_meshes`, `prepare_models.sh`, workspace built in both containers).

**Terminals:**

| | Where | Prompt |
|---|---|---|
| **A** | WSL itself (Windows: open "Ubuntu-24.04") | `you@LAPTOP...$` |
| **B** | Isaac container (only for Isaac) | `root@LAPTOP...#` |
| **C** | perception container | `root@LAPTOP...#` |

## A: prepare and start the containers

**A1.** Network buffers (skip if `%USERPROFILE%\.wslconfig` has the `kernelCommandLine` line):

```bash
sudo sysctl -w net.core.rmem_max=33554432 net.core.rmem_default=8388608
```

**A2.** Put the recording in `~/Kitting-Humanoid-Robots-HT26/local_data/videos` (the containers see `~/Kitting-Humanoid-Robots-HT26/local_data`):

```bash
mkdir -p ~/Kitting-Humanoid-Robots-HT26/local_data/videos
cp "/mnt/c/Users/<Windows user>/Downloads/test_video.bag" ~/Kitting-Humanoid-Robots-HT26/local_data/videos/
ls -lh ~/Kitting-Humanoid-Robots-HT26/local_data/videos
```

**A3.** Start the containers:

```bash
cd ~/Kitting-Humanoid-Robots-HT26
docker compose -f docker/compose.yaml -f docker/compose.wsl.yaml up -d perception
docker compose -f docker/compose.yaml -f docker/compose.wsl.yaml --profile isaac up -d isaac_fp
```

## B: Isaac helper

New WSL window:

```bash
cd ~/Kitting-Humanoid-Robots-HT26
docker compose -f docker/compose.yaml exec isaac_fp bash
ros2 launch karis_isaac_fp isaac_fp.launch.py classes:="Ratblock Chamfer"
```

Wait for `Isaac ROS FoundationPose bridge for ['Ratblock Chamfer'] ...` and two
`Node was started`. Leave it running.

On the 6 GB laptop GPU only one class fits (~1.5 GB per class). Classes the helper does not
serve get their position only. Class names: `Rund mutter`, `Ihalig bojd kvadrat`,
`Ratblock Chamfer` (comma-separated in `classes:=`).

## C: perception with the video

New WSL window:

```bash
cd ~/Kitting-Humanoid-Robots-HT26
docker compose -f docker/compose.yaml exec perception bash
```

**C1.** Only if the container was (re)created since you last did this: `lz4` (needed to
repair bags, see below; it is in the Dockerfile for the next image build):

```bash
python3 -m pip install lz4
```

**C2.** Start (one line):

```bash
cd $KARIS_DIR
ros2 launch karis_bringup perception.launch.py model:=YOLO_v8s.pt player:=$KARIS_DIR/videos/test_video.bag player_rate:=5 show:=true
```

Start B first: the pose estimator gets every pose from the Isaac helper.

## Speed: `player_rate`

`player_rate:=N` sends N frames per second (default `0.0` = the speed of the recording).
Perception always works on the newest frame and skips the ones it had no time for, like
with the live camera. Watch the status line in C, every ~10 s:

```
4.8 frames/s processed, 0 older frames skipped, 1 parts tracked, last: track 52 ms
```

- **older frames skipped > 0:** frames arrive faster than perception handles them. Lower
  `player_rate` (5 → 3 → 2) until it stays 0.
- **frames/s processed** with 0 skipped is about what this computer manages.
- **last: track .. ms** is one FoundationPose tracking step. If it is small (Isaac: 45–70 ms
  per part) but the video still lags, the time goes to YOLO (every 5 frames, `redetect_every:=`) or to
  registering parts (a few seconds when a part first appears).
- `show:=true` costs a little; leave it out for a pure timing run.

Expected: 1–2 parts at ~5–10 frames/s on the laptop.

## What you should see

- C: `Playing $KARIS_DIR/videos/test_video.bag ...`, `Camera 640x480: fx=...`, then
  the status line every ~10 s; at the end `End of the recording (... frames), again` and it
  starts over (the jump back is like a sudden scene change).
- B (Isaac): `first pose output from Isaac FoundationPose`.
- The OpenCV window: the video with boxes and axes on the parts.
- Poses on a topic (any container): `ros2 topic echo /karis/object_poses`.

## Bags that were not closed properly ("Bag unindexed")

If the recording program was killed or the file was copied while being written, the
index at the end of the bag is missing and RealSense cannot open it. The camera player
repairs it by itself the first time:

```
test_video.bag was not closed properly (no index): repairing it into test_video.reindexed.bag ...
Reindexed test_video.bag: N chunks, M messages, K topics -> .../test_video.reindexed.bag
```

The original is left as it is. Once the repaired file plays, keep only that one (in A):

```bash
mv ~/Kitting-Humanoid-Robots-HT26/local_data/videos/test_video.reindexed.bag ~/Kitting-Humanoid-Robots-HT26/local_data/videos/test_video.bag
```

Repair without playing: `ros2 run karis_perception bag_reindex $KARIS_DIR/videos/test_video.bag` (in C).

## Stop

Ctrl+C in C and B. Then in A:

```bash
cd ~/Kitting-Humanoid-Robots-HT26
docker compose -f docker/compose.yaml --profile isaac down
```

## Troubleshooting

| Problem | Fix |
|---|---|
| `bash: your: No such file or directory` | A placeholder like `<your file>` was pasted: use the real file name. |
| `show:=true: command not found` / argument ignored | The command was split over two lines: paste it as one line. |
| `source '...' does not exist` | Check the path and the exact (case-sensitive) name: `ls ~/Kitting-Humanoid-Robots-HT26/local_data/videos`. |
| `Bag unindexed` in a traceback, without the "repairing it" line | The workspace has old code: in C, `bash $KARIS_WS/build.sh && source $KARIS_WS/karis_ros_env.sh`. |
| `this bag uses lz4 compression: pip install lz4` | In C: `python3 -m pip install lz4`. |
| Another `RuntimeError` from pyrealsense2 | The file is not a RealSense recording with colour and depth. A `ros2 bag record` folder (`.db3`/`.mcap`) does not work as `player:=`. |
| `/karis/fp/estimate_pose is not available` | The Isaac helper in B is not running or died at start: look at B. |
| NumPy error in B (`_ARRAY_API not found`) | Rebuild the Isaac layer: see "Check once" in [ROS2_image_test_quickstart.md](ROS2_image_test_quickstart.md). |
| `NMS time limit ... exceeded` once at the start | YOLO's first run on the GPU (warm-up). Ignore. |
| `No TF pelvis <- head_camera_color_optical_frame yet` | No robot model running; `/karis/object_poses` (camera frame) still works. |
| `unknown class '...' -> position only` | That class is not in the helper's `classes:=`. |
| OpenCV window empty or frozen, cannot be opened | The WSL display link is stuck: Windows PowerShell `wsl --shutdown`, open WSL again, start the containers again. Or watch `ros2 run rqt_image_view rqt_image_view /karis/viz/annotated/compressed`. |
