# local_data/: local data of this computer (not code)

The perception reads its trained models from here, and writes and reads its local files.
Everything in this folder except this README and `models/README.md` stays on each computer
(see `.gitignore`); the code is in `ros2_ws/src/`.

| Folder | What | Made by |
|---|---|---|
| `models/` | trained detectors: YOLO `.pt`, RF-DETR `.pth` | training |
| `meshes_isaac/` | meshes for the Isaac FoundationPose helper | `ros2 run karis_perception make_isaac_meshes` |
| `data/` | test photos (`rgb/`, `depth/`, `cam_K/`) for `player:=$KARIS_DIR/data` | copied in |
| `videos/` | RealSense `.bag` recordings for `player:=$KARIS_DIR/videos/<name>.bag` | copied in |
| `bags/` | ROS 2 bags (`ros2 bag record`) | you |

Inside the containers this folder is `$KARIS_DIR` (set by `docker/setup_env.sh`).
