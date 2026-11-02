# perception_scene_updater

Puts the parts that the KARIS perception sees into MoveIt's planning scene as boxes, in the robot
frame (`pelvis`), so the arm plans around them. Runs on the **motion laptop** next to
`move_group`. Uses **only standard ROS 2 / MoveIt messages**: no KARIS packages needed.

```
perception laptop (WiFi)                       motion laptop (cable + WiFi)
  /karis/markers  (MarkerArray, camera frame) ─▶ scene_updater ─/collision_object─▶ move_group / RViz
  TF d435_link → camera (camera mount)        ─▶ TF pelvis → … → d435_link (robot model, run_g1.sh)
  TF karis/<object_id> (each part)            ─▶ for your picking code: lookup_transform("pelvis", "karis/<id>")
```

## Install (motion laptop)

Copy the folder `perception_scene_updater/` into `ros2_ws/src/`, then in the MoveIt container:

```bash
cd /workspace
colcon build --symlink-install --packages-select perception_scene_updater
source install/setup.bash
```

## Run

With the MoveIt stack running (`run_g1.sh`, with ROS 2 also on WiFi:
`G1_ROS_EXTRA_INTERFACES=<wifi-if> G1_ROS_PEERS=<perception laptop IP>`), in a second shell of
the MoveIt container (same `CYCLONEDDS_URI` / interfaces as the stack):

```bash
ros2 run perception_scene_updater scene_updater
# with the table as an obstacle (measure the values first!):
ros2 run perception_scene_updater scene_updater --ros-args \
  -p add_table:=true -p table_top_z:=-0.06 -p table_center_xy:="[0.55, 0.0]" -p table_size:="[0.8, 1.2, 0.04]"
```

## Check

```bash
ros2 topic hz /karis/markers                                        # perception arrives (WiFi)
ros2 run tf2_ros tf2_echo pelvis head_camera_color_optical_frame    # ~[0.054, 0.033, 0.474]
ros2 topic echo /collision_object --once                            # a box with id e.g. ratblock_chamfer_2, frame pelvis
```

RViz: **MotionPlanning → Scene Geometry → Show Scene Geometry**; the parts appear as green
boxes. Also **Add → By topic → /karis/markers** to see the perception's own boxes and names.

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `base_frame` | `pelvis` | MoveIt's planning frame |
| `rate` | `2.0` | scene updates per second |
| `remove_after_s` | `3.0` | remove a part not seen for this long |
| `padding` | `0.005` | added to each box side (m) |
| `classes` | `[]` (all) | only object_ids starting with these, e.g. `["ratblock_chamfer"]` |
| `add_table` | `false` | add the table as a box |
| `table_top_z` | `-0.06` | table top height in `base_frame` (m) — measure it |
| `table_size` | `[0.8, 1.2, 0.04]` | x (depth), y (width), z (thickness) in m |
| `table_center_xy` | `[0.55, 0.0]` | table centre in `base_frame` (m) |

The part being picked: publish its id on `/karis/task/active_object` (`std_msgs/String`)
before planning to it, so it is taken out of the scene; publish `""` afterwards.

## Plan / TODO for the motion team

See the long comment at the top of `scene_updater_node.py` (scene: tray as obstacle,
attaching the part after grasping; picking: target from TF `karis/<id>`, measured grasp
offset and orientation, left arm by mirroring, hand yaw, placing in a slot).

## Tests (no ROS needed)

```bash
cd perception_scene_updater && python3 -m pytest -q test
```
