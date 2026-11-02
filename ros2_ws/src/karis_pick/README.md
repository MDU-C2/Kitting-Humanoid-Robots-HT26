# KARIS pick server, for the motion laptop

`pick_server` is the link between the KARIS perception (on the perception laptop, over WiFi) and
MoveIt (on the motion laptop). You send it a goal ("pick a Ratblock Chamfer"); it asks the
perception where that part is, in `pelvis`, computes the wrist targets and sends them to MoveIt.

Two ROS 2 packages, copy both into `ros2_ws/src/`:

| Package | What |
|---|---|
| `karis_interfaces` | the messages: `ObjectPose(Array)`, service `GetObjects`, action `PickObject` (also `EstimatePose`, unused here). Must be the **same** on both laptops (it is: copied from the perception branch). |
| `karis_task` | `pick_server` (and `scene_updater`, `pose_listener`; see below) |

## How it talks to the other nodes

`pick_server` subscribes to **no topics**. It asks the perception once per goal (a service call)
and talks to MoveIt through its action.

```
                 motion laptop                                        perception laptop (WiFi)
you ── action /karis/pick (PickObject) ──▶ pick_server
                                           │ 1. service call /karis/get_objects ────────────────▶ pose estimator
                                           │    (GetObjects: class_name/object_id,                 (answers with the part's
                                           │     target_frame "pelvis", fresh true) ◀────────────  pose + box in pelvis)
                                           │ 2. publishes /karis/task/active_object (String) ──▶ scene updater (part out of the scene)
                                           │ 3. publishes /karis/task/target_pose (PoseStamped) ─▶ RViz (arrow: wrist target)
                                           │ 4. action /move_action (moveit_msgs/MoveGroup) ────▶ move_group ─▶ arm controller
```

| Name | Type | Direction (pick_server) | Partner |
|---|---|---|---|
| `/karis/pick` | action `karis_interfaces/action/PickObject` | **server** | you / your task code |
| `/karis/get_objects` | service `karis_interfaces/srv/GetObjects` | **client** (once per goal, 5 s timeout) | the perception laptop |
| `/move_action` | action `moveit_msgs/action/MoveGroup` | **client** | `move_group` (run_g1.sh) |
| `/karis/task/target_pose` | topic `geometry_msgs/PoseStamped` | publishes | RViz |
| `/karis/task/active_object` | topic `std_msgs/String` | publishes | the scene updater |

The perception does the transform to `pelvis` itself (`target_frame: pelvis`), so **the
perception laptop needs the robot's TF** (`pelvis` → … → `d435_link`). With your `run_g1.sh`
running and ROS 2 also on WiFi (`G1_ROS_EXTRA_INTERFACES` / `G1_ROS_PEERS`), your
`robot_state_publisher` provides it over WiFi. (The robot camera's `--tf` option is then **off**:
only one robot model at a time.)

What the perception answers (`ObjectPose`, used by pick_server): `object_id`, `class_name`,
`has_orientation`, `box_pose` (centre + axes of the part's box, in `pelvis`), `box_size`. pick_server
uses the **box centre** + `wrist_offset_xyz` for the position and the fixed hand orientation
`grasp_quat_xyzw` (the part's own rotation is not used yet, see TODO). Of several matching parts it
takes the one closest to the robot (smallest `hypot(x, y)`).

## Build (in the MoveIt container)

```bash
cd /workspace
colcon build --symlink-install --packages-select karis_interfaces karis_task
source install/setup.bash
```

## Run

Same ROS settings as the stack (`ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, and a
`CYCLONEDDS_URI` with the cable **and** WiFi interfaces, like `run_g1.sh` with
`G1_ROS_EXTRA_INTERFACES`). Plan only first:

```bash
ros2 run karis_task pick_server --ros-args \
  -p execute:=false -p approach_only:=true -p z_reference:=centre \
  -p wrist_offset_xyz:="[-0.197, -0.036, 0.1127]" \
  -p grasp_quat_xyzw:="[-0.7087, 0.0589, 0.0797, 0.6985]" \
  -p approach_height:=0.10 -p velocity_scale:=0.1
```

(`wrist_offset_xyz` and `grasp_quat_xyzw` are the values from your
`move_right_arm_RealG1_kitting_arm_only.cpp`. It prints `/karis/pick ready (...)`.)

Goals (another shell, same settings):

```bash
# target only: computed and shown on /karis/task/target_pose, nothing planned
ros2 action send_goal /karis/pick karis_interfaces/action/PickObject \
  "{class_name: 'Ratblock Chamfer', dry_run: true}" --feedback
# plan in MoveIt (execute:=false) / plan + move (pick_server started with execute:=true)
ros2 action send_goal /karis/pick karis_interfaces/action/PickObject \
  "{class_name: 'Ratblock Chamfer', arm: 'right', dry_run: false}" --feedback
# one particular part (the name shown in the perception window)
ros2 action send_goal /karis/pick karis_interfaces/action/PickObject \
  "{object_id: 'ratblock_chamfer_2', dry_run: true}" --feedback
```

Goal fields: `object_id` or `class_name`, `place_slot` (slot class, "" = pick only), `dry_run`,
`arm` (`right` | `left` | `auto` = the part's side | "" = `planning_group`). Result: `success`,
`message`, `grasp_pose`, `place_pose`. Feedback: `state`, `progress`.

Parameters: `base_frame` (pelvis), `planning_group` (right_arm), `ee_link` (right_wrist_yaw_link),
`approach_height` (0.08), `grasp_offset_z` (0.0), `lift_height` (0.10), `grasp_rpy_deg` ([0, 90, 0]),
`grasp_quat_xyzw` ([0,0,0,0] = use grasp_rpy_deg), `wrist_offset_xyz` ([0,0,0]),
`z_reference` (top | centre), `approach_only` (false), `execute` (false = plan only),
`velocity_scale` (0.2), `planning_time` (5.0). Left arm: mirrored automatically (`arm: 'left'`).

MoveIt goal: position constraint (1 cm sphere) + orientation constraint (±0.15/0.15/0.3 rad) on
`ee_link`, in `base_frame`, starting from the current robot state.

## Checks before the first goal

```bash
ros2 service list | grep karis              # /karis/get_objects (from the perception laptop)
ros2 service call /karis/get_objects karis_interfaces/srv/GetObjects \
  "{class_name: '', object_id: '', target_frame: 'pelvis', fresh: false}"
ros2 action list | grep move_action         # /move_action (move_group)
```

The service call must return `success: true` and the parts with `header.frame_id: pelvis`.
`success: false` with "No TF" → the perception laptop does not get your robot model's TF.

## Scene updater: use one

- `perception_scene_updater` (sent before; standard messages: `/karis/markers` → `/collision_object`), or
- `karis_task scene_updater` (in this package; `/karis/object_poses_base` → `/collision_object`).

Both remove the part named on `/karis/task/active_object`. Run only **one** of them.

## Common messages

| Message | Meaning |
|---|---|
| `part not found: perception is not running (/karis/get_objects not available)` | No ROS 2 link to the perception laptop (WiFi, peers, domain), or the perception is not running. |
| `part not found: perception did not answer` | The service is seen but the answer took > 5 s (slow first frame, WiFi): send the goal again. |
| `part not found: no matching part` | The perception does not see that class/id now. |
| `MoveIt move_group is not running (/move_action)` | Start run_g1.sh. |
| `MoveIt error code -4` | CONTROL_FAILED: planned, but the execution was aborted by the arm controller. |
| other negative codes | planning failed: target out of reach / orientation not reachable: move the part closer. |

## PLAN / TODO (motion team; fill in with your own code or an AI assistant)

In `karis_task/karis_task/pick_server.py`:

- [ ] **Hand close/open**: `_run()` has `TODO` at "grasping" (close) and "placing" (open). Send a
      FollowJointTrajectory to your hand controller (e.g. `g1_right_ftp_hand/follow_joint_trajectory`)
      and wait for the result before lifting.
- [ ] **Grasp orientation from the part's yaw**: now the hand orientation is fixed
      (`grasp_quat_xyzw`). Rotate it about `pelvis` z by the part's yaw (from `box_pose.orientation`,
      or TF `karis/<object_id>`), using the symmetry of the part (a block: 90° steps, pick the
      smallest wrist turn).
- [ ] **Wrist offset in the part's frame**: `wrist_offset_xyz` is added in `pelvis`; it is only right
      for parts lying like in your test. Rotate it with the same yaw as above.
- [ ] **Straight approach / lift**: approach → grasp → lift are three separate MoveIt goals; for a
      straight vertical motion use a Cartesian path (`/compute_cartesian_path`) for grasp and lift.
- [ ] **Confirm before executing**: optional `confirm` parameter: plan, show it in RViz, wait for a
      key/service, then execute (`plan_only=false` with the same goal or ExecuteTrajectory).
- [ ] **Attach the part** after closing the hand (`/attached_collision_object`, link = the hand), so
      MoveIt plans the lift/place with the part in the hand; detach after opening.
- [ ] **Place in a slot**: already planned with `place_slot` (the slot's position from the
      perception, part height + approach above it). Check the heights on the robot; slots have
      position only (`has_orientation: false`), so the hand keeps the grasp orientation.
- [ ] **Timeout**: `locate()` waits 5 s for the perception; make it a parameter if the perception
      laptop is slow (`fresh: true` waits for the next processed frame).
- [ ] **Table**: add it to the planning scene (`perception_scene_updater add_table:=true`), measured.
