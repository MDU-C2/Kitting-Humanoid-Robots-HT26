# G1 headless MoveIt → XR integration (physical-code candidate; NOT robot-validated)

**Goal:** MoveIt 2 supplies trajectories to Unitree XR's existing `G1_29_ArmController` in motion mode, without a Meta Quest, image server, VR IK, or another `ros2_control` publisher.

The XR physical-control path is *not* reimplemented. The original `robot_arm.py` continues to supply the actual 250 Hz `rt/arm_sdk` writer, clipping, gains, and CRC. Only its headless-only initialization/release branch is added. Existing Quest/VR operation is unchanged when `headless_moveit=False`.

This is an **implementation candidate, not verified physical safety**. Source patching, Python syntax, and offline patch idempotence were checked on the supplied XR archive. Neither the patched live controller nor its actuator behavior has been tested on the G1. **Do not run physical activation until a supervising robot operator has reviewed startup, independent stop, arm/hand load, clearance, and controller exclusivity.** Releasing SDK weight might allow arms to move under gravity; it is *not* a dependable emergency stop. Never use `kill -9` to test stopping.

## Included files

- `install_xr_headless_patch.py`: modifies only the existing `G1_29_ArmController` class, creating a timestamped? **No:** fixed-name `.before_headless_moveit.bak` backup. Dry-run by default; `--apply` required.
- `xr_headless_moveit.py`: headless Micromamba XR process; reads real LowState, calculates original Pinocchio RNEA gravity, owns the **single** XR `arm_sdk` publisher, accepts trajectory/hold requests through a local Unix socket.
- `ros2_headless_moveit_action.py`: ROS Humble `FollowJointTrajectory` action server at `/g1_xr_bridge/follow_joint_trajectory`; accepts either a 7-joint left/right trajectory or full 14-joint trajectory; obtains missing-arm baseline from the real XR state, not zeros.
- `moveit_controllers.example.yaml`: suggested MoveIt Simple Controller Manager wiring; must be merged with your existing MoveIt configuration, not blindly replace it.

## Preparation on Marc

Unpack the zip contents into `/workspace/src/g1_xr_bridge/` (host checkout usually `ros2_ws/src/g1_xr_bridge/`; the project bind mount provides `/workspace/src/`). **Do not overwrite other packages or the original XR repo indiscriminately.**

In the Docker container, with the robot not commanded by this package:

```bash
cd /workspace/src/g1_xr_bridge
python3 install_xr_headless_patch.py --source /workspace/src/xr_teleoperate/teleop/robot_control/robot_arm.py
python3 install_xr_headless_patch.py --source /workspace/src/xr_teleoperate/teleop/robot_control/robot_arm.py --apply
python3 -m py_compile install_xr_headless_patch.py xr_headless_moveit.py ros2_headless_moveit_action.py
G1_XR_SOURCE_ROOT=/workspace/src/xr_teleoperate /opt/mamba/envs/xr/bin/python xr_headless_moveit.py --verify
```

**Installing the source patch does not publish motor commands.** It is only executed if the separate headless launcher is explicitly started with `--hardware`. For rollback, stop the headless controller, restore the backup and preserve your repository's other changes; do not overwrite newer XR edits blindly.

## Proposed physical activation (not yet validated)

**Only after a trained operator checks the physical stop procedure, power/clearance, no competing external XR process, and supported Regular mode.** The operator must remain beside the robot with the verified controller stop throughout.

Terminal A, Docker, Micromamba XR environment; no VR headset:

```bash
cd /workspace/src/g1_xr_bridge
unset PYTHONPATH
export LD_LIBRARY_PATH=/opt/mamba/envs/xr/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
/opt/mamba/envs/xr/bin/python xr_headless_moveit.py \
  --hardware --interface enx50a03000b0dc \
  --operator-ack G1_REGULAR_MODE_SUPERVISED
```

This launches the physical XR arm publisher and a local socket (`/tmp/g1_xr_headless_moveit.sock`), starting with the **measured arm position**, calculating initial gravity feedforward, ramping `arm_sdk` weight from 0 to 1 in 3 seconds, and remaining in HOLD. It does **not** start motion automatically, and does not initialize or command Inspire hands. Verify a stable stationary HOLD **before starting Terminal B**.

Terminal B, Docker, ROS Humble environment:

```bash
cd /workspace
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
/usr/bin/python3 /workspace/src/g1_xr_bridge/ros2_headless_moveit_action.py
```

This advertises `/g1_xr_bridge/follow_joint_trajectory`. It does **not** generate any movement by itself. Execution requires a MoveIt controller configuration mapping the arm trajectory controller to this action server, and then deliberately planning and executing a checked trajectory from RViz or MoveIt.

To use the stock MoveIt simple controller manager, the corresponding `moveit_controllers.yaml` needs to map all 14 arm joints to this action namespace. **The archive did not include your current MoveIt controller YAML**, so it has intentionally not been rewritten automatically. Joint names/order are defined in `/workspace/src/g1_xr_bridge/xr_arm_trajectory.py`. Do not enable any other hardware controller for the same arms.

## Deliberate first-experiment constraints

- Headless controller must be in regular/motion mode, have LowState <150 ms old, and finish the 3.5-second takeover ramp.
- Goal must start within 0.05 rad of measured actual arms; velocity <0.12 rad/s at the start.
- For the first physical experiments only: whole motion is limited to 0.12 rad from starting measured position, and sampled maximum speed 0.2 rad/s; duration 0.5–12 s.
- Only one active goal. Any ROS sample gap >250 ms causes XR to capture and HOLD its measured arm pose. Stale LowState triggers a ramped release of arm SDK authority, but **cannot guarantee robot safety**.
- Trajectory tracking error >0.15 rad aborts; final measured position must be within 0.05 rad with speed <0.12 rad/s to report success.
- Terminal A `Ctrl-C` requests XR authority ramp-down without return-to-zero; this must be physically validated before relying on it. The independent robot emergency stop takes precedence.

## Limitations

- Not tested on G1 hardware. Runtime SDK, timing and arm ownership remain to be validated with the actual installed XR version and G1.
- Headless controller does **not** command Inspire FTP hands. Gravity model may not account accurately for mounted hand mass.
- The controller action is **not a collision checker**: use MoveIt to plan and verify collision avoidance and joint bounds before execution.
- Action config integration into your exact MoveIt package still requires reviewing the repository's actual controller configuration.
- A Python process crash, OS freeze, power loss, or network failure may prevent orderly ramp-down. The robot's independent safety measures are required.
- Existing `g1_pipeline.py --mode hardware` remains blocked and should not be bypassed; this is a separate opt-in headless entry point.
