# Minimal XR MoveIt adapter (isolated simulator only)

**Purpose:** Allow ROS 2 MoveIt to select targets at the existing original XR
`arm_ctrl.ctrl_dual_arm(sol_q, sol_tauff)` call, without changing any of the
original `xr_teleoperate` files, the `G1_29_ArmController` class, its initial
zero targets, weight handling, VR activation procedure, Pinocchio algorithms,
or `Inspire_Controller_FTP`.

The adapter is implemented as an **opt-in, memory-only source overlay**.
The project's existing XR-SIM overlay remains the default. The original VR
controller receives its original IK outputs **unchanged until the first
accepted MoveIt trajectory sample**. An accepted trajectory takes ownership;
its targets and XR gravity torques are selected. On sample timeout it switches
to a request for HOLD (not physical torque holding); VR must be resumed
explicitly and only after its target is close to measured joints.

## Offline checks (physical G1 off)

```bash
cd /workspace
python3 -m unittest discover -s src/g1_xr_bridge -p 'test_*.py' -v
G1_XR_SOURCE_ROOT=/workspace/src/xr_teleoperate \
  /opt/mamba/envs/xr/bin/python src/g1_xr_bridge/xr_minimal_sim_overlay.py --verify
python3 src/g1_xr_bridge/g1_pipeline.py --mode xr-sim --xr-adapter minimal \
  --isolation-ack ISOLATED_SIMULATOR --sim-interface sim0 --print-plan
```

The `--print-plan` command **only prints intended processes**; it does not
create a network interface or start the robot. The `sim0` name is a placeholder.

## Optional isolated simulator run

**ONLY** when a real independent Isaac simulator DDS network is available,
physically and logically isolated from the physical G1:

```bash
bash src/g1_xr_bridge/run_g1_pipeline.sh --mode xr-sim \
  --xr-adapter minimal --isolation-ack ISOLATED_SIMULATOR \
  --sim-interface <ISOLATED_SIM_INTERFACE> \
  --image-server-ip <SIM_CAMERA_SERVER>
```

Never use a physical G1 Ethernet/Wi-Fi interface for this experiment.
The adapter always requires `--sim` (XR DDS domain 1); `--motion` here is the
**simulator's original XR motion option**, NOT permission to move a real robot.

## Important limits

- Physical motor commands remain blocked by the current pipeline (`--mode hardware`).
- No new hardware launch path, arm SDK publisher, or changes to XR files.
- Offline tests verify selection semantics, **not real safety or tracking**.
- The external MoveIt action sim's acknowledgments do not prove physical tracking.
- The XR IK loop still executes during MoveIt trajectories (to minimize changes).
  We can optimize that later, after comparing original VR behavior.
- FTP hands remain wholly controlled by upstream XR; MoveIt controls 14 arm joints.
- On original XR shutdown, its original `go_home` behavior remains intact.
