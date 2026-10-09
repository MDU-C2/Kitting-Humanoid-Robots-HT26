# G1 MoveIt 2 ↔ XR pipeline — safe, single-entry launcher

**Status:** software integration prototype. Hardware execution remains **disabled**.
This bundle was prepared from the supplied `g1_pipeline_sources.tar.gz` pinned XR
checkout; upstream `src/xr_teleoperate/` is not edited. The archive does not
contain `g1_description`, the main Dockerfile, the complete ROS install or a G1;
those cannot be verified or changed here.

## Install patch into existing repository

From the directory containing your ROS workspace `src/` (typically `/workspace`
inside Docker, or `ros2_ws/` on your host), extract the provided patch ZIP:

```bash
cd /workspace
unzip -o /path/to/g1_moveit_xr_pipeline_patch.zip
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select g1_moveit_config
source /workspace/install/setup.bash
```

The ZIP includes **only** `src/g1_xr_bridge` and `src/g1_moveit_config` changes,
not the 205 MB XR source. It leaves your `g1_control` source and original
`g1_moveit_config/launch/move_group.launch.py` unchanged. The existing Docker
image must supply the `XR` Micromamba Python at `/opt/mamba/envs/xr/bin/python`;
there is no manual `pip install` or `conda activate` step. Since the actual
Dockerfile was not in your upload, image setup is not modified by this patch.

## One launcher (inside the Docker container)

```bash
cd /workspace

# Offline: NO Unitree SDK or DDS, simulated instantaneous tracking, MoveIt+RViz
python3 src/g1_xr_bridge/g1_pipeline.py --mode offline

# Observe: ONLY subscribe to real G1 rt/lowstate, MoveIt planning but no execution
python3 src/g1_xr_bridge/g1_pipeline.py --mode observe --interface eth0

# Review without starting any process (also supports --mode xr-sim)
python3 src/g1_xr_bridge/g1_pipeline.py --mode offline --print-plan

# Run checks, including pinned XR overlay/source-mock checks
python3 src/g1_xr_bridge/g1_pipeline.py --verify
```

`Ctrl+C` shuts down all children. Another socket owner causes refusal rather
than unlinking someone else's IPC endpoint. Observer mode **does not launch XR**,
VR, the trajectory action or any motion-related DDS publisher.

### XR + VR + MoveIt — actual *isolated simulator* only

`--mode xr-sim` launches the original XR application, using its original
`G1_29_ArmIK`, original `G1_29_ArmController` in **motion mode**, original
Pinocchio gravity compensation and hand/VR stack, plus MoveIt, ROS state
relay and an XR trajectory action. It requires Isaac, the XR camera/image
server, VR device, SDK Python environment and correct network routing.

The VR display/image server may need an actual address instead of `127.0.0.1`.
Only run on a network that cannot reach the physical G1, including multicast DDS:

```bash
python3 src/g1_xr_bridge/g1_pipeline.py \
  --mode xr-sim \
  --isolation-ack ISOLATED_SIMULATOR \
  --sim-interface <ISOLATED_SIM_DDS_NIC> \
  --image-server-ip <SIMULATOR_CAMERA_SERVER_IP>
```

The word `--sim` and a different DDS domain **do not guarantee physical
isolation**. Ensure physical cable/VLAN/interface isolation and verify that no
robot can receive Unitree traffic before using this mode. This launch is
**not validated end-to-end** in the provided environment. XR starts with its
usual `r` keyboard/IPC start interaction; MoveIt goals are rejected while XR's
main loop is not providing fresh feedback. Option `--xr-ipc` selects the upstream
IPC server in place of keyboard commands. The original Inspire FTP hand
controller runs only in the XR simulated mode, not the offline harness.

Never launch `g1_control` or any other `rt/arm_sdk` publisher concurrently with
an XR actuator owner. The launcher itself never starts `g1_control`, but cannot
detect every other writer in ROS, DDS or onboard robot services.

### Disabled physical mode

```bash
python3 src/g1_xr_bridge/g1_pipeline.py --mode hardware
```

The launcher **intentionally refuses** this mode. Unlike observation, XR's
original controller periodically publishes to `rt/arm_sdk` in motion mode;
we have not authorized that against a real G1.

## Architecture

```text
              VR headset + camera
                     |
            Original XR IK + RNEA ---------------.
                     |                            |
                VR target                        |
                     v                            |
MoveIt 2 --> FollowJointTrajectory --> ROS samples
                                       |
                                   Unix IPC
                                       |
                              XR in-process mux
                     VR / TRAJECTORY / HOLD
                                       |
                           single XR ctrl_dual_arm setter
                                       |
                          original G1_29_ArmController
                          Pinocchio gravity FF + motion guard
                                       |
                        Unitree DDS ONLY in isolated simulator

Observing physical robot: rt/lowstate -> read-only observer -> Unix state IPC
                         -> ROS /joint_states -> robot_state_publisher + MoveIt
Offline simulator: fake ideal XR state -> same /joint_states relay + MoveIt
```

- `XrArmInputMux` arbitrates full 14-joint commands. No automatic trajectory→VR
  resume; only explicit resume IPC after HOLD and pose-distance check.
- New seven-joint left or right arm trajectories are expanded using **fresh
  measured XR state** for the other arm; there is still only one action owner,
  not two competing per-arm publishers.
- The original `g1_moveit_config/launch/move_group.launch.py` still points at
  `ros2_control`. Only new `xr_pipeline_moveit.launch.py` points to the XR
  prototype action `/g1_xr_bridge/inprocess_sim/follow_joint_trajectory`.
- In simulated XR, the patched controller seeds commanded `q` from LowState,
  computes `tau_ff` from original XR Pinocchio RNEA **before** its 250 Hz
  publisher starts, begins motion blend weight at zero, then ramps up only
  with fresh XR loop heartbeats and LowState receive timestamps. It ramps down
  when those timestamps become stale; a software watchdog is **not** an E-stop.
- XR controller source remains unchanged on disk: fail-closed in-memory
  overlays check pinned source anchors and compile before launching.

## Verification scope and known limitations

- The local offline tests cover 14/7-joint interpolation, VR arbitration,
  stale commands, read-only state contract, startup source patch, first
  measured-position+gravity seed, simulator blend guard, launch modes and
  hardware-mode refusal.
- ROS 2 Humble, MoveIt 2 and the Unitree SDK were **not installed** in the
  authoring runtime. Therefore we could not run the actual ROS action,
  `colcon`, RViz, Isaac, VR, camera server or DDS/G1 end-to-end.
- In offline mode joint feedback is **synthetic, ideal tracking**. Do not read
  action success as a real measured/goal-tolerance result. The Isaac XR mode
  also needs real feedback tracking tolerances before hardware certification.
- The archived `src/` tree lacks `g1_description` and the Dockerfile, so
  verify URDF 29-joint names and dependency image installation on your machine.
- Physical startup needs joint limits, tracking-error and torque checks,
  mode-machine/arm ownership tests, watchdog with safe hardware stop, collision
  validation, measured action tolerances, correct gripper behavior, and
  supervised safe commissioning before it can be enabled.
