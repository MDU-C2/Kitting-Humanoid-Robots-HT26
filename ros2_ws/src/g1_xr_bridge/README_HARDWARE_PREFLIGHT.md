# Physical G1 feedback preflight — read-only

This addition is a **validation stage**, **not a physical XR actuator launcher**.
It leaves every file in upstream `xr_teleoperate` untouched, including the
original zero initialization, VR activation, arm publisher, Pinocchio, and
Inspire FTP hand logic. It does not change the minimal simulation adapter.

## What is implemented

- `xr_hardware_feedback.py`: SDK-free validator for the **actual** read-only
  `rt/lowstate` observer response, 29-joint ordering, freshness, finite position
  and velocity, and 14-arm indexing. Also contains an independently testable
  `TrajectoryTrackingValidator` for start, path, and final position/velocity
  checks. **This validator is not yet attached to a physical trajectory action.**
- `g1_hardware_preflight.py`: polls the *existing* local read-only observer
  socket, checks that fresh LowState receipts advance, and fails if state is
  missing, stale, or malformed. No DDS publisher, no SDK import, no ROS action.
- `g1_pipeline.py --mode hardware-preflight`: starts the read-only observer,
  continuous preflight monitor, ROS joint-state relay, and optionally MoveIt
  and RViz **with execution disabled**. The old `observe`/`offline`/`xr-sim`
  modes remain available and `--mode hardware` is still blocked.

## Test without physical G1 (preferred first)

Inside the running project Docker container:

```bash
cd /workspace
python3 -m unittest discover -s src/g1_xr_bridge -p 'test_*.py' -v
python3 src/g1_xr_bridge/g1_pipeline.py --mode hardware-preflight \
  --interface enx50a03000b0dc --print-plan
python3 src/g1_xr_bridge/g1_pipeline.py --mode hardware --print-plan
```

The last command **must be refused**. The interface in `--print-plan` is
not used for network traffic; it merely appears in the printed commands.

## Later, when a supervised physical read-only test is appropriate

Power the G1 on using its normal procedure, keep the robot stable, and ensure
its Ethernet adapter is unambiguously routed. On Marc, Wi-Fi and Ethernet
previously both had addresses on `192.168.123.0/24`; check `ip route` and
DDS interface selection instead of assuming traffic uses Ethernet.

**Stop any previous pipeline first.** Inside Docker:

```bash
bash src/g1_xr_bridge/run_g1_pipeline.sh --mode hardware-preflight \
  --interface enx50a03000b0dc
```

The continuous monitor prints `G1 READ-ONLY PREFLIGHT PASS` after enough
fresh valid measurements. If the observer stops providing valid state, the
monitor exits and the supervisor shuts down the read-only processes. The
monitor itself never commands a hold, stop, motor, or hand.

`/joint_states` is sourced from read-only measured state. MoveIt may **plan**
but cannot execute through this launcher. No simulated or physical trajectory
action is started. Original VR-XR hardware controller is **not started**.

## What this does NOT prove

- That the LowState DDS publisher belongs to the intended physical G1;
  independent robot/network identity verification is still required.
- That no other application is publishing `rt/arm_sdk` or other commands;
  ROS graph discovery alone cannot guarantee exclusive DDS ownership.
- Any arm SDK actuation, original XR VR headset function in the new mode,
  safe physical hold/stop, or real motor tracking performance.
- The physical Inspire FTP fingers: this feedback contract deliberately
  covers **14 G1 arm joints only**, while the original XR hand controller
  remains unchanged and unmodified.
- Collision checking: missing pelvis/waist collision geometry in the model
  still needs separate attention before relying on physical planning.

**Next implementation stage**, after read-only preflight is validated on
hardware: connect an opt-in external measured-feedback monitor to the physical
command-source path, prove command ownership/cancellation/stop behavior,
validate XR's original VR handover on hardware, and only then separately
consider enabling physical trajectory execution. Do **not** bypass the current
`--mode hardware` block.
