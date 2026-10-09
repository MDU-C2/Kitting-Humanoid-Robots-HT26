# Real G1 LowState → XR command selector: read-only probe

**This is NOT a hardware controller, an XR launcher, or a trajectory action server.**
It never publishes commands, runs VR, acquires robot command ownership, or
changes any file in upstream `xr_teleoperate`.

The physical `--mode hardware` command path stays **hard-blocked**.

## Purpose

The existing G1 read-only observer already returns a validated 29-joint state.
`xr_hardware_feedback.validate_lowstate_snapshot()` extracts the **measured**
14 G1 arm joints. `ReadOnlyMeasuredArmProbe` feeds those exact measurements,
with their reconstructed receipt timestamp, into the existing
`XrArmInputMux.update_measured()` method, then simulates receiving a
**zero-motion** trajectory whose first waypoint equals measured joint q.
The same `TrajectoryTrackingValidator` checks start/path disagreement.

This lets us test a core dependency of the future MoveIt command selector
against real G1 feedback *without* starting XR's motor thread.

Important limitations:

- Simulated acceptance of this no-motion target is **not** physical execution.
- The probe deliberately does not return a motor-ready torque/position command.
- It is not a ROS FollowJointTrajectory action server.
- It does not exercise real VR activation or prove the handover is safe.
- The probe does not authenticate the DDS robot or detect competing publishers.
- It does not address incomplete Inspire FTP hand state or collision geometry.
- In a stationary test, measuring error against a hypothetically stationary
  target is not proof of physical trajectory tracking.

## Offline checks on Marc (G1 off)

From `/workspace` inside Docker:

```bash
python3 -m unittest discover -s src/g1_xr_bridge -p 'test_*.py' -v
python3 src/g1_xr_bridge/g1_pipeline.py \
  --mode hardware-preflight \
  --interface enx50a03000b0dc \
  --arm-selector-probe \
  --no-moveit \
  --print-plan
python3 src/g1_xr_bridge/g1_pipeline.py --mode hardware --print-plan
```

The last command **must refuse** hardware execution.

## Read-only physical integration check (only after offline checks)

Connect the powered G1 over Ethernet, keep it stable, and don't run
any XR or other motor controller as part of this probe. Start the container
with the correct robot-facing interface available. Inside Docker:

```bash
bash src/g1_xr_bridge/run_g1_pipeline.sh \
  --mode hardware-preflight \
  --interface enx50a03000b0dc \
  --arm-selector-probe \
  --no-moveit
```

Do **not** enable `--ftp-observe` until its independent DDS publishers are
available: prior tests discovered only `rt/lowstate` and no FTP state writers.

Look for:

```text
G1 READ-ONLY PREFLIGHT PASS: ...
G1 ARM SELECTOR READ-ONLY PROBE PASS: {"...", "read_only": true, "execution_enabled": false, "physical_execution": false}
NO ACTUATION: selector takeover and tracking are hypothetical only.
```

The two monitors continuously check real LowState freshness. A failure shuts
down the read-only pipeline. Stop it normally with `Ctrl+C`; for stubborn
processes, identify the specific PID and terminate only that observer.

## Next stage

The actual future controller must use the **same measured feedback** during
trajectory execution, enforce one command owner and an independent physical
stop, and report action success only after reaching the measured goal within
validated tolerances. The original XR startup, Pinocchio, VR IK, and Inspire
FTP controller must remain unchanged.
