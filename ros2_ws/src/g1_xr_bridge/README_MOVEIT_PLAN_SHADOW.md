# Live MoveIt Plan → measured G1 start → XR selector shadow (NO MOTION)

This is the next **read-only** integration point toward VR/MoveIt arbitration.
It does not start the original XR application, bypass `--mode hardware`'s
hard block, create `FollowJointTrajectory` action servers, or publish any DDS
motor commands. The original XR IK, zero initialization, Pinocchio, motion
mode, and Inspire FTP hand behavior are unchanged.

## What it does

When RViz MotionPlanning **Plan** publishes `moveit_msgs/DisplayTrajectory`
on `/display_planned_path`, the monitor:

1. Obtains a **fresh, actual** 29-joint G1 body snapshot from our already
   validated read-only observer.
2. Extracts 14 measured arm q and checks the plan's starting arm position.
3. Accepts exactly one 7-joint left/right arm plan or 14-arm plan (never
   fabricates FTP hand state, never ignores multi-DoF motion).
4. Replays the plan **in an isolated virtual selector** to exercise trajectory
   ordering and the TRAJECTORY → HOLD transition. From the second sample on,
   its feedback is *virtual*, not actual hardware tracking.
5. Prints a summary explicitly marked `NOT EXECUTED` or a rejection reason.

Plan visualization is **not execution authorization**. No physical tracking,
controller ownership, SDK stop behavior, physical collision checking, VR
handover under load, or Inspire hand feedback is validated here. A successful
shadow test must NEVER be interpreted as physical readiness to execute.

## Offline checks (G1 powered off)

```bash
cd /workspace
python3 -m unittest discover -s src/g1_xr_bridge -p 'test_*.py' -v
python3 src/g1_xr_bridge/g1_pipeline.py \
  --mode hardware-preflight \
  --interface enx50a03000b0dc \
  --moveit-plan-shadow --print-plan
python3 src/g1_xr_bridge/g1_pipeline.py --mode hardware --print-plan
```

The plan must show a **read-only** observer, preflight, ROS state relay,
MoveIt + RViz with `enable_execution:=false`, and the new display-plan
subscriber. Hardware mode MUST refuse.

## Physical read-only shadow (only after offline checks)

With G1 powered on and Ethernet available to Docker, original XR not
running, and the robot stationary:

```bash
bash src/g1_xr_bridge/run_g1_pipeline.sh \
  --mode hardware-preflight \
  --interface enx50a03000b0dc \
  --moveit-plan-shadow
```

In RViz select `left_arm`, `right_arm`, or `both_arms`; choose a conservative
goal and press **Plan**, **NOT Execute**. The shadow monitor should print:

```
MOVEIT PLAN SHADOW PASS — NOT EXECUTED: ... "read_only": true, "physical_execution": false
```

The plan will be REJECTED if it starts far from the **measured** arms,
contains unsupported joints, or if there is no fresh G1 feedback.
MoveIt may continue warning about 12 unmeasured FTP joints; this patch
never substitutes synthetic hand positions on physical hardware. Current
pelvis/waist collision geometry limitations also still apply.

If no messages appear, inspect `ros2 topic info /display_planned_path -v`.
Stop with `Ctrl+C` after observing results. The G1 is never commanded by this
new code.
