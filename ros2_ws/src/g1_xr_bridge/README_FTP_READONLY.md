# Optional Inspire FTP feedback preflight — read-only

This is an **opt-in read-only addition** to the existing G1 hardware preflight.
It **does not modify or run** the original `xr_teleoperate` hardware controller,
Pinocchio, VR code, zero initialization, or Inspire FTP hand command code.
No motor publishers, trajectory action server, or `/joint_states` hand values
are added. `--mode hardware` is still blocked.

## Why the hands are not automatically inserted into `/joint_states`

The original `Inspire_Controller_FTP` subscribes to these DDS topics:

- `rt/inspire_hand/state/l`
- `rt/inspire_hand/state/r`

It reads `angle_act[0..5]` per hand and divides by 1000, yielding six
**normalized actuator positions** per hand. Order: pinky, ring, middle,
index, thumb bend, thumb rotation. Those are **not** 12 validated independent
URDF joint rotations in radians. Publishing them unmodified under the MoveIt
`left_*_joint` / `right_*_joint` names would misrepresent the robot geometry.

This patch intentionally publishes only a diagnostic ROS topic:

- `/g1_ftp/angle_act_normalized` (`std_msgs/msg/Float64MultiArray`), 12 values
  `[L:pinky,ring,middle,index,thumb_bend,thumb_rotation,
  R:pinky,ring,middle,index,thumb_bend,thumb_rotation]`, each in `[0,1]`.

When either hand is stale or missing, it does **not** publish fake replacements.
MoveIt will continue warning about missing finger joints until their correct
hardware-to-URDF mapping is verified and implemented later.

## Offline tests — keep robot powered off

Inside Docker:

```bash
cd /workspace
python3 -m unittest discover -s src/g1_xr_bridge -p 'test_*.py' -v
python3 src/g1_xr_bridge/g1_pipeline.py --mode hardware-preflight \
  --ftp-observe --interface enx50a03000b0dc --print-plan
python3 src/g1_xr_bridge/g1_pipeline.py --mode hardware --print-plan
```

The last command **must** refuse hardware execution. `--print-plan` does not
connect to the physical G1.

## Optional SDK dependency check — before real-robot testing

The FTP observer uses the **same** `inspire_sdkpy.inspire_dds.inspire_hand_state`
DDS message class as original XR, but our archived Micromamba Docker script did
**not** explicitly install `inspire_sdkpy`. Check its availability first:

```bash
/opt/mamba/envs/xr/bin/python -c 'from inspire_sdkpy import inspire_dds; print("FTP DDS import OK")'
```

If it fails, **do not install an arbitrary substitute package or copy private
runtime files blindly**. Locate the actual version of the SDK used by your
working original XR installation and integrate that pinned dependency into the
Docker build separately. Standard G1 LowState observation remains unaffected.

## Later: supervised *read-only* G1+FTP hardware test

Only when the SDK import is verified, G1 Ethernet is correct, the G1 is stable,
and the normal XR motor controller is **not** running:

```bash
bash src/g1_xr_bridge/run_g1_pipeline.sh --mode hardware-preflight \
  --ftp-observe --interface enx50a03000b0dc
```

Look for both `G1 READ-ONLY PREFLIGHT PASS` and
`G1 FTP READ-ONLY PREFLIGHT PASS`. The FTP preflight requires continuously
advancing messages from **both** hands, not merely nonzero values. Its failure
stops the supervisor; it never publishes a physical stop command.

In another Docker terminal:

```bash
ros2 topic echo /g1_ftp/angle_act_normalized --once
ros2 topic hz /g1_ftp/angle_act_normalized
```

When finished, stop with `Ctrl+C` and use the robot's normal shutdown.
This verifies reception and basic data validity only. It **does not** prove
identity, hand calibration, joint mapping, collision geometry, exclusive DDS
command ownership, or safe motor tracking. Do not attempt MoveIt execution.
