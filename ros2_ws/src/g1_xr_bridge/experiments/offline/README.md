# Offline XR integration experiments

**Not the physical G1 pipeline.** The physical MoveIt–XR launcher remains
`../../launch_g1_moveit_xr.sh` and is intentionally unaffected by these tools.

These scripts use the experimental in-process XR simulation bridge still
located at `../../xr_inprocess_bridge.py`; that module is also imported by
other simulation overlays and their tests, so it has **not** been moved.

From the ROS 2 Humble container, use the original `../../run_g1_pipeline.sh`
for an offline run. The original `../../run_xr_harness.sh` continues to
launch the harness at its new location. Do not run these tools against the
physical G1. They are not motor-control implementations.

This directory contains the offline ROS action server, simulated action
endpoint, XR harness, fault-injection runner, and standalone status/client
tools. The optional `send_inprocess_sim_goal.py` still depends on
`send_offline_ipc_goal`, which was **absent** from the uploaded source
snapshot; that tool has not been demonstrated runnable by this cleanup.

Run offline regressions from the bridge root:

```bash
python3 -m unittest -v test_g1_bridge_contracts test_g1_pipeline \
    test_xr_ipc_protocol test_xr_arm_trajectory \
    test_offline_feedback_contract test_xr_inprocess_bridge \
    test_g1_offline_execution test_g1_offline_layout
```
