# XR simulation overlays — development only

These launch-time overlays and their VR-first adapter are **not** part of the working physical
MoveIt–XR–G1 headless pipeline. They only run with explicit simulator flags;
they do not install modifications into `xr_teleoperate`.

* `xr_integrated_sim_overlay.py` — original isolated XR simulator adapter.
* `xr_minimal_sim_overlay.py` — isolated adapter preserving original VR IK.
* `xr_runtime_overlay.py` — earlier in-memory XR simulator overlay.
* `xr_vr_passthrough_bridge.py` — VR-first adapter for the isolated minimal simulation.

Run the existing bridge-root scripts `run_xr_integrated_sim.sh` or the
`g1_pipeline.py --mode xr-sim` supervisor. Both point to the new locations.
Original XR and its Pinocchio + FTP hand behavior are still owned by the
separately pinned `xr_teleoperate` checkout.

The shared support modules (`xr_inprocess_bridge.py`, `xr_arm_input_mux.py`,
and `xr_sim_startup_patch.py`) deliberately
remain at bridge root. They are imported by multiple existing experiments,
audit scripts, and tests and must be relocated only after a separate review.

To run offline tests (robot not required), run from the bridge directory:

```bash
python3 -m unittest -v \
    test_xr_integrated_sim test_xr_minimal_adapter \
    test_xr_minimal_no_vr_dependency test_xr_inprocess_bridge \
    test_g1_simulation_layout
```

Never use simulator overlays to drive the physical robot. The normal
physical launcher remains `../../launch_g1_moveit_xr.sh`.
