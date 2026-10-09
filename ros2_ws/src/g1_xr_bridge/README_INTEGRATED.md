# G1 XR + MoveIt Integration (updated)

The maintained integration instructions are now in **[README_PIPELINE.md](README_PIPELINE.md)**.

The older implementation in this directory was a simulation-only prototype.
This update adds an offline and read-only ROS pipeline launcher, an explicitly
isolated-simulator XR/VR launcher, XR action mapping and a simulator-only
motion-blend/watchdog patch. **Physical G1 execution remains disabled.**

Run `python3 g1_pipeline.py --verify` inside the supplied Docker environment,
or `python3 g1_pipeline.py --print-plan` to inspect the child commands.
