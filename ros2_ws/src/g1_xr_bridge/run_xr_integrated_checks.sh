#!/usr/bin/env bash
# Single offline check; no SDK/robot modules imported.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${ROOT}"
python3 -m unittest -v \
  test_xr_integrated_sim.py \
  test_xr_inprocess_bridge.py \
  test_xr_arm_trajectory.py \
  test_xr_ipc_protocol.py \
  test_g1_pipeline.py
bash "${ROOT}/run_xr_integrated_sim.sh" --verify
printf '\nOffline integrated XR / MoveIt overlay checks PASSED; physical G1 NOT ENABLED.\n'
