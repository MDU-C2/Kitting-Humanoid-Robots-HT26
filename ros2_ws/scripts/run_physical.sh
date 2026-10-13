#!/usr/bin/env bash
# Inside the clean Docker container; EXPLICIT supervised physical activation.
set -Eeuo pipefail
[[ -x /opt/mamba/envs/xr/bin/python ]] || { echo 'XR environment missing; run install.sh' >&2; exit 1; }
[[ -f /workspace/install/setup.bash ]] || { echo 'Built MoveIt workspace missing' >&2; exit 1; }
[[ "${G1_INTERFACE:-unconfigured}" != unconfigured ]] || {
  echo 'G1_INTERFACE missing; restart Docker with correct Ethernet interface.' >&2; exit 1;
}
echo 'WARNING: This will activate physical Unitree G1 arm control.'
echo 'An independent tested stop method, trained operator, and clear area are REQUIRED.'
echo 'This script and Ctrl+C are NOT an emergency stop.'
read -r -p 'Type G1_REGULAR_MODE_SUPERVISED to authorize physical activation: ' ACK
[[ "$ACK" == G1_REGULAR_MODE_SUPERVISED ]] || { echo 'Cancelled; no publisher started.'; exit 2; }
exec bash /workspace/src/g1_xr_bridge/launch_g1_moveit_xr.sh \
  --hardware G1_REGULAR_MODE_SUPERVISED
