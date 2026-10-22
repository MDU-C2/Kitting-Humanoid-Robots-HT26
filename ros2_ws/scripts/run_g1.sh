#!/usr/bin/env bash
# Container-side, explicitly supervised physical G1 connection and startup.
# Network/DDS are selected WHEN invoked, not when Docker is created.
# --check: read-only G1 LowState DDS test; never starts motors.
set -Eeuo pipefail

usage() {
  echo 'Usage: bash /workspace/scripts/run_g1.sh [--check]' >&2
  echo 'Optional: G1_INTERFACE=eth2; G1_ROBOT_IP=verified_robot_IP (route hint)' >&2
  exit 2
}
MODE="${1:-}"
[[ "$MODE" == '' || "$MODE" == '--check' ]] && (( $# <= 1 )) || usage
[[ -f /workspace/src/g1_xr_bridge/launch_g1_moveit_xr.sh ]] || {
  echo 'Run this script INSIDE the G1 Docker container (/workspace is missing).' >&2
  exit 1
}
[[ -x /opt/mamba/envs/xr/bin/python ]] || { echo 'XR environment missing; run install.sh.' >&2; exit 1; }
[[ -f /workspace/install/setup.bash ]] || { echo 'Built MoveIt workspace missing; run install.sh.' >&2; exit 1; }
command -v ip >/dev/null || { echo 'iproute2 is required.' >&2; exit 1; }
command -v timeout >/dev/null || { echo 'timeout is required.' >&2; exit 1; }

IFACE="${G1_INTERFACE:-}"
[[ "$IFACE" == unconfigured ]] && IFACE=''
if [[ -z "$IFACE" && -n "${G1_ROBOT_IP:-}" ]]; then
  # A caller-provided robot IP is a routing HINT, not proof of robot identity.
  IFACE="$(ip route get "$G1_ROBOT_IP" 2>/dev/null |
    awk '{for (i=1;i<=NF;i++) if ($i=="dev") {print $(i+1);exit}}' || true)"
fi
if [[ -z "$IFACE" ]]; then
  # Legacy directly wired Unitree links commonly use 192.168.123.0/24.
  # Use that only if it identifies exactly one interface. Never assume .161
  # is the robot; it may be the address of this workstation.
  mapfile -t CANDIDATES < <(ip -o -4 addr show scope global |
    awk '$4 ~ /^192\.168\.123\./ {print $2}' | sort -u)
  if (( ${#CANDIDATES[@]} == 1 )); then
    IFACE="${CANDIDATES[0]}"
    echo "Discovered one candidate G1 interface: $IFACE"
  fi
fi
if [[ -z "$IFACE" ]]; then
  echo 'Cannot uniquely identify a G1 network interface.' >&2
  echo 'Inspect with: ip -br addr; ip route' >&2
  echo 'Then run: G1_INTERFACE=<actual-interface> bash /workspace/scripts/run_g1.sh --check' >&2
  exit 1
fi
if [[ ! "$IFACE" =~ ^[a-zA-Z0-9_.:-]+$ || "$IFACE" == lo || ! -d "/sys/class/net/$IFACE" ]]; then
  echo "Invalid or missing G1 interface: $IFACE" >&2
  exit 1
fi
if [[ "$(cat "/sys/class/net/$IFACE/operstate")" != up ]]; then
  echo "G1 interface $IFACE is not UP. Connect robot/link and retry." >&2
  exit 1
fi
if ! ip -4 -o addr show dev "$IFACE" | grep -q 'inet '; then
  echo "G1 interface $IFACE has no IPv4 address." >&2
  exit 1
fi

# ROS CycloneDDS is scoped to this launch and its children. The Unitree SDK's
# ChannelFactoryInitialize independently binds its own DDS to --interface.
export G1_INTERFACE="$IFACE"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${IFACE}\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
echo "Selected G1 interface: $IFACE"
echo 'Checking read-only Unitree rt/lowstate DDS before physical activation...'
# Limit even an unexpectedly blocking vendor SDK call. No DDS command publisher.
if ! timeout --signal=TERM --kill-after=2s 16s \
  env -u PYTHONPATH \
    LD_LIBRARY_PATH="/opt/mamba/envs/xr/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
    /opt/mamba/envs/xr/bin/python /workspace/scripts/check_g1_dds.py "$IFACE"; then
  echo 'G1 read-only DDS check FAILED. No motor-control process was started.' >&2
  echo 'Check interface, robot power, network/DDS isolation, and Unitree topic discovery.' >&2
  exit 1
fi
if [[ "$MODE" == '--check' ]]; then
  echo 'READ-ONLY CHECK PASSED. No motor-control process was started.'
  exit 0
fi

# G1_STARTUP_DEFAULTS_V1: explicit defaults for the physical MoveIt-XR run.
# Caller overrides remain possible for troubleshooting; never change upstream VR.
export G1_GRAVITY_MODEL="${G1_GRAVITY_MODEL:-inspire_ftp}"
export G1_GOAL_CHECK_MODE="${G1_GOAL_CHECK_MODE:-wrist}"
case "$G1_GRAVITY_MODEL" in
  xr|inspire_ftp) ;;
  *) echo "Invalid G1_GRAVITY_MODEL=$G1_GRAVITY_MODEL (xr or inspire_ftp)" >&2; exit 2 ;;
esac
case "$G1_GOAL_CHECK_MODE" in
  joint|wrist) ;;
  *) echo "Invalid G1_GOAL_CHECK_MODE=$G1_GOAL_CHECK_MODE (joint or wrist)" >&2; exit 2 ;;
esac
# Fail closed if the requested new modes are not installed; otherwise a
# launcher could silently use the original model or original joint check.
if [[ "$G1_GRAVITY_MODEL" == inspire_ftp ]] && \
   ! grep -q 'G1_INSPIRE_FTP_GRAVITY_V1' /workspace/src/g1_xr_bridge/launch_g1_moveit_xr.sh; then
  echo 'Inspire FTP gravity integration not installed. No motor controller started.' >&2
  exit 2
fi
if [[ "$G1_GOAL_CHECK_MODE" == wrist ]] && \
   ! grep -q 'G1_WRIST_GOAL_CHECK_V1' /workspace/src/g1_xr_bridge/ros2_headless_moveit_action.py; then
  echo 'Wrist-goal completion integration not installed. No motor controller started.' >&2
  exit 2
fi
echo "G1 defaults: gravity=$G1_GRAVITY_MODEL; goal_check=$G1_GOAL_CHECK_MODE"
echo 'WARNING: Next step activates physical Unitree G1 arm control.'
echo 'A trained operator, clear area, and independent tested stop method are REQUIRED.'
echo 'This launcher and Ctrl+C are NOT an emergency stop.'
read -r -p 'Type G1_REGULAR_MODE_SUPERVISED to authorize physical activation: ' ACK
[[ "$ACK" == G1_REGULAR_MODE_SUPERVISED ]] || {
  echo 'Cancelled; no motor-control process started.'
  exit 2
}
exec bash /workspace/src/g1_xr_bridge/launch_g1_moveit_xr.sh \
  --hardware G1_REGULAR_MODE_SUPERVISED
