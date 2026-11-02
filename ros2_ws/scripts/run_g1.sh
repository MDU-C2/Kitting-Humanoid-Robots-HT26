#!/usr/bin/env bash
# Container-side, explicitly supervised physical G1 connection and startup.
# Network/DDS are selected WHEN invoked, not when Docker is created.
# --check: read-only G1 LowState DDS test; never starts motors.
set -Eeuo pipefail
usage() {
  echo 'Usage: bash /workspace/scripts/run_g1.sh [--check]' >&2
  echo 'Optional: G1_INTERFACE=<G1-Ethernet>; G1_ROBOT_IP=<verified-G1-IP>' >&2
  echo 'Optional: G1_ROS_WIFI_INTERFACE=<Marc-WiFi>; G1_ROS_PEERS=<Kim-IP>' >&2
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
# Opt-in Architecture A mode: only ROS 2 uses Wi-Fi; Unitree SDK2 uses $IFACE.
# Do not combine Wi-Fi and Ethernet inside ROS 2 CycloneDDS: the working
# Kim->Marc test selected Wi-Fi with static unicast discovery.
ROS_WIFI_IFACE="${G1_ROS_WIFI_INTERFACE:-}"
if [[ -n "${G1_ROS_EXTRA_INTERFACES:-}" ]]; then
  echo 'G1_ROS_EXTRA_INTERFACES is obsolete for Architecture A.' >&2
  echo 'Use G1_ROS_WIFI_INTERFACE for Wi-Fi-only ROS 2 discovery.' >&2
  exit 2
fi
if [[ -z "$ROS_WIFI_IFACE" && -n "${G1_ROS_PEERS:-}" ]]; then
  echo 'G1_ROS_PEERS requires G1_ROS_WIFI_INTERFACE.' >&2
  exit 2
fi
if [[ -n "$ROS_WIFI_IFACE" ]]; then
  [[ -n "${G1_ROS_PEERS:-}" ]] || { echo 'Specify G1_ROS_PEERS for Wi-Fi mode.' >&2; exit 2; }
  [[ "$ROS_WIFI_IFACE" =~ ^[a-zA-Z0-9_.:-]+$ && "$ROS_WIFI_IFACE" != lo && "$ROS_WIFI_IFACE" != "$IFACE" ]] || {
    echo "Invalid Wi-Fi interface: $ROS_WIFI_IFACE" >&2; exit 2;
  }
  [[ -d "/sys/class/net/$ROS_WIFI_IFACE" ]] || { echo "Missing Wi-Fi interface: $ROS_WIFI_IFACE" >&2; exit 2; }
  [[ "$(cat "/sys/class/net/$ROS_WIFI_IFACE/operstate")" == up ]] || {
    echo "Wi-Fi interface $ROS_WIFI_IFACE is not UP." >&2; exit 2;
  }
  ip -4 -o addr show dev "$ROS_WIFI_IFACE" | grep -q 'inet ' || {
    echo "Wi-Fi interface $ROS_WIFI_IFACE has no IPv4 address." >&2; exit 2;
  }

  # Validate robot routing if a verified robot IP is supplied. In Marc's
  # overlapping 192.168.123.0/24 setup, routes MUST be checked individually.
  if [[ -n "${G1_ROBOT_IP:-}" ]]; then
    robot_dev="$(ip -4 route get "$G1_ROBOT_IP" 2>/dev/null |
      awk '{for (i=1;i<=NF;i++) if ($i=="dev") {print $(i+1);exit}}' || true)"
    [[ "$robot_dev" == "$IFACE" ]] || {
      echo "G1_ROBOT_IP=$G1_ROBOT_IP routes via $robot_dev, not G1 Ethernet $IFACE." >&2; exit 2;
    }
  fi

  # With multicast disabled, include Marc's own Wi-Fi IP in the peer list
  # so separate local ROS processes can discover one another. This uses only
  # the supported <Peer Address="..."/> syntax (no AddLocalhost attribute).
  ROS_WIFI_IPV4="$(ip -4 -o addr show dev "$ROS_WIFI_IFACE" |
    awk '$3 == "inet" { split($4, a, "/"); print a[1]; exit }')"
  [[ -n "$ROS_WIFI_IPV4" ]] || { echo "Wi-Fi interface $ROS_WIFI_IFACE has no IPv4 address." >&2; exit 2; }
  peer_xml="<Peer Address=\"${ROS_WIFI_IPV4}\"/>"
  peer_count=0
  for peer in ${G1_ROS_PEERS}; do
    # Validate as a unicast IPv4 literal; prevents malformed XML injection.
    /usr/bin/python3 - "$peer" <<'PYIP' || { echo "Invalid ROS peer IPv4: $peer" >&2; exit 2; }
import ipaddress, sys
try:
    ip = ipaddress.IPv4Address(sys.argv[1])
    if ip.is_loopback or ip.is_multicast or ip.is_unspecified or ip.is_link_local:
        raise ValueError('not a unicast LAN address')
    if str(ip) == '255.255.255.255':
        raise ValueError('broadcast')
except ValueError:
    raise SystemExit(1)
PYIP
    # Reject the old failure where Marc sent Kim's traffic out the G1 cable.
    peer_dev="$(ip -4 route get "$peer" 2>/dev/null |
      awk '{for (i=1;i<=NF;i++) if ($i=="dev") {print $(i+1);exit}}' || true)"
    [[ "$peer_dev" == "$ROS_WIFI_IFACE" ]] || {
      echo "ROS peer $peer routes via '${peer_dev:-unreachable}', not Wi-Fi '$ROS_WIFI_IFACE'." >&2
      echo "On Marc HOST, fix its /32 Wi-Fi route before starting Docker pipeline." >&2
      exit 2
    }
    peer_xml+="<Peer Address=\"${peer}\"/>"
    (( peer_count+=1 ))
    (( peer_count <= 16 )) || { echo 'Too many discovery peers (max 16).' >&2; exit 2; }
  done
  # The installed ROS Humble CycloneDDS rejects Peers AddLocalhost.
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${ROS_WIFI_IFACE}\"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>64</MaxAutoParticipantIndex><Peers>${peer_xml}</Peers></Discovery></Domain></CycloneDDS>"
  export ROS_LOCALHOST_ONLY=0
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  echo "ROS 2: Wi-Fi-only ${ROS_WIFI_IFACE}; peers ${G1_ROS_PEERS}; unicast discovery."
  echo 'Checking local ROS 2 middleware initialization (does not prove Kim is publishing)...'
  (
    set +u
    source /opt/ros/humble/setup.bash
    set -u
    /usr/bin/python3 - <<'PYROS'
import rclpy
rclpy.init()
try:
    node = rclpy.create_node('g1_ros_wifi_preflight')
    node.destroy_node()
finally:
    rclpy.shutdown()
PYROS
  ) || { echo 'ROS 2 Wi-Fi preflight failed; no motor controller started.' >&2; exit 1; }
else
  # Unchanged original behavior when the Wi-Fi mode is not selected.
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${IFACE}\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
fi
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
