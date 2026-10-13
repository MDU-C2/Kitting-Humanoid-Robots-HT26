#!/usr/bin/env bash
# Enters the clean Docker container. DOES NOT actuate robot or start XR.
set -Eeuo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
WS="$ROOT/ros2_ws"
IMAGE="g1-xr-clean:humble-$(id -u)-$(id -g)"
NAME='g1_xr_clean_humble'
docker image inspect "$IMAGE" >/dev/null 2>&1 || { echo 'Image missing; run ros2_ws/scripts/install.sh' >&2; exit 1; }
if docker container inspect "$NAME" >/dev/null 2>&1; then
  echo "Container $NAME already exists. Refusing to stop or remove it automatically." >&2
  echo 'If stopped, remove it explicitly AFTER checking XR released control:' >&2
  echo "  docker container rm $NAME" >&2
  exit 1
fi
if [[ -n "$(docker ps --format '{{.Names}}' | grep -Ex 'g1_project_humble' || true)" ]]; then
  echo 'Development G1 container is running. Refusing possible competing control.' >&2
  echo 'Shut down the original XR supervisor using the established site procedure.' >&2
  exit 1
fi
[[ -n "${DISPLAY:-}" && -d /tmp/.X11-unix ]] || {
  echo 'X11 display unavailable; RViz requires an X11 session.' >&2; exit 1;
}
GUI=(--mount type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix,readonly
     -e "DISPLAY=$DISPLAY")
if [[ -n "${XAUTHORITY:-}" && -f "${XAUTHORITY:-}" ]]; then
  GUI+=(--mount "type=bind,source=$XAUTHORITY,target=/tmp/.docker.xauth,readonly"
        -e XAUTHORITY=/tmp/.docker.xauth)
elif [[ -n "${WSL_INTEROP:-}" || "$(cat /proc/sys/kernel/osrelease)" == *icrosoft* ]]; then
  echo 'WSLg X11 socket detected without XAUTHORITY.'
else
  echo 'Valid XAUTHORITY required for RViz (or use WSLg).' >&2; exit 1
fi
# Preserve the validated development launcher network selection. A selected
# G1_INTERFACE is passed to XR, while CYCLONEDDS_URI binds ROS 2 discovery to
# that same reachable Ethernet interface. Merely setting RMW_IMPLEMENTATION
# is insufficient when this computer has multiple network interfaces.
G1_ROBOT_IP_VALUE="${G1_ROBOT_IP:-192.168.123.161}"
G1_INTERFACE_VALUE="${G1_INTERFACE:-}"
if [[ -z "$G1_INTERFACE_VALUE" ]]; then
  G1_INTERFACE_VALUE="$(ip route get "$G1_ROBOT_IP_VALUE" 2>/dev/null |
    awk '{for (i=1;i<=NF;i++) if ($i=="dev") {print $(i+1);exit}}' || true)"
fi
DDS_ARGS=(-e 'RMW_IMPLEMENTATION=rmw_cyclonedds_cpp')
if [[ -n "$G1_INTERFACE_VALUE" && "$G1_INTERFACE_VALUE" =~ ^[a-zA-Z0-9_.:-]+$ ]] &&
    ping -I "$G1_INTERFACE_VALUE" -c 1 -W 1 "$G1_ROBOT_IP_VALUE" >/dev/null 2>&1; then
  echo "Verified G1 network: $G1_INTERFACE_VALUE -> $G1_ROBOT_IP_VALUE"
  DDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${G1_INTERFACE_VALUE}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"
  DDS_ARGS+=(-e "CYCLONEDDS_URI=$DDS_URI")
else
  echo 'G1 network not verified; container starts without robot-specific DDS configuration.' >&2
  echo 'Physical activation will be blocked until a reachable G1 interface is configured.' >&2
  G1_INTERFACE_VALUE='unconfigured'
fi
echo 'Starting persistent Docker container; NO motor publisher will be started.'
echo 'Leaving the shell will NOT stop this container or terminate other sessions.'
docker run -d --network host --name "$NAME" \
  --mount "type=bind,source=$WS,target=/workspace" \
  "${GUI[@]}" \
  -e "G1_INTERFACE=$G1_INTERFACE_VALUE" \
  "${DDS_ARGS[@]}" \
  "$IMAGE" sleep infinity
echo "Docker ready. To reconnect later: docker exec -it $NAME bash"
docker exec -it "$NAME" bash
