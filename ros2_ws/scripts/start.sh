#!/usr/bin/env bash
# Enter or create the persistent development container. NO physical robot setup.
# Never stop, remove, or restart an existing container from this script.
set -Eeuo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
WS="$ROOT/ros2_ws"
IMAGE="g1-xr-clean:humble-$(id -u)-$(id -g)"
NAME='g1_xr_clean_humble'

command -v docker >/dev/null || { echo 'Docker is not installed.' >&2; exit 1; }
# Keep the established check for the older potentially competing G1 container.
if docker ps --format '{{.Names}}' | grep -Exq 'g1_project_humble'; then
  echo 'Another G1 container is running. Refusing potentially competing control.' >&2
  echo 'Use the site-approved procedure to establish that physical control has ended.' >&2
  exit 1
fi

# Reattach immediately: network/DDS configuration is no longer tied to Docker startup.
if docker container inspect "$NAME" >/dev/null 2>&1; then
  state="$(docker inspect -f '{{.State.Running}}' "$NAME")"
  if [[ "$state" == true ]]; then
    echo "Reusing existing container: $NAME (not stopping or recreating it)."
    exec docker exec -it "$NAME" bash
  fi
  echo "Container $NAME exists but is stopped; refusing to change its state automatically." >&2
  echo "After verifying the robot is safe, manually inspect it and run: docker start $NAME" >&2
  echo "Or explicitly remove the stopped container yourself if you need a fresh one." >&2
  exit 1
fi

docker image inspect "$IMAGE" >/dev/null 2>&1 || {
  echo 'Image missing; run ros2_ws/scripts/install.sh first.' >&2; exit 1;
}
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

echo 'Creating persistent Docker container; no G1 network or DDS binding yet.'
echo 'Later, from inside Docker, run: bash /workspace/scripts/run_g1.sh --check'
echo 'Physical activation uses: bash /workspace/scripts/run_g1.sh'
docker run -d --network host --name "$NAME" \
  --mount "type=bind,source=$WS,target=/workspace" \
  "${GUI[@]}" \
  -e 'RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' \
  "$IMAGE" sleep infinity
echo "Docker ready. Reconnect anytime: docker exec -it $NAME bash"
exec docker exec -it "$NAME" bash
