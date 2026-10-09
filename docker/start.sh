#!/usr/bin/env bash

set -e

USERNAME="$(id -un)"
USER_UID="$(id -u)"
USER_GID="$(id -g)"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="${PROJECT_DIR}/ros2_ws"

DISPLAY_VALUE="${DISPLAY:-:0}"
XAUTHORITY_VALUE="${XAUTHORITY:-}"

IMAGE_NAME="g1-moveit:humble-${USER_UID}-${USER_GID}"
CONTAINER_NAME="g1_project_humble"

G1_ROBOT_IP="192.168.123.161"

G1_NETWORK_INTERFACE="$(
    ip route get "${G1_ROBOT_IP}" 2>/dev/null |
    awk '{for (i = 1; i <= NF; i++) if ($i == "dev") {print $(i+1); exit}}'
)"

if [ -n "${G1_NETWORK_INTERFACE}" ] &&
   ! ping -I "${G1_NETWORK_INTERFACE}" -c 1 -W 1 "${G1_ROBOT_IP}" >/dev/null 2>&1; then
    G1_NETWORK_INTERFACE=""
fi

DOCKER_ROS_NETWORK_ARGS=()

if [ -n "${G1_NETWORK_INTERFACE}" ]; then
    echo "G1 network detected on interface: ${G1_NETWORK_INTERFACE}"

    CYCLONEDDS_URI_VALUE="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${G1_NETWORK_INTERFACE}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"

    DOCKER_ROS_NETWORK_ARGS+=(
        -e "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
        -e "CYCLONEDDS_URI=${CYCLONEDDS_URI_VALUE}"
    )
else
    echo "G1 network not detected."
    echo "Starting container without G1-specific DDS configuration."
fi

if [ ! -d "${WORKSPACE_DIR}" ]; then
    echo "Error: ROS 2 workspace not found at ${WORKSPACE_DIR}"
    exit 1
fi

DOCKER_GUI_ARGS=()

IS_WSL=false
if [ -n "${WSL_INTEROP:-}" ] || grep -qi microsoft /proc/sys/kernel/osrelease 2>/dev/null; then
    IS_WSL=true
fi

if [ ! -d /tmp/.X11-unix ]; then
    echo "Error: /tmp/.X11-unix does not exist."
    echo "GUI applications such as RViz cannot be started."
    exit 1
fi

DOCKER_GUI_ARGS+=(
    --mount type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix,readonly
    -e "DISPLAY=${DISPLAY_VALUE}"
)

if [ -n "${XAUTHORITY_VALUE}" ] && [ -f "${XAUTHORITY_VALUE}" ]; then
    echo "Using X11 authentication file: ${XAUTHORITY_VALUE}"
    DOCKER_GUI_ARGS+=(
        --mount type=bind,source="${XAUTHORITY_VALUE}",target=/tmp/.docker.xauth,readonly
        -e XAUTHORITY=/tmp/.docker.xauth
    )
elif [ "${IS_WSL}" = true ] && [ -n "${WAYLAND_DISPLAY:-}" ]; then
    echo "WSLg detected; using its X11 socket without XAUTHORITY."
else
    echo "Error: XAUTHORITY is not set or does not point to a valid file."
    echo "GUI applications such as RViz cannot be started safely."
    exit 1
fi

echo "Building Docker image ${IMAGE_NAME}..."

docker build \
    --network host \
    --build-arg USERNAME="${USERNAME}" \
    --build-arg USER_UID="${USER_UID}" \
    --build-arg USER_GID="${USER_GID}" \
    -t "${IMAGE_NAME}" \
    "${PROJECT_DIR}"

# Import/verify pinned source repositories from ros2.repos.
"${SCRIPT_DIR}/import_dependencies.sh"

if docker container inspect "${CONTAINER_NAME}" >/dev/null 2>&1; then
    echo "Removing existing container ${CONTAINER_NAME}..."
    docker rm -f "${CONTAINER_NAME}"
fi

echo "Starting ${CONTAINER_NAME}..."

docker run -it \
    --network host \
    --name "${CONTAINER_NAME}" \
    -e "IMAGE_NAME=${IMAGE_NAME}" \
    --mount type=bind,source="${WORKSPACE_DIR}",target=/workspace \
    "${DOCKER_GUI_ARGS[@]}" \
    "${DOCKER_ROS_NETWORK_ARGS[@]}" \
    "${IMAGE_NAME}" \
    bash
