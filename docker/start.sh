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

if [ ! -d "${WORKSPACE_DIR}" ]; then
    echo "Error: ROS 2 workspace not found at ${WORKSPACE_DIR}"
    exit 1
fi

XAUTH_ARGS=()
if [ -n "${XAUTHORITY_VALUE}" ] && [ -f "${XAUTHORITY_VALUE}" ]; then
    XAUTH_ARGS=(--mount type=bind,source="${XAUTHORITY_VALUE}",target=/tmp/.docker.xauth,readonly
                -e XAUTHORITY=/tmp/.docker.xauth)
else
    # WSLg (Windows) has no XAUTHORITY; on a normal Linux desktop RViz needs it
    echo "Warning: XAUTHORITY is not set or not a file; GUI programs may not open."
fi

# ROS 2 talks to the robot and the perception container over DDS: the container
# must share the host network (multicast discovery does not pass Docker's bridge).
ROS_ARGS=(--network host --ipc host
          -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
          -e KARIS_NET_IF="${KARIS_NET_IF:-}"
          -e KARIS_DDS_PEERS="${KARIS_DDS_PEERS:-}")

echo "Building Docker image ${IMAGE_NAME}..."

docker build \
    --build-arg USERNAME="${USERNAME}" \
    --build-arg USER_UID="${USER_UID}" \
    --build-arg USER_GID="${USER_GID}" \
    -t "${IMAGE_NAME}" \
    -f "${SCRIPT_DIR}/moveit/Dockerfile" \
    "${PROJECT_DIR}"

if docker container inspect "${CONTAINER_NAME}" >/dev/null 2>&1; then
    echo "Removing existing container ${CONTAINER_NAME}..."
    docker rm -f "${CONTAINER_NAME}"
fi

echo "Starting ${CONTAINER_NAME}..."

docker run -it \
    --name "${CONTAINER_NAME}" \
    --mount type=bind,source="${WORKSPACE_DIR}",target=/workspace \
    --mount type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix,readonly \
    "${XAUTH_ARGS[@]}" \
    "${ROS_ARGS[@]}" \
    -e DISPLAY="${DISPLAY_VALUE}" \
    "${IMAGE_NAME}" \
    bash
