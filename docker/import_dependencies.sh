#!/usr/bin/env bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

REPOS_FILE="${PROJECT_DIR}/ros2.repos"
WORKSPACE_SRC="${PROJECT_DIR}/ros2_ws/src"

if [ ! -f "${REPOS_FILE}" ]; then
    echo "Error: dependency manifest not found at ${REPOS_FILE}"
    exit 1
fi

if [ ! -d "${WORKSPACE_SRC}" ]; then
    echo "Error: ROS 2 workspace source directory not found at ${WORKSPACE_SRC}"
    exit 1
fi

USER_UID="$(id -u)"
USER_GID="$(id -g)"

IMAGE_NAME="g1-moveit:humble-${USER_UID}-${USER_GID}"

if ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
    echo "Error: Docker image ${IMAGE_NAME} does not exist."
    echo "Run ./docker/start.sh first to build the development image."
    exit 1
fi

echo "Importing ROS 2 dependencies from ${REPOS_FILE}..."

docker run --rm \
    --mount type=bind,source="${REPOS_FILE}",target=/tmp/ros2.repos,readonly \
    --mount type=bind,source="${WORKSPACE_SRC}",target=/workspace/src \
    "${IMAGE_NAME}" \
    bash -c 'vcs import /workspace/src < /tmp/ros2.repos'

echo "ROS 2 dependencies imported successfully."
