#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

REPOS_FILE="${PROJECT_DIR}/ros2.repos"
WORKSPACE_SRC="${PROJECT_DIR}/ros2_ws/src"

IMAGE_NAME="g1-moveit:humble-$(id -u)-$(id -g)"

if [ ! -f "${REPOS_FILE}" ]; then
    echo "Error: ros2.repos not found."
    exit 1
fi

if [ ! -d "${WORKSPACE_SRC}" ]; then
    echo "Error: ROS 2 workspace source directory not found."
    exit 1
fi

if ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
    echo "Error: Docker image ${IMAGE_NAME} does not exist."
    exit 1
fi

echo "Checking external repositories..."

# Import only missing repositories.
docker run --rm \
    --network host \
    --mount type=bind,source="${REPOS_FILE}",target=/tmp/ros2.repos,readonly \
    --mount type=bind,source="${WORKSPACE_SRC}",target=/workspace/src \
    "${IMAGE_NAME}" \
    bash -c 'vcs import --skip-existing /workspace/src < /tmp/ros2.repos'

# Verify pinned revisions without modifying existing repositories.
python3 - "${REPOS_FILE}" "${WORKSPACE_SRC}" <<'VERIFY'
import subprocess
import sys
from pathlib import Path

repos_file = Path(sys.argv[1])
workspace_src = Path(sys.argv[2])

repos = {}
name = None

for line in repos_file.read_text().splitlines():
    if line.startswith("  ") and not line.startswith("    ") and line.strip().endswith(":"):
        name = line.strip()[:-1]
        repos[name] = {}
    elif name and line.strip().startswith("version:"):
        repos[name]["version"] = line.split(":", 1)[1].strip()
    elif name and line.strip().startswith("url:"):
        repos[name]["url"] = line.split(":", 1)[1].strip()

for name, config in repos.items():
    directory = workspace_src / name
    expected = config["version"]

    if not (directory / ".git").exists():
        raise SystemExit(f"Error: missing Git repository: {directory}")

    current = subprocess.check_output(
        ["git", "-C", str(directory), "rev-parse", "HEAD"],
        text=True,
    ).strip()

    # Supports both complete and abbreviated pinned hashes.
    if not current.startswith(expected):
        raise SystemExit(
            f"Error: {name} is at the wrong commit.\n"
            f"Expected: {expected}\n"
            f"Current:  {current}\n"
            "Existing checkouts are never reset automatically."
        )

    print(f"{name}: OK ({current[:12]})")

print("All external repositories verified.")
VERIFY
