#!/usr/bin/env bash
# Offline/non-actuating installation. Builds image, clones pinned XR/SDK,
# installs the verified opt-in XR patch, builds MoveIt workspace, runs tests.
set -Eeuo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
WS="$ROOT/ros2_ws"
SRC="$WS/src"
IMAGE="g1-xr-clean:humble-$(id -u)-$(id -g)"
REPOS="$WS/deployment/ros2.repos"
CONTAINER='g1_xr_clean_humble'
command -v docker >/dev/null || { echo 'Docker is required' >&2; exit 1; }
command -v git >/dev/null || { echo 'Git is required' >&2; exit 1; }
# Building a Docker image on the same host may disturb a live control session.
if [[ -n "$(docker ps --format '{{.Names}}' | grep -Ex '(g1_project_humble|g1_xr_clean_humble)' || true)" ]]; then
  echo 'Refusing to rebuild while a G1 development/clean container is running.' >&2
  echo 'Complete the independently supervised robot shutdown first.' >&2
  exit 1
fi
[[ -f "$SRC/g1_xr_bridge/patches/xr_headless_moveit.patch" ]] || { echo 'Missing committed XR patch' >&2; exit 1; }
echo 'Building the isolated XR/ROS image (NO robot connection)...'
docker build --network host \
  --build-arg "USERNAME=$(id -un)" \
  --build-arg "USER_UID=$(id -u)" \
  --build-arg "USER_GID=$(id -g)" \
  -t "$IMAGE" "$WS/deployment"
echo 'Importing pinned XR and Unitree Python repositories...'
docker run --rm --network host \
  --mount "type=bind,source=$SRC,target=/workspace/src" \
  --mount "type=bind,source=$REPOS,target=/tmp/ros2.repos,readonly" \
  "$IMAGE" bash -c 'vcs import --skip-existing /workspace/src < /tmp/ros2.repos'
# Verify checkout identity instead of silently resetting a modified repo.
while read -r name commit; do
  [[ -n "$name" && "$name" != \#* ]] || continue
  actual="$(git -C "$SRC/$name" rev-parse HEAD)" || exit 1
  if [[ "$actual" != "$commit" ]]; then
    echo "Pinned checkout mismatch: $name: expected $commit, found $actual" >&2
    exit 1
  fi
  echo "Verified $name @ ${commit:0:12}"
done < "$WS/deployment/revisions.lock"
echo 'Applying/verifying the known XR headless patch (offline, NO motor publisher)...'
docker run --rm --network none \
  --mount "type=bind,source=$WS,target=/workspace" \
  "$IMAGE" bash /workspace/src/g1_xr_bridge/verify_and_apply_xr.sh --apply
# G1_MOVEIT_DIRECT_INSTALL_V1: verify the FULL automatic patch chain.
# A reverse check of the original patch alone is invalid after an overlay.
docker run --rm --network none \
  --mount "type=bind,source=$WS,target=/workspace" \
  "$IMAGE" bash /workspace/src/g1_xr_bridge/verify_and_apply_xr.sh
echo 'Building MoveIt ROS workspace and running offline tests...'
docker run --rm --network none \
  --mount "type=bind,source=$WS,target=/workspace" \
  "$IMAGE" bash -c '
    set -e
    bash /workspace/deployment/docker/initialize_workspace.sh
    PYTHONPATH=/workspace/src/g1_xr_bridge \
      /usr/bin/python3 -m unittest discover -v \
      -s /workspace/tests -p "test_*.py"
  '
echo 'PASS: clean deployment installed and offline tests completed.'
echo 'No physical G1 commands were sent. Use ros2_ws/scripts/start.sh to open Docker.'
