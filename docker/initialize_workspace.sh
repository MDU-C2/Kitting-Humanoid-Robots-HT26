#!/usr/bin/env bash
# Non-actuating ROS workspace setup executed by docker/start.sh.
# No DDS subscriptions, network access, or robot commands are required.
set -euo pipefail

# ROS/ament generated setup scripts are not safe under bash nounset (-u).
# Temporarily disable it only while sourcing the ROS environments.
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
set +u
source /opt/ros/humble/setup.bash
set -u
cd /workspace

if [[ ! -f src/g1_moveit_config/package.xml ]]; then
    echo 'ERROR: g1_moveit_config/package.xml is missing under /workspace/src.' >&2
    exit 1
fi

if [[ ! -f src/g1_moveit_config/launch/xr_pipeline_moveit.launch.py ]]; then
    echo 'ERROR: XR MoveIt launch source is missing:' >&2
    echo '  /workspace/src/g1_moveit_config/launch/xr_pipeline_moveit.launch.py' >&2
    exit 1
fi

if [[ ! -f src/g1_moveit_config/config/moveit.rviz ]]; then
    echo 'ERROR: saved MoveIt RViz configuration is missing:' >&2
    echo '  /workspace/src/g1_moveit_config/config/moveit.rviz' >&2
    exit 1
fi

if [[ ! -x /opt/mamba/envs/xr/bin/python ]]; then
    echo 'ERROR: XR Micromamba Python is missing: /opt/mamba/envs/xr/bin/python' >&2
    exit 1
fi

# Validate the Python interpreter used by the original XR/Pinocchio code.
/opt/mamba/envs/xr/bin/python - <<'PY'
import pinocchio
print(f'XR Micromamba Pinocchio: {pinocchio.__version__}')
PY

# Ensure the ROS interpreter and core Python packages work independently
# of Micromamba. Building is incremental with --symlink-install.
/usr/bin/python3 - <<'PY'
import rclpy
print('ROS 2 Python: OK')
PY

if ! command -v colcon >/dev/null 2>&1; then
    echo 'ERROR: colcon is missing from the Docker image.' >&2
    exit 1
fi

# A previous non-symlink colcon build can leave a real package directory
# where ament_cmake_python now needs to create a symlink. Clean only this
# known generated artifact; never touch /workspace/src or other build trees.
stale_unitree_hg="/workspace/build/unitree_hg/ament_cmake_python/unitree_hg/unitree_hg"
if [[ -d "${stale_unitree_hg}" && ! -L "${stale_unitree_hg}" ]]; then
    echo "Removing stale generated unitree_hg symlink target: ${stale_unitree_hg}"
    rm -r -- "${stale_unitree_hg}"
fi

echo 'Building installed ROS packages needed by MoveIt (incremental)...'
colcon build --symlink-install --packages-up-to g1_moveit_config

if [[ ! -f install/setup.bash ]]; then
    echo 'ERROR: colcon completed without /workspace/install/setup.bash' >&2
    exit 1
fi
set +u
source install/setup.bash
set -u

# Check *installed* artifacts, not just source, to catch the exact failure
# that previously occurred after switching development computers.
prefix="$(ros2 pkg prefix g1_moveit_config)" || {
    echo 'ERROR: g1_moveit_config is unavailable after sourcing workspace.' >&2
    exit 1
}
for relpath in \
    launch/xr_pipeline_moveit.launch.py \
    config/moveit.rviz; do
    if [[ ! -f "${prefix}/share/g1_moveit_config/${relpath}" ]]; then
        echo "ERROR: missing installed ${relpath}" >&2
        echo "Expected under ${prefix}/share/g1_moveit_config" >&2
        echo 'Check g1_moveit_config/CMakeLists.txt install(DIRECTORY ...) rules.' >&2
        exit 1
    fi
done

echo "PASS: ROS 2 workspace ready; g1_moveit_config installed at ${prefix}"
echo 'Robot actuation: NOT USED by workspace initialization.'
