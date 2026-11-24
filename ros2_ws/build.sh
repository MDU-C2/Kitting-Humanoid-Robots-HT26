#!/usr/bin/env bash
# Builds the KARIS ROS 2 packages for THIS container (KARIS_ROLE decides which and where):
#   perception container  -> karis_interfaces karis_perception karis_bringup   (build_perception/, install_perception/)
#   Isaac container       -> karis_interfaces karis_isaac_fp                   (build_isaac/, install_isaac/)
#   MoveIt container      -> the KARIS pose listener: karis_interfaces karis_task karis_bringup
#     (the motion team's container; it and its MoveIt stack live on their branch
#      feature/g1-xr-clean-deployment; same build/ install/ as theirs, --symlink-install like it)
# Run inside the container:  bash <repo>/ros2_ws/build.sh   (extra args go to colcon)
set -e
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS"
source /opt/ros/humble/setup.bash
# The containers run as root on a repo owned by your user; git (called by colcon/ament) then
# refuses the repo ("dubious ownership"). Allow this repo once, in this container's git config.
if command -v git >/dev/null; then
    _repo="$(dirname "$WS")"
    git config --global --get-all safe.directory 2>/dev/null | grep -qxF "$_repo" \
        || git config --global --add safe.directory "$_repo"
fi
EXTRA=()
case "${KARIS_ROLE:-}" in
    perception) B=build_perception; I=install_perception; L=log_perception
                P="karis_interfaces karis_perception karis_bringup" ;;
    isaac)      B=build_isaac; I=install_isaac; L=log_isaac
                P="karis_interfaces karis_isaac_fp" ;;
    *)          B=build; I=install; L=log; EXTRA=(--symlink-install)
                P="karis_interfaces karis_task karis_bringup" ;;
esac
# own log folder per container too: the perception/Isaac containers run as root, the team's
# MoveIt container as your user, which cannot write into a root-owned log/
colcon --log-base "$L" build --build-base "$B" --install-base "$I" "${EXTRA[@]}" --packages-select $P "$@"
echo
echo "Built for '${KARIS_ROLE:-motion}' into $I. New terminals pick it up; in this one:  source $WS/karis_ros_env.sh"
