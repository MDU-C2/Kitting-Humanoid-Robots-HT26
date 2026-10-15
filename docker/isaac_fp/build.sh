#!/usr/bin/env bash
# Builds the KARIS Isaac helper image (Isaac ROS 3.2 + FoundationPose with TensorRT).
#     bash docker/isaac_fp/build.sh
# 1. NVIDIA's Isaac ROS base image (isaac_ros_common, release-3.2; pulled from nvcr.io
#    when available, else built - large, 15-20 GB)
# 2. the KARIS layer on top (docker/isaac_fp/Dockerfile)
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARCH="$(uname -m)"
if [ "$ARCH" != "x86_64" ]; then
    echo "This builds the x86_64 image; on Jetson use Isaac ROS's own Jetson setup."; exit 1
fi
if [ ! -d "$HERE/isaac_ros_common" ]; then
    git clone --depth 1 -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git "$HERE/isaac_ros_common"
fi
"$HERE/isaac_ros_common/scripts/build_image_layers.sh" \
    --image_key x86_64.ros2_humble --image_name karis/isaac_ros_base:3.2
docker build -t karis/isaac_fp:3.2 --build-arg BASE_IMAGE=karis/isaac_ros_base:3.2 "$HERE"
echo "Built karis/isaac_fp:3.2"
