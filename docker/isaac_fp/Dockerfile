# KARIS Isaac helper image: NVIDIA Isaac ROS 3.2 (ROS 2 Humble, CUDA 12.6, TensorRT)
# + isaac_ros_foundationpose. It runs FoundationPose with TensorRT engines and serves
# /karis/fp/estimate_pose for the perception container (backend:=isaac).
#
# Build with docker/isaac_fp/build.sh (it first builds NVIDIA's own Isaac ROS base image
# from isaac_ros_common release-3.2, then this layer). Needs an x86_64 PC with an NVIDIA
# RTX GPU, driver >= 560 and the NVIDIA Container Toolkit.
ARG BASE_IMAGE=karis/isaac_ros_base:3.2
FROM ${BASE_IMAGE}

USER root
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-isaac-ros-foundationpose \
        ros-humble-isaac-ros-tensor-list-interfaces \
        ros-humble-vision-msgs \
        ros-humble-rmw-cyclonedds-cpp \
        python3-opencv python3-yaml iproute2 wget \
    && rm -rf /var/lib/apt/lists/*
# ROS 2 Humble's Python message libraries and Ubuntu's OpenCV are built for NumPy 1.x;
# the Isaac base image comes with NumPy 2 from pip, which makes them fail to import.
RUN python3 -m pip install --no-cache-dir "numpy<2"

COPY prepare_models.sh /opt/karis/prepare_models.sh
RUN chmod +x /opt/karis/prepare_models.sh

ENV KARIS_ROLE=isaac \
    KARIS_ISAAC_MODELS=/models \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RUN echo '[ -d /usr/lib/wsl/lib ] && export LD_LIBRARY_PATH=/usr/lib/wsl/lib:$LD_LIBRARY_PATH   # WSL GPU' >> /root/.bashrc \
    && echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
    && echo '[ -f "$KARIS_WS/karis_ros_env.sh" ] && source "$KARIS_WS/karis_ros_env.sh"' >> /root/.bashrc
WORKDIR /root
ENTRYPOINT []
CMD ["bash"]
