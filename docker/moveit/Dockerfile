FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    nano \
    tmux \
    git \
    ros-humble-moveit \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-image-transport-plugins \
    ros-humble-rqt-image-view \
    ros-humble-tf2-tools \
    iproute2 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

ARG USERNAME=rosdev
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} \
               --gid ${USER_GID} \
               --create-home \
               --shell /bin/bash \
               ${USERNAME}

USER ${USERNAME}

# KARIS ROS 2 setup in every terminal (ROS_DOMAIN_ID, CycloneDDS on the robot network,
# the built workspace), once ros2_ws is built: see Documents/Instructions/ROS2_pipeline.md
RUN echo '[ -f /workspace/karis_ros_env.sh ] && source /workspace/karis_ros_env.sh' >> /home/${USERNAME}/.bashrc
