FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    nano \
    tmux \
    git \
    iproute2 \
    iputils-ping\
    python3-vcstool \
    ros-humble-moveit \
    ros-humble-pinocchio \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-generator-dds-idl \
    ros-humble-joint-state-publisher-gui \
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

RUN echo 'source /opt/ros/humble/setup.bash' >> /home/${USERNAME}/.bashrc \
    && echo 'if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi' >> /home/${USERNAME}/.bashrc \
    && chown ${USERNAME}:${USERNAME} /home/${USERNAME}/.bashrc

USER ${USERNAME}
