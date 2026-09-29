FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    nano \
    tmux \
    git \
    ros-humble-moveit \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
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

USER ${USERNAME}
