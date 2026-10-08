FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
    nano tmux git iproute2 iputils-ping \
    python3-vcstool python3-pip python3-numpy python3-scipy python3-yaml \
    python3-colcon-common-extensions \
    build-essential \
    cmake \
    pkg-config \
    python3-dev \
    ros-humble-moveit ros-humble-pinocchio \
    ros-humble-ros2-control ros-humble-ros2-controllers \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-generator-dds-idl \
    ros-humble-joint-state-publisher-gui \
    ros-humble-control-msgs ros-humble-trajectory-msgs \
    ros-humble-sensor-msgs ros-humble-rclpy \
    && rm -rf /var/lib/apt/lists/*

# -------------------------------------------------------------------
# CycloneDDS compatibility prefix
#
# The Unitree Python SDK requires cyclonedds==0.10.2.
#
# ROS 2 Humble provides the native CycloneDDS library in a multiarch
# directory, while the Python package expects lib/libddsc.so.
#
# Create a compatibility prefix without installing a second native
# CycloneDDS library.
# -------------------------------------------------------------------

RUN mkdir -p /opt/cyclonedds-prefix/lib \
    /opt/cyclonedds-prefix/bin \
    && ln -s /opt/ros/humble/include \
        /opt/cyclonedds-prefix/include \
    && ln -s /opt/ros/humble/lib/x86_64-linux-gnu/libddsc.so \
        /opt/cyclonedds-prefix/lib/libddsc.so

ENV CYCLONEDDS_HOME=/opt/cyclonedds-prefix

# -------------------------------------------------------------------
# Python dependencies
#
# Keep NumPy 1.x to maintain compatibility with the Pinocchio
# binaries provided by ROS 2 Humble.
# -------------------------------------------------------------------

# Install the Python bindings against ROS Humble's native CycloneDDS.
RUN python3 -m pip install --no-cache-dir \
    --no-binary cyclonedds \
    "cyclonedds==0.10.2"

# Install the versions validated in our development container.
# --no-deps preserves Ubuntu's NumPy 1.21.5.
RUN python3 -m pip install --no-cache-dir --no-deps \
    "casadi==3.8.1" \
    "meshcat==0.3.2" \
    "opencv-python==4.6.0.66" \
    "logging-mp"

# Install Meshcat's additional dependencies without upgrading NumPy.
RUN python3 -m pip install --no-cache-dir --no-deps \
    pillow ipython u-msgpack-python pyzmq pyngrok tornado \
    traitlets jedi pexpect exceptiongroup prompt_toolkit \
    matplotlib-inline stack_data parso ptyprocess wcwidth \
    asttokens executing pure-eval


# Verify core Python dependencies during image build.
RUN python3 - <<'PY'
import numpy
import pinocchio
import rclpy
import cyclonedds
import casadi
import meshcat
import cv2

assert numpy.__version__.startswith("1."), \
    f"Incompatible NumPy version: {numpy.__version__}"

print("NumPy:", numpy.__version__)
print("Pinocchio:", pinocchio.__version__)
print("ROS 2: OK")
print("CycloneDDS: OK")
print("CasADi:", casadi.__version__)
print("Meshcat: OK")
print("OpenCV:", cv2.__version__)
PY

# -------------------------------------------------------------------
# Workspace
# -------------------------------------------------------------------

WORKDIR /workspace

ARG USERNAME=rosdev
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} \
               --create-home --shell /bin/bash ${USERNAME}

RUN echo 'source /opt/ros/humble/setup.bash' >> /home/${USERNAME}/.bashrc \
    && echo 'if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi' >> /home/${USERNAME}/.bashrc \
    && chown ${USERNAME}:${USERNAME} /home/${USERNAME}/.bashrc

USER ${USERNAME}
