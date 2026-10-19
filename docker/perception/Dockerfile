# KARIS perception image: ROS 2 Humble + YOLO (Ultralytics, PyTorch) + RF-DETR (own venv).
# Ubuntu 22.04, Python 3.10. FoundationPose is not in this image: it runs in the Isaac
# helper (docker/isaac_fp, Isaac ROS FoundationPose with TensorRT).
#
# Built and started with docker/compose.yaml (service "perception"):
#     docker compose -f docker/compose.yaml build perception
# Nothing is compiled; the build is mostly downloads (10-30 min the first time).
#
# The GPU: PyTorch's wheels bring their own CUDA libraries, and the NVIDIA container
# toolkit adds the driver, so a plain Ubuntu base is enough (no CUDA image needed).
ARG BASE_IMAGE=ubuntu:22.04
FROM ${BASE_IMAGE}

ARG WITH_RFDETR=1
# Optional Ubuntu mirror for slow or unreliable networks, e.g. http://se.archive.ubuntu.com/ubuntu
# (docker compose: UBUNTU_MIRROR in docker/.env). Empty = Ubuntu's default servers.
ARG UBUNTU_MIRROR=""
ENV DEBIAN_FRONTEND=noninteractive \
    PIP_CONSTRAINT=/opt/karis/constraints.txt \
    PIP_DEFAULT_TIMEOUT=120 \
    PIP_RETRIES=10 \
    PIP_RESUME_RETRIES=20 \
    LANG=C.UTF-8 LC_ALL=C.UTF-8

# apt: retry and wait longer on a slow or dropping network, optionally use a nearer mirror
RUN printf 'Acquire::Retries "10";\nAcquire::http::Timeout "120";\nAcquire::https::Timeout "120";\n' \
        > /etc/apt/apt.conf.d/80-karis-retries \
    && if [ -n "${UBUNTU_MIRROR}" ]; then \
        sed -i -e "s#http://archive.ubuntu.com/ubuntu#${UBUNTU_MIRROR}#g" \
               -e "s#http://security.ubuntu.com/ubuntu#${UBUNTU_MIRROR}#g" /etc/apt/sources.list; \
    fi

# ---- ROS 2 Humble (ros-base) + what the KARIS nodes use ----------------------------
# build-essential + cmake: colcon builds karis_interfaces (generated C/C++ message code)
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg2 lsb-release ca-certificates \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
        > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-base \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-tf2-ros \
        ros-humble-vision-msgs \
        ros-humble-visualization-msgs \
        ros-humble-std-srvs \
        ros-humble-image-transport-plugins \
        ros-humble-rqt-image-view \
        python3-colcon-common-extensions \
        python3-pip python3-venv python3-dev \
        git wget nano iproute2 build-essential cmake \
        libgl1 libglib2.0-0 libgomp1 \
        libsm6 libxext6 libxrender1 libfontconfig1 libxkbcommon-x11-0 libxcb-xinerama0 \
        libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-randr0 libxcb-render-util0 libxcb-shape0 \
    && rm -rf /var/lib/apt/lists/*

# ---- Python packages (system Python 3.10, the one ROS uses) --------------------------
# One RUN per big download: a dropped connection only repeats that step, and the pip cache
# (a BuildKit cache, not part of the image) keeps the files that were already downloaded.
COPY constraints.txt /opt/karis/constraints.txt
RUN --mount=type=cache,target=/root/.cache/pip \
    python3 -m pip install --upgrade pip "setuptools<70" wheel \
    && python3 -m pip install --ignore-installed blinker
RUN --mount=type=cache,target=/root/.cache/pip \
    python3 -m pip install torch==2.4.1 torchvision==0.19.1 --index-url https://download.pytorch.org/whl/cu121
# numpy<2 (ROS), opencv, scipy + trimesh (meshes, drift check), pyyaml,
# pillow, lz4 (bag_reindex), pyrealsense2 (camera player for .bag), pytest + flake8 (tests)
RUN --mount=type=cache,target=/root/.cache/pip \
    python3 -m pip install "numpy<2" scipy pyyaml opencv-python trimesh pillow lz4 "pyrealsense2<2.56" pytest flake8
# YOLO (Ultralytics >= 8.4: YOLOv8 ... YOLO26)
RUN --mount=type=cache,target=/root/.cache/pip \
    python3 -m pip install "ultralytics>=8.4"
# fail the build here (not at the first frame) if something does not import
RUN python3 -c "import torch, torchvision, cv2, numpy, scipy, trimesh, yaml, lz4, ultralytics, pyrealsense2; \
print('perception deps OK: torch', torch.__version__, 'numpy', numpy.__version__, 'ultralytics', ultralytics.__version__)"

# ---- RF-DETR in its own venv (it wants newer torch/transformers) -----------------------
RUN --mount=type=cache,target=/root/.cache/pip \
    if [ "${WITH_RFDETR}" = "1" ]; then \
        python3 -m venv /opt/rfdetr-venv \
        && env -u PIP_CONSTRAINT /opt/rfdetr-venv/bin/pip install --upgrade pip \
        && env -u PIP_CONSTRAINT /opt/rfdetr-venv/bin/pip install "rfdetr>=1.9" ; \
    fi
ENV KARIS_RFDETR_PYTHON=/opt/rfdetr-venv/bin/python

# ---- environment in every shell ----------------------------------------------------
ENV KARIS_ROLE=perception \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RUN echo '[ -d /usr/lib/wsl/lib ] && export LD_LIBRARY_PATH=/usr/lib/wsl/lib:$LD_LIBRARY_PATH   # WSL GPU' >> /root/.bashrc \
    && echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
    && echo '[ -f "$KARIS_WS/karis_ros_env.sh" ] && source "$KARIS_WS/karis_ros_env.sh"' >> /root/.bashrc
WORKDIR /root
CMD ["bash"]
