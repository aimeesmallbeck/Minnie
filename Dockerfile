# Aimee Robot ROS2 Environment
# Pre-built Docker image for Arduino UNO Q (ARM64/aarch64)
# Usage:
#   docker build -t aimee-robot:latest .
#   docker compose up -d

FROM ros:humble-ros-base

LABEL maintainer="Aimee Project"
LABEL description="Aimee Robot ROS2 runtime environment"

ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1

# ─────────────────────────────── System Dependencies ───────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ROS2 build tools
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    python3-vcstool \
    build-essential \
    cmake \
    git \
    # Audio / Video
    alsa-utils \
    libportaudio2 \
    libv4l-dev \
    v4l-utils \
    ffmpeg \
    # Serial
    libserialport0 \
    # Camera & CV
    libopencv-dev \
    python3-opencv \
    # Networking
    curl \
    wget \
    iputils-ping \
    # General
    usbutils \
    udev \
    && rm -rf /var/lib/apt/lists/*

# ─────────────────────────────── ROS2 Packages ───────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-compressed-image-transport \
    ros-humble-usb-cam \
    ros-humble-v4l2-camera \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-nav-msgs \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# ─────────────────────────────── Python Dependencies ───────────────────────────────
# Upgrade pip first: older system pip doesn't support --break-system-packages
RUN pip3 install --no-cache-dir --upgrade pip "setuptools<80" wheel
RUN pip3 install --no-cache-dir --break-system-packages \
    aiohttp==3.13.5 \
    flask \
    gtts \
    pygame==2.6.1 \
    vosk==0.3.45 \
    pykokoro==0.6.5 \
    kokorog2p==0.6.7 \
    paho-mqtt==2.1.0 \
    websocket-client==1.9.0 \
    requests==2.33.1 \
    "numpy<2" \
    opencv-python-headless==4.13.0.92 \
    python-osc \
    pillow \
    pyserial \
    pyyaml

# ─────────────────────────────── Initialize rosdep ───────────────────────────────
RUN rosdep init 2>/dev/null || true && rosdep update

# ─────────────────────────────── Workspace Setup ───────────────────────────────
# The actual workspace source is bind-mounted at runtime.
# We pre-create the directory and set up the ROS2 environment.
WORKDIR /workspace

# Add ROS2 setup to bashrc for interactive shells
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Default command: build (if needed) and drop to shell
CMD ["/bin/bash", "-c", "\
    source /opt/ros/humble/setup.bash && \
    if [ ! -f /workspace/install/setup.bash ]; then \
        echo 'Building AIMEE workspace...' && \
        cd /workspace && \
        rosdep install --from-paths src --ignore-src -y 2>/dev/null || true && \
        colcon build --symlink-install 2>&1 | tail -20; \
    fi && \
    source /workspace/install/setup.bash 2>/dev/null || true && \
    bash \
"]
