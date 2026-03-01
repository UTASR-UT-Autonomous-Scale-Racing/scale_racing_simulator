####################################################
#
#   AutoDRIVE Combined Dockerfile
#   Supports both Simulator and Devkit
#   Usage:
#     Simulator: docker run -e MODE=simulator ...
#     Devkit:    docker run -e MODE=devkit ...
#
####################################################

# ── Base: CUDA 11.8 on Ubuntu 22.04 ──────────────
FROM nvidia/cuda:11.8.0-base-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    XDG_RUNTIME_DIR=/tmp/runtime-root \
    MODE=devkit

# ── CUDA repo keys ────────────────────────────────
RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/3bf863cc.pub \
 && apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu2204/x86_64/7fa2af80.pub

# ── Common system packages ────────────────────────
RUN apt update && apt install -y --no-install-recommends \
        sudo wget gedit nano vim curl unzip net-tools \
        # Vulkan (replaces the nvidia/vulkan base image)
        libvulkan1 libvulkan-dev vulkan-tools \
        libc++1 libc++abi1 \
        # Display utilities
        xvfb ffmpeg libgdal-dev libsm6 libxext6 \
        # Python
        python3-pip \
        # Misc build tools needed later
        gnupg lsb-release ca-certificates \
 && rm -rf /var/lib/apt/lists/*

# ── Install ROS 2 Humble ──────────────────────────
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/ros2.list \
 && apt update && apt install -y --no-install-recommends \
        ros-humble-desktop \
        ros-humble-tf-transformations \
        ros-humble-imu-tools \
        python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

# ── Python dependencies (Devkit) ──────────────────
RUN pip3 install --no-cache-dir \
        attrdict \
        numpy==1.22.2 \
        pillow \
        opencv-contrib-python==4.10.0.84 \
        eventlet==0.33.3 \
        Flask==1.1.1 \
        Flask-SocketIO==4.1.0 \
        python-socketio==4.2.0 \
        python-engineio==3.13.0 \
        greenlet==1.1.0 \
        gevent==21.12.0 \
        gevent-websocket==0.10.1 \
        Jinja2==3.0.3 \
        itsdangerous==2.0.1 \
        werkzeug==2.0.3 \
        transforms3d

# ── AutoDRIVE Simulator ───────────────────────────
COPY autodrive_simulator /home/autodrive_simulator
RUN chmod +x "/home/autodrive_simulator/AutoDRIVE Simulator.x86_64"

# ── AutoDRIVE Devkit (ROS 2) ──────────────────────
COPY autodrive_devkit/. /home/autodrive_devkit/src/autodrive_devkit
RUN /bin/bash -c "source /opt/ros/humble/setup.bash \
    && cd /home/autodrive_devkit \
    && colcon build"

# Persist ROS sourcing for interactive shells
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
 && echo "source /home/autodrive_devkit/install/setup.bash" >> ~/.bashrc \
 && echo "export XDG_RUNTIME_DIR=/tmp/runtime-root" >> ~/.bashrc

# ── Entrypoint scripts ────────────────────────────
COPY autodrive_simulator.sh /home/autodrive_simulator.sh
COPY autodrive_devkit.sh    /home/autodrive_devkit.sh
RUN chmod +x /home/autodrive_simulator.sh /home/autodrive_devkit.sh

# ── Combined entrypoint ───────────────────────────
COPY <<'EOF' /home/entrypoint.sh
#!/bin/bash
set -e

echo "[entrypoint] Starting AutoDRIVE Simulator in background..."
/home/autodrive_simulator.sh &

echo "[entrypoint] Starting AutoDRIVE Devkit in foreground..."
source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash
exec /home/autodrive_devkit.sh "$@"
EOF
RUN chmod +x /home/entrypoint.sh

WORKDIR /home
EXPOSE 4567
ENTRYPOINT ["/home/entrypoint.sh"]