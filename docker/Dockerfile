FROM cuda-ros-base:humble

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV SB_SLAM_ROS2_WS=/workspaces/sb_slam_ros2_ws
ENV PATH=/usr/local/bin:/root/.pixi/bin:${PATH}

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    ca-certificates \
    curl \
    git \
    libfaiss-dev \
    unzip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-interactive-markers \
  && rm -rf /var/lib/apt/lists/*

RUN rosdep init 2>/dev/null || true
RUN curl -fsSL https://pixi.sh/install.sh | bash \
  && install -m 0755 /root/.pixi/bin/pixi /usr/local/bin/pixi

COPY docker/entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

WORKDIR ${SB_SLAM_ROS2_WS}/src/sb_slam_ros2

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
