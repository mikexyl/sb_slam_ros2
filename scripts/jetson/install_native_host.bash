#!/usr/bin/env bash
set -euo pipefail

if [[ "$(uname -m)" != aarch64 ]]; then
  echo "This task is only for the Jetson aarch64 host" >&2
  exit 1
fi

initial_trt_version="$(dpkg-query -W -f='${Version}' libnvinfer10)"
if [[ "${initial_trt_version}" != 10.16.* ]]; then
  echo "Expected JetPack 7.2 TensorRT 10.16, found ${initial_trt_version}" >&2
  exit 1
fi
if [[ ! -x /usr/local/cuda-13.2/bin/nvcc ]]; then
  echo "JetPack CUDA 13.2 compiler is unavailable" >&2
  exit 1
fi

sudo env DEBIAN_FRONTEND=noninteractive apt-get update
sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  ca-certificates \
  curl \
  software-properties-common
sudo add-apt-repository -y universe

if ! dpkg-query -W ros2-apt-source >/dev/null 2>&1; then
  ros_apt_source_version="$({
    curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest
  } | sed -n 's/.*"tag_name": "\([^"]*\)".*/\1/p' | head -n 1)"
  if [[ -z "${ros_apt_source_version}" ]]; then
    echo "Could not determine the current ros2-apt-source version" >&2
    exit 1
  fi
  ubuntu_codename="$(. /etc/os-release && printf '%s' "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")"
  ros_apt_source_deb="/tmp/ros2-apt-source_${ros_apt_source_version}.${ubuntu_codename}_all.deb"
  curl -fL \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ros_apt_source_version}/ros2-apt-source_${ros_apt_source_version}.${ubuntu_codename}_all.deb" \
    -o "${ros_apt_source_deb}"
  sudo dpkg -i "${ros_apt_source_deb}"
fi

sudo env DEBIAN_FRONTEND=noninteractive apt-get update
sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  build-essential \
  cmake \
  git \
  git-lfs \
  graphviz-dev \
  libboost-all-dev \
  libcereal-dev \
  libcurl4-openssl-dev \
  libeigen3-dev \
  libgflags-dev \
  libgoogle-glog-dev \
  libmetis-dev \
  libopenblas-dev \
  libpcl-dev \
  libspdlog-dev \
  libtbb-dev \
  libtiff-dev \
  libyaml-cpp-dev \
  ninja-build \
  nlohmann-json3-dev \
  pkg-config \
  unzip \
  wget \
  ros-jazzy-image-transport \
  ros-jazzy-interactive-markers \
  ros-jazzy-message-filters \
  ros-jazzy-pcl-conversions \
  ros-jazzy-pcl-msgs \
  ros-jazzy-rmw-zenoh-cpp \
  ros-jazzy-ros-base \
  ros-jazzy-rosbag2-storage-mcap \
  ros-jazzy-tf2-eigen

final_trt_version="$(dpkg-query -W -f='${Version}' libnvinfer10)"
if [[ "${final_trt_version}" != "${initial_trt_version}" ]]; then
  echo "TensorRT changed unexpectedly: ${initial_trt_version} -> ${final_trt_version}" >&2
  exit 1
fi

printf 'Native host ready: CUDA 13.2, TensorRT %s, ROS Jazzy\n' "${final_trt_version}"
