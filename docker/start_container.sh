#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
HOST_WORKSPACE_ROOT="${SB_SLAM_ROS2_WS_HOST:-$(cd -- "${PACKAGE_ROOT}/../.." && pwd)}"
CONTAINER_WORKSPACE_ROOT="${SB_SLAM_ROS2_WS_CONTAINER:-/workspaces/sb_slam_ros2_ws}"

IMAGE_NAME="${SB_SLAM_ROS2_IMAGE:-sb-slam-ros2:metapackage}"
CONTAINER_NAME="${SB_SLAM_ROS2_CONTAINER:-sb_slam_ros2}"
ROS_DISTRO="${ROS_DISTRO:-humble}"

if ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
  docker build -t "${IMAGE_NAME}" -f "${SCRIPT_DIR}/Dockerfile" "${PACKAGE_ROOT}"
fi

if ! docker container inspect "${CONTAINER_NAME}" >/dev/null 2>&1; then
  docker create \
    --name "${CONTAINER_NAME}" \
    --gpus all \
    --user "$(id -u):$(id -g)" \
    -e "HOME=/tmp" \
    -e "ROS_DISTRO=${ROS_DISTRO}" \
    -e "SB_SLAM_ROS2_WS=${CONTAINER_WORKSPACE_ROOT}" \
    -v "${HOST_WORKSPACE_ROOT}:${CONTAINER_WORKSPACE_ROOT}" \
    -w "${CONTAINER_WORKSPACE_ROOT}/src/sb_slam_ros2" \
    -it \
    "${IMAGE_NAME}" \
    bash >/dev/null
fi

if [[ "$(docker inspect -f '{{.State.Running}}' "${CONTAINER_NAME}")" != "true" ]]; then
  docker start "${CONTAINER_NAME}" >/dev/null
fi

tty_args=(-i)
if [[ -t 0 && -t 1 ]]; then
  tty_args=(-it)
fi

if [[ "$#" -eq 0 ]]; then
  docker exec "${tty_args[@]}" \
    -w "${CONTAINER_WORKSPACE_ROOT}/src/sb_slam_ros2" \
    "${CONTAINER_NAME}" \
    bash -lc 'source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"; if [[ -f "${SB_SLAM_ROS2_WS}/install/setup.bash" ]]; then source "${SB_SLAM_ROS2_WS}/install/setup.bash"; fi; exec bash'
else
  docker exec "${tty_args[@]}" \
    -w "${CONTAINER_WORKSPACE_ROOT}/src/sb_slam_ros2" \
    "${CONTAINER_NAME}" \
    bash -lc 'source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"; if [[ -f "${SB_SLAM_ROS2_WS}/install/setup.bash" ]]; then source "${SB_SLAM_ROS2_WS}/install/setup.bash"; fi; exec "$@"' \
    bash \
    "$@"
fi
