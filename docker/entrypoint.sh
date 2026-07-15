#!/usr/bin/env bash
set -e

ROS_DISTRO="${ROS_DISTRO:-humble}"
SB_SLAM_ROS2_WS="${SB_SLAM_ROS2_WS:-/workspaces/sb_slam_ros2_ws}"

if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

if [[ -f "${SB_SLAM_ROS2_WS}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${SB_SLAM_ROS2_WS}/install/setup.bash"
fi

exec "$@"
