#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="${ISAAC_ROS_WS:-/workspaces/isaac_ros-dev}"
MODEL_DIR="${MODEL_DIR:-${WORKSPACE_ROOT}/models/jetpack_7.2_trt_10.16}"
BAG_PATH="${BAG_PATH:-/home/mikexyl/data/graco/ground-01}"
BAG_RATE="${BAG_RATE:-0.8}"
BAG_START_DELAY="${BAG_START_DELAY:-20.0}"
RUN_ID="${RUN_ID:-$(date +%Y%m%d_%H%M%S_%z)}"
OUTPUT_DIR="${OUTPUT_DIR:-${WORKSPACE_ROOT}/benchmark/graco_ground_01/${RUN_ID}}"

set +u
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u
mkdir -p "${OUTPUT_DIR}"

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ISAAC_ROS_WS="${WORKSPACE_ROOT}"

exec ros2 launch sb_slam_ros2 \
  graco_ground_01_jetson_benchmark.launch.py \
  bag_path:="${BAG_PATH}" \
  bag_rate:="${BAG_RATE}" \
  bag_start_delay:="${BAG_START_DELAY}" \
  models.xfeat:="${MODEL_DIR}/xfeat_320x224_fp16.engine" \
  models.lightglue_frontend:="${MODEL_DIR}/lg_320x224_dyn_n1_o500_m1024_fp16.engine" \
  models.lightglue_lcd:="${MODEL_DIR}/lg_320x224_dyn_n1_o500_m1024_fp16.engine" \
  models.jist:="${MODEL_DIR}/JIST_r18_512_seqgem_simplified_fp16.engine" \
  log_output_path:="${OUTPUT_DIR}"
