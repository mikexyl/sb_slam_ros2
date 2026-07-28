#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_ROOT="$(cd -- "${PACKAGE_ROOT}/../.." && pwd)"

RUN_ID="${RUN_ID:-a5678_split_$(date +%Y%m%d_%H%M%S_%z)}"
BAG_RATE="${BAG_RATE:-0.6}"
BAG_START_DELAY="${BAG_START_DELAY:-30.0}"
RUN_DURATION="${RUN_DURATION:--1}"
RERUN_HOST="${RERUN_HOST:-rerun+http://127.0.0.1:9876/proxy}"
OUTPUT_DIR="${OUTPUT_DIR:-${WORKSPACE_ROOT}/benchmark/graco_aerial_5678/${RUN_ID}/workstation}"
SPLIT_RMW_IMPLEMENTATION="${SPLIT_RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
FASTDDS_PROFILE="${FASTDDS_PROFILE:-${WORKSPACE_ROOT}/install/sb_slam_ros2/share/sb_slam_ros2/config/fastdds_workstation_to_jetson.xml}"

AERIAL_06_BAG_PATH="${AERIAL_06_BAG_PATH:-/data/graco/aerial-06-20m_ros2}"
AERIAL_07_BAG_PATH="${AERIAL_07_BAG_PATH:-/data/graco/aerial-07-25m_ros2}"
AERIAL_08_BAG_PATH="${AERIAL_08_BAG_PATH:-/data/graco/aerial-08-25m_ros2}"

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u

required_files=(
  "${AERIAL_06_BAG_PATH}/metadata.yaml"
  "${AERIAL_07_BAG_PATH}/metadata.yaml"
  "${AERIAL_08_BAG_PATH}/metadata.yaml"
  "${FASTDDS_PROFILE}"
)
for required_file in "${required_files[@]}"; do
  if [[ ! -s "${required_file}" ]]; then
    echo "Required file is missing or empty: ${required_file}" >&2
    exit 1
  fi
done

if ! pgrep -f nvidia-cuda-mps-control >/dev/null; then
  echo "CUDA MPS is not running on the workstation." >&2
  exit 1
fi

curl --silent --show-error --max-time 3 \
  "http://127.0.0.1:9876/" >/dev/null

mkdir -p "${OUTPUT_DIR}/ros_logs"
export SB_SLAM_ROS2_WS="${WORKSPACE_ROOT}"
export RMW_IMPLEMENTATION="${SPLIT_RMW_IMPLEMENTATION}"
export ROS_DOMAIN_ID
export ROS_LOCALHOST_ONLY=0
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_PROFILE}"
unset ZENOH_SESSION_CONFIG_URI
unset ZENOH_ROUTER_CONFIG_URI
unset ROS_DISCOVERY_SERVER
export ROS_LOG_DIR="${OUTPUT_DIR}/ros_logs"
export CUDA_MPS_PIPE_DIRECTORY="${CUDA_MPS_PIPE_DIRECTORY:-/tmp/nvidia-mps}"
export CUDA_MPS_LOG_DIRECTORY="${CUDA_MPS_LOG_DIRECTORY:-/tmp/nvidia-log}"

{
  echo "run_id=${RUN_ID}"
  echo "active_robot_ids=1,2,3"
  echo "bag_rate=${BAG_RATE}"
  echo "bag_start_delay=${BAG_START_DELAY}"
  echo "run_duration=${RUN_DURATION}"
  echo "rerun_host=${RERUN_HOST}"
  echo "rmw_implementation=${SPLIT_RMW_IMPLEMENTATION}"
  echo "ros_domain_id=${ROS_DOMAIN_ID}"
  echo "fastdds_profile=${FASTDDS_PROFILE}"
  echo "visualization_mode=minimal"
  echo "started_at=$(date --iso-8601=seconds)"
} >"${OUTPUT_DIR}/run_manifest.txt"

exec ros2 launch sb_slam_ros2 \
  graco_aerial_05_06_07_08_multi_robot.launch.py \
  active_robot_ids:=1,2,3 \
  aerial_06_bag_path:="${AERIAL_06_BAG_PATH}" \
  aerial_07_bag_path:="${AERIAL_07_BAG_PATH}" \
  aerial_08_bag_path:="${AERIAL_08_BAG_PATH}" \
  bag_rate:="${BAG_RATE}" \
  bag_start_delay:="${BAG_START_DELAY}" \
  bag_playback_duration:="${RUN_DURATION}" \
  log_output:=true \
  log_output_path:="${OUTPUT_DIR}/output" \
  rerun_recording_id:="${RUN_ID}" \
  rerun_host:="${RERUN_HOST}" \
  visualization_mode:=minimal \
  start_zenoh_router:=false
