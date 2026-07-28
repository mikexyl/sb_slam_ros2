#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="${ISAAC_ROS_WS:-/home/mikexyl/workspaces/isaac_ros-dev}"
MODEL_DIR="${MODEL_DIR:-${WORKSPACE_ROOT}/models/jetpack_7.2_trt_10.16}"
BAG_PATH="${BAG_PATH:-/home/mikexyl/data/graco/aerial-05-40m}"
RUN_ID="${RUN_ID:-a5678_split_$(date +%Y%m%d_%H%M%S_%z)}"
BAG_RATE="${BAG_RATE:-0.6}"
BAG_START_DELAY="${BAG_START_DELAY:-30.0}"
RUN_DURATION="${RUN_DURATION:--1}"
RERUN_HOST="${RERUN_HOST:-rerun+http://192.168.0.206:9876/proxy}"
OUTPUT_DIR="${OUTPUT_DIR:-${WORKSPACE_ROOT}/benchmark/graco_aerial_5678/${RUN_ID}/jetson}"
ROBOT_NAMES_FILE="${ROBOT_NAMES_FILE:-${WORKSPACE_ROOT}/install/kimera_distributed/share/kimera_distributed/params/robot_names_graco.yaml}"
SPLIT_RMW_IMPLEMENTATION="${SPLIT_RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
FASTDDS_PROFILE="${FASTDDS_PROFILE:-${WORKSPACE_ROOT}/install/sb_slam_ros2/share/sb_slam_ros2/config/fastdds_jetson_to_workstation.xml}"

set +u
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u

required_files=(
  "${BAG_PATH}/metadata.yaml"
  "${ROBOT_NAMES_FILE}"
  "${FASTDDS_PROFILE}"
  "${MODEL_DIR}/xfeat_320x224_fp16.engine"
  "${MODEL_DIR}/lg_320x224_dyn_n1_o500_m1024_fp16.engine"
  "${MODEL_DIR}/JIST_r18_512_seqgem_simplified_fp16.engine"
  "${MODEL_DIR}/interpolator_bilinear_640x480.onnx"
  "${MODEL_DIR}/interpolator_bicubic_640x480.onnx"
  "${MODEL_DIR}/interpolator_nearest_640x480.onnx"
)
for required_file in "${required_files[@]}"; do
  if [[ ! -s "${required_file}" ]]; then
    echo "Required file is missing or empty: ${required_file}" >&2
    exit 1
  fi
done

curl --silent --show-error --max-time 3 \
  "http://192.168.0.220:9876/" >/dev/null

export ISAAC_ROS_WS="${WORKSPACE_ROOT}"
export RMW_IMPLEMENTATION="${SPLIT_RMW_IMPLEMENTATION}"
export ROS_DOMAIN_ID
export ROS_LOCALHOST_ONLY=0
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_PROFILE}"
unset ZENOH_SESSION_CONFIG_URI
unset ZENOH_ROUTER_CONFIG_URI
unset ROS_DISCOVERY_SERVER
export CUDA_MPS_PIPE_DIRECTORY="${CUDA_MPS_PIPE_DIRECTORY:-/tmp/nvidia-mps}"
export CUDA_MPS_LOG_DIRECTORY="${CUDA_MPS_LOG_DIRECTORY:-/tmp/nvidia-log}"

if ! pgrep -f nvidia-cuda-mps-control >/dev/null; then
  mkdir -p "${CUDA_MPS_PIPE_DIRECTORY}" "${CUDA_MPS_LOG_DIRECTORY}"
  sudo env \
    CUDA_MPS_PIPE_DIRECTORY="${CUDA_MPS_PIPE_DIRECTORY}" \
    CUDA_MPS_LOG_DIRECTORY="${CUDA_MPS_LOG_DIRECTORY}" \
    nvidia-cuda-mps-control -d
fi

mkdir -p "${OUTPUT_DIR}/ros_logs"
export ROS_LOG_DIR="${OUTPUT_DIR}/ros_logs"

{
  echo "run_id=${RUN_ID}"
  echo "active_robot_ids=0"
  echo "bag_path=${BAG_PATH}"
  echo "bag_rate=${BAG_RATE}"
  echo "bag_start_delay=${BAG_START_DELAY}"
  echo "run_duration=${RUN_DURATION}"
  echo "rerun_host=${RERUN_HOST}"
  echo "rmw_implementation=${SPLIT_RMW_IMPLEMENTATION}"
  echo "ros_domain_id=${ROS_DOMAIN_ID}"
  echo "fastdds_profile=${FASTDDS_PROFILE}"
  echo "nvpmodel=$(sudo nvpmodel -q | tr '\n' ' ')"
  echo "visualization_mode=minimal"
  echo "started_at=$(date --iso-8601=seconds)"
} >"${OUTPUT_DIR}/run_manifest.txt"

launch_pid=""
sampler_pid=""
tegrastats_pid=""

stop_pid() {
  local signal_name="$1"
  local pid="$2"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill "-${signal_name}" "${pid}" 2>/dev/null || true
  fi
}

cleanup() {
  set +e
  stop_pid TERM "${launch_pid}"
  stop_pid TERM "${sampler_pid}"
  stop_pid TERM "${tegrastats_pid}"
  [[ -n "${launch_pid}" ]] && wait "${launch_pid}" 2>/dev/null
  [[ -n "${sampler_pid}" ]] && wait "${sampler_pid}" 2>/dev/null
  [[ -n "${tegrastats_pid}" ]] && wait "${tegrastats_pid}" 2>/dev/null
}
trap cleanup EXIT INT TERM

python3 "${WORKSPACE_ROOT}/src/sb_slam_ros2/scripts/jetson/sample_ros_processes.py" \
  --output-dir "${OUTPUT_DIR}" &
sampler_pid=$!

tegrastats --interval 1000 >"${OUTPUT_DIR}/tegrastats.log" 2>&1 &
tegrastats_pid=$!

ros2 launch sb_slam_ros2 \
  graco_aerial_05_06_07_08_multi_robot.launch.py \
  active_robot_ids:=0 \
  robot_names_file:="${ROBOT_NAMES_FILE}" \
  aerial_05_bag_path:="${BAG_PATH}" \
  models.xfeat:="${MODEL_DIR}/xfeat_320x224_fp16.engine" \
  models.xfeat_interp_bilinear:="${MODEL_DIR}/interpolator_bilinear_640x480.onnx" \
  models.xfeat_interp_bicubic:="${MODEL_DIR}/interpolator_bicubic_640x480.onnx" \
  models.xfeat_interp_nearest:="${MODEL_DIR}/interpolator_nearest_640x480.onnx" \
  models.lightglue_frontend:="${MODEL_DIR}/lg_320x224_dyn_n1_o500_m1024_fp16.engine" \
  models.lightglue_lcd:="${MODEL_DIR}/lg_320x224_dyn_n1_o500_m1024_fp16.engine" \
  models.jist:="${MODEL_DIR}/JIST_r18_512_seqgem_simplified_fp16.engine" \
  bag_rate:="${BAG_RATE}" \
  bag_start_delay:="${BAG_START_DELAY}" \
  bag_playback_duration:="${RUN_DURATION}" \
  log_output:=true \
  log_output_path:="${OUTPUT_DIR}/output" \
  rerun_recording_id:="${RUN_ID}" \
  rerun_host:="${RERUN_HOST}" \
  visualization_mode:=minimal \
  start_zenoh_router:=false \
  >"${OUTPUT_DIR}/ros_launch.log" 2>&1 &
launch_pid=$!

set +e
wait "${launch_pid}"
launch_status=$?
set -e
launch_pid=""

{
  echo "finished_at=$(date --iso-8601=seconds)"
  echo "launch_status=${launch_status}"
} >>"${OUTPUT_DIR}/run_manifest.txt"

if [[ "${launch_status}" -ne 0 && "${launch_status}" -ne 130 &&
      "${launch_status}" -ne 143 ]]; then
  exit "${launch_status}"
fi
