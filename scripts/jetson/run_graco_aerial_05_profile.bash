#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="${ISAAC_ROS_WS:-/workspaces/isaac_ros-dev}"
MODEL_DIR="${MODEL_DIR:-${WORKSPACE_ROOT}/models/jetpack_7.2_trt_10.16}"
BAG_PATH="${BAG_PATH:-/home/mikexyl/data/graco/aerial-05-40m}"
BAG_RATE="${BAG_RATE:-1.0}"
BAG_START_DELAY="${BAG_START_DELAY:-20.0}"
RUN_DURATION="${RUN_DURATION:-0}"
RERUN_HOST="${RERUN_HOST:-rerun+http://192.168.0.206:9876/proxy}"
VISUALIZATION_MODE="${VISUALIZATION_MODE:-minimal}"
RUN_ID="${RUN_ID:-$(date +%Y%m%d_%H%M%S_%z)}"
OUTPUT_DIR="${OUTPUT_DIR:-${WORKSPACE_ROOT}/benchmark/graco_aerial_05/${RUN_ID}}"
ROBOT_NAMES_FILE="${ROBOT_NAMES_FILE:-${WORKSPACE_ROOT}/install/kimera_distributed/share/kimera_distributed/params/robot_names_graco.yaml}"

set +u
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u

mkdir -p "${OUTPUT_DIR}/ros_logs"
export ISAAC_ROS_WS="${WORKSPACE_ROOT}"
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ROS_LOG_DIR="${OUTPUT_DIR}/ros_logs"
export CUDA_MPS_PIPE_DIRECTORY="${CUDA_MPS_PIPE_DIRECTORY:-/tmp/nvidia-mps}"
export CUDA_MPS_LOG_DIRECTORY="${CUDA_MPS_LOG_DIRECTORY:-/tmp/nvidia-log}"

required_files=(
  "${BAG_PATH}/metadata.yaml"
  "${ROBOT_NAMES_FILE}"
  "${MODEL_DIR}/xfeat_320x224_fp16.engine"
  "${MODEL_DIR}/lg_320x224_dyn_n1_o500_m1024_fp16.engine"
  "${MODEL_DIR}/JIST_r18_512_seqgem_simplified_fp16.engine"
)
for required_file in "${required_files[@]}"; do
  if [[ ! -s "${required_file}" ]]; then
    echo "Required file is missing or empty: ${required_file}" >&2
    exit 1
  fi
done

curl --silent --show-error --max-time 3 \
  "http://192.168.0.206:9876/" >/dev/null

if ! pgrep -f nvidia-cuda-mps-control >/dev/null; then
  mkdir -p "${CUDA_MPS_PIPE_DIRECTORY}" "${CUDA_MPS_LOG_DIRECTORY}"
  sudo env \
    CUDA_MPS_PIPE_DIRECTORY="${CUDA_MPS_PIPE_DIRECTORY}" \
    CUDA_MPS_LOG_DIRECTORY="${CUDA_MPS_LOG_DIRECTORY}" \
    nvidia-cuda-mps-control -d
fi

launch_pid=""
bag_pid=""
duration_pid=""
sampler_pid=""
tegrastats_pid=""

stop_pid() {
  local signal_name="$1"
  local pid="$2"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill "-${signal_name}" "${pid}" 2>/dev/null || true
  fi
}

stop_process_group() {
  local signal_name="$1"
  local leader_pid="$2"
  if [[ -n "${leader_pid}" ]] && kill -0 "${leader_pid}" 2>/dev/null; then
    kill "-${signal_name}" -- "-${leader_pid}" 2>/dev/null || true
  fi
}

cleanup() {
  set +e
  stop_process_group TERM "${bag_pid}"
  stop_pid TERM "${duration_pid}"
  stop_process_group TERM "${launch_pid}"
  stop_pid TERM "${sampler_pid}"
  stop_pid TERM "${tegrastats_pid}"
  [[ -n "${bag_pid}" ]] && wait "${bag_pid}" 2>/dev/null
  [[ -n "${duration_pid}" ]] && wait "${duration_pid}" 2>/dev/null
  [[ -n "${launch_pid}" ]] && wait "${launch_pid}" 2>/dev/null
  [[ -n "${sampler_pid}" ]] && wait "${sampler_pid}" 2>/dev/null
  [[ -n "${tegrastats_pid}" ]] && wait "${tegrastats_pid}" 2>/dev/null
}
trap cleanup EXIT INT TERM

{
  echo "run_id=${RUN_ID}"
  echo "bag_path=${BAG_PATH}"
  echo "bag_rate=${BAG_RATE}"
  echo "bag_start_delay=${BAG_START_DELAY}"
  echo "run_duration=${RUN_DURATION}"
  echo "rerun_host=${RERUN_HOST}"
  echo "visualization_mode=${VISUALIZATION_MODE}"
  echo "started_at=$(date --iso-8601=seconds)"
} >"${OUTPUT_DIR}/run_manifest.txt"

python3 "${WORKSPACE_ROOT}/src/sb_slam_ros2/scripts/jetson/sample_ros_processes.py" \
  --output-dir "${OUTPUT_DIR}" &
sampler_pid=$!

tegrastats --interval 1000 >"${OUTPUT_DIR}/tegrastats.log" 2>&1 &
tegrastats_pid=$!

setsid ros2 launch sb_slam_ros2 graco_robot.launch.py \
  robot_id:=0 \
  robot_name:=a5 \
  num_robots:=1 \
  robot_names_file:="${ROBOT_NAMES_FILE}" \
  models.xfeat:="${MODEL_DIR}/xfeat_320x224_fp16.engine" \
  models.lightglue_frontend:="${MODEL_DIR}/lg_320x224_dyn_n1_o500_m1024_fp16.engine" \
  models.lightglue_lcd:="${MODEL_DIR}/lg_320x224_dyn_n1_o500_m1024_fp16.engine" \
  models.jist:="${MODEL_DIR}/JIST_r18_512_seqgem_simplified_fp16.engine" \
  dense_mapping.enabled:=false \
  keyframe_state.publisher_enabled:=false \
  play_bag:=false \
  use_sim_time:=false \
  image_topic:=/a5/cam0/image_raw \
  imu_topic:=/a5/imu0 \
  log_output:=true \
  log_output_path:="${OUTPUT_DIR}" \
  rerun_application_id:=graco_aerial_05_jetson_profile \
  rerun_recording_id:="${RUN_ID}" \
  rerun_host:="${RERUN_HOST}" \
  visualization_mode:="${VISUALIZATION_MODE}" \
  start_zenoh_router:=true \
  >"${OUTPUT_DIR}/ros_launch.log" 2>&1 &
launch_pid=$!

sleep "${BAG_START_DELAY}"
if ! kill -0 "${launch_pid}" 2>/dev/null; then
  echo "ROS launch exited during TensorRT initialization" >&2
  tail -100 "${OUTPUT_DIR}/ros_launch.log" >&2
  exit 1
fi
if ! pgrep -f nvidia-cuda-mps-server >/dev/null; then
  echo "CUDA clients initialized without creating the MPS server" >&2
  tail -100 "${OUTPUT_DIR}/ros_launch.log" >&2
  exit 1
fi

setsid ros2 bag play "${BAG_PATH}" \
  -r "${BAG_RATE}" \
  --read-ahead-queue-size 1000 \
  --topics /camera_left/image_raw /gnss/imu \
  --remap \
  /camera_left/image_raw:=/a5/cam0/image_raw \
  /gnss/imu:=/a5/imu0 \
  >"${OUTPUT_DIR}/rosbag.log" 2>&1 &
bag_pid=$!

if [[ "${RUN_DURATION}" != "0" ]]; then
  (
    sleep "${RUN_DURATION}"
    stop_process_group TERM "${bag_pid}"
  ) &
  duration_pid=$!
fi

set +e
wait "${bag_pid}"
bag_status=$?
set -e
bag_pid=""
stop_pid TERM "${duration_pid}"
duration_pid=""

# Give the asynchronous backend, JIST, Rerun, CBS, and distributed queues time
# to drain before requesting a clean shutdown that flushes StatisticsVIO.csv.
sleep 5
stop_process_group TERM "${launch_pid}"
set +e
wait "${launch_pid}"
launch_status=$?
set -e
launch_pid=""

{
  echo "finished_at=$(date --iso-8601=seconds)"
  echo "bag_status=${bag_status}"
  echo "launch_status=${launch_status}"
} >>"${OUTPUT_DIR}/run_manifest.txt"

if [[ "${bag_status}" -ne 0 && "${bag_status}" -ne 130 ]]; then
  exit "${bag_status}"
fi
