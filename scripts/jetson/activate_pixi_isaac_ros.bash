#!/usr/bin/env bash

# Pixi isolates Python/build tooling. JetPack and ROS remain system-provided so
# CUDA/TensorRT stay aligned with the Orin's BSP.
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy is not installed yet; run the isaac-install-host task" >&2
else
  source /opt/ros/jazzy/setup.bash
fi

export ISAAC_ROS_WS="${ISAAC_ROS_WS:-${PIXI_PROJECT_ROOT}}"
export MODEL_DIR="${MODEL_DIR:-${ISAAC_ROS_WS}/models/jetpack_7.2_trt_10.16}"
export FAISS_ROOT="${FAISS_ROOT:-${ISAAC_ROS_WS}/.deps/faiss}"
export ONNXRUNTIME_ROOT="${ONNXRUNTIME_ROOT:-${ISAAC_ROS_WS}/.deps/onnxruntime}"
export RERUN_ROOT="${RERUN_ROOT:-${ISAAC_ROS_WS}/.deps/rerun}"
export PATH="/usr/local/cuda-13.2/bin:/usr/src/tensorrt/bin:${PATH}"
export LD_LIBRARY_PATH="/usr/local/cuda-13.2/lib64:/usr/local/cuda-13.2/targets/aarch64-linux/lib:${LD_LIBRARY_PATH:-}"

if [[ -f "${ISAAC_ROS_WS}/install/setup.bash" ]]; then
  source "${ISAAC_ROS_WS}/install/setup.bash"
fi
