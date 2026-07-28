#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="${ISAAC_ROS_WS:-/workspaces/isaac_ros-dev}"
# Kimera's template-heavy ROS translation units exceed Orin NX memory when
# several cc1plus processes run concurrently. Keep the Jetson default reliable;
# larger-memory targets may opt in to more concurrency through the environment.
BUILD_JOBS="${BUILD_JOBS:-1}"
PARALLEL_WORKERS="${PARALLEL_WORKERS:-1}"
CUDA_ROOT="${CUDA_ROOT:-${CUDA_HOME:-/usr/local/cuda-13.0}}"
FAISS_ROOT="${FAISS_ROOT:-${WORKSPACE_ROOT}/.deps/faiss}"
ONNXRUNTIME_ROOT="${ONNXRUNTIME_ROOT:-/opt/onnxruntime}"
RERUN_ROOT="${RERUN_ROOT:-/opt/rerun}"

set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "${WORKSPACE_ROOT}"

export CMAKE_BUILD_PARALLEL_LEVEL="${BUILD_JOBS}"
export CUDA_HOME="${CUDA_ROOT}"
export CUDA_PATH="${CUDA_ROOT}"
export CUDACXX="${CUDA_ROOT}/bin/nvcc"

colcon build \
  --symlink-install \
  --parallel-workers "${PARALLEL_WORKERS}" \
  --packages-up-to sb_slam_ros2 \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DBUILD_TESTING=OFF \
    -DKIMERA_BUILD_EXAMPLES=OFF \
    -DKIMERA_BUILD_TESTS=OFF \
    -DKIMERA_BUILD_OPENCV_VIZ=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_PYTHON=OFF \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_UNSTABLE=ON \
    -DGTSAM_POSE3_EXPMAP=ON \
    -DGTSAM_ROT3_EXPMAP=ON \
    -DGTSAM_TANGENT_PREINTEGRATION=OFF \
    -DGTSAM_USE_QUATERNIONS=OFF \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=ON \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -Dfaiss_DIR="${FAISS_ROOT}/share/faiss" \
    -Donnxruntime_DIR="${ONNXRUNTIME_ROOT}" \
    -Drerun_sdk_DIR="${RERUN_ROOT}/lib/cmake/rerun_sdk" \
    -DDENSE_MAPPING_RERUN_SDK_DIR="${RERUN_ROOT}/lib/cmake/rerun_sdk" \
    -DCUDAToolkit_ROOT="${CUDA_ROOT}" \
    -DCUDA_TOOLKIT_ROOT_DIR="${CUDA_ROOT}" \
    -DCUDA_NVCC_EXECUTABLE="${CUDA_ROOT}/bin/nvcc" \
    -DCMAKE_CUDA_ARCHITECTURES=87 \
    -DVILIB_CUDA_ARCHITECTURES=8.7 \
    -DCUBLAS_DIR="${CUDA_ROOT}" \
    -DXFEAT_BUILD_EXAMPLES=OFF \
    -DXFEAT_BUILD_SCRIPTS=OFF \
    -DVIO_FACTORS_BUILD_EXAMPLES=OFF \
    -DCBS_BUILD_UTILS=OFF \
    -DCBS_BUILD_EXAMPLES=OFF \
    -DBUILD_DEMO=OFF \
    -DBUILD_EXAMPLE=OFF \
    -DBUILD_TOOLS=OFF
