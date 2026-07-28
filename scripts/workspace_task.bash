#!/usr/bin/env bash
set -euo pipefail

PACKAGE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="${SB_SLAM_ROS2_WS:-$(cd -- "${PACKAGE_ROOT}/../.." && pwd)}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
REPOS_FILE="${PACKAGE_ROOT}/sb_slam_ros2.repos"
ONNXRUNTIME_VERSION="1.22.0"
ONNXRUNTIME_DIST="onnxruntime-linux-x64-gpu-${ONNXRUNTIME_VERSION}"
ONNXRUNTIME_ROOT="${WORKSPACE_ROOT}/src/${ONNXRUNTIME_DIST}"
ONNXRUNTIME_ARCHIVE="${WORKSPACE_ROOT}/src/${ONNXRUNTIME_DIST}.tgz"
ONNXRUNTIME_URL="https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/${ONNXRUNTIME_DIST}.tgz"
RERUN_VERSION="0.35.0"
RERUN_ROOT="${WORKSPACE_ROOT}/.deps/rerun/${RERUN_VERSION}"
RERUN_PREFIX="${RERUN_ROOT}/install"
RERUN_SDK_DIR="${RERUN_PREFIX}/lib/cmake/rerun_sdk"
RERUN_ARCHIVE="${RERUN_ROOT}/rerun_cpp_sdk.zip"
RERUN_URL="https://github.com/rerun-io/rerun/releases/download/${RERUN_VERSION}/rerun_cpp_sdk.zip"
XFEAT_ROOT="${WORKSPACE_ROOT}/src/xfeat-cpp"
LIBSGM_ROOT="${XFEAT_ROOT}/thirdparty/libsgm"
LIBSGM_COMMIT="c9309ec7366db9cd10ea93eea4912c287484c60e"
LIBSGM_URL="https://github.com/mikexyl/libSGM.git"
COMPAT_INCLUDE_DIR="${PACKAGE_ROOT}/compat/include"
BUILD_PARALLEL_JOBS="${BUILD_PARALLEL_JOBS:-12}"
COLCON_PARALLEL_WORKERS="${COLCON_PARALLEL_WORKERS:-${BUILD_PARALLEL_JOBS}}"

source_ros() {
  local setup_file="/opt/ros/${ROS_DISTRO}/setup.bash"
  if [[ ! -f "${setup_file}" ]]; then
    echo "ROS 2 setup file not found: ${setup_file}" >&2
    return 1
  fi

  set +u
  # shellcheck source=/dev/null
  source "${setup_file}"
  set -u
}

source_workspace_if_present() {
  local setup_file="${WORKSPACE_ROOT}/install/setup.bash"
  if [[ -f "${setup_file}" ]]; then
    set +u
    # shellcheck source=/dev/null
    source "${setup_file}"
    set -u
  fi
}

require_command() {
  local command_name="$1"
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "Required command not found: ${command_name}" >&2
    return 1
  fi
}

ensure_onnxruntime() {
  if [[ -f "${ONNXRUNTIME_ROOT}/include/onnxruntime_cxx_api.h" &&
        -f "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so" ]]; then
    return 0
  fi

  require_command wget
  require_command tar
  mkdir -p "${WORKSPACE_ROOT}/src"
  echo "Downloading ONNX Runtime ${ONNXRUNTIME_VERSION} from ${ONNXRUNTIME_URL}"
  wget -nv -O "${ONNXRUNTIME_ARCHIVE}" "${ONNXRUNTIME_URL}"
  tar -xzf "${ONNXRUNTIME_ARCHIVE}" -C "${WORKSPACE_ROOT}/src"
}

ensure_rerun_sdk() {
  if [[ -f "${RERUN_SDK_DIR}/rerun_sdkConfig.cmake" ]]; then
    return 0
  fi

  require_command wget
  require_command unzip
  require_command cmake
  mkdir -p "${RERUN_ROOT}"
  if [[ ! -f "${RERUN_ARCHIVE}" ]]; then
    echo "Downloading Rerun C++ SDK ${RERUN_VERSION} from ${RERUN_URL}"
    wget -nv -O "${RERUN_ARCHIVE}" "${RERUN_URL}"
  fi
  unzip -oq "${RERUN_ARCHIVE}" -d "${RERUN_ROOT}"
  cmake -S "${RERUN_ROOT}/rerun_cpp_sdk" \
    -B "${RERUN_ROOT}/build" \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DCMAKE_INSTALL_PREFIX="${RERUN_PREFIX}"
  cmake --build "${RERUN_ROOT}/build" --parallel "${BUILD_PARALLEL_JOBS}"
  cmake --install "${RERUN_ROOT}/build"
}

ensure_xfeat_thirdparty() {
  if [[ ! -d "${XFEAT_ROOT}/.git" ]]; then
    return 0
  fi

  require_command git
  git -C "${XFEAT_ROOT}" \
    -c url.https://github.com/.insteadOf=git@github.com: \
    submodule update --init --recursive

  if [[ ! -d "${LIBSGM_ROOT}/.git" ]]; then
    git clone "${LIBSGM_URL}" "${LIBSGM_ROOT}"
  fi

  if ! git -C "${LIBSGM_ROOT}" cat-file -e "${LIBSGM_COMMIT}^{commit}" 2>/dev/null; then
    if ! git -C "${LIBSGM_ROOT}" remote get-url mikexyl >/dev/null 2>&1; then
      git -C "${LIBSGM_ROOT}" remote add mikexyl "${LIBSGM_URL}"
    fi
    git -C "${LIBSGM_ROOT}" fetch mikexyl "${LIBSGM_COMMIT}"
  fi

  git -C "${LIBSGM_ROOT}" checkout "${LIBSGM_COMMIT}"
}

export_compat_includes() {
  export CPLUS_INCLUDE_PATH="${COMPAT_INCLUDE_DIR}:${CPLUS_INCLUDE_PATH:-}"
}

configure_build_parallelism() {
  export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-${BUILD_PARALLEL_JOBS}}"
  export MAKEFLAGS="${MAKEFLAGS:--j${BUILD_PARALLEL_JOBS}}"
}

ported_package_paths() {
  colcon list \
    --base-paths "${WORKSPACE_ROOT}/src" \
    --packages-up-to sb_slam_ros2 \
    --paths-only
}

common_cmake_args=(
  -DBUILD_TESTING=OFF
  -DKIMERA_BUILD_EXAMPLES=OFF
  -DKIMERA_BUILD_TESTS=OFF
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF
  -DGTSAM_BUILD_PYTHON=OFF
  -DGTSAM_BUILD_TESTS=OFF
  -DGTSAM_BUILD_UNSTABLE=ON
  -DGTSAM_POSE3_EXPMAP=ON
  -DGTSAM_ROT3_EXPMAP=ON
  -DGTSAM_TANGENT_PREINTEGRATION=OFF
  -DGTSAM_USE_QUATERNIONS=OFF
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
  -DGTSAM_USE_SYSTEM_EIGEN=ON
  "-UGTSAM_COMPILE_OPTIONS_PRIVATE*"
  -DPython3_EXECUTABLE=/usr/bin/python3
  -DPYTHON_EXECUTABLE=/usr/bin/python3
  -Donnxruntime_DIR="${ONNXRUNTIME_ROOT}"
  -Drerun_sdk_DIR="${RERUN_SDK_DIR}"
  -DCUDAToolkit_ROOT="${CUDA_TOOLKIT_ROOT}"
  -DCUDA_TOOLKIT_ROOT_DIR="${CUDA_TOOLKIT_ROOT}"
  -DCUDA_NVCC_EXECUTABLE="${CUDA_TOOLKIT_ROOT}/bin/nvcc"
  -DCMAKE_CUDA_ARCHITECTURES="${CUDA_ARCHITECTURES}"
  -DVILIB_CUDA_ARCHITECTURES="${VILIB_CUDA_ARCHITECTURES}"
  -DCUBLAS_DIR="${CUDA_TOOLKIT_ROOT}"
  -DXFEAT_BUILD_EXAMPLES=OFF
  -DXFEAT_BUILD_SCRIPTS=OFF
  -DVIO_FACTORS_BUILD_EXAMPLES=OFF
  -DCBS_BUILD_UTILS=OFF
  -DCBS_BUILD_EXAMPLES=OFF
)

run_build() {
  local package_name="$1"
  source_ros
  require_command colcon
  ensure_onnxruntime
  ensure_rerun_sdk
  ensure_xfeat_thirdparty
  export_compat_includes
  configure_build_parallelism
  cd "${WORKSPACE_ROOT}"
  colcon build --symlink-install --parallel-workers "${COLCON_PARALLEL_WORKERS}" --packages-up-to "${package_name}" --cmake-args "${common_cmake_args[@]}"
}

case "${1:-}" in
  import)
    require_command vcs
    mkdir -p "${WORKSPACE_ROOT}/src"
    vcs import --skip-existing "${WORKSPACE_ROOT}/src" < "${REPOS_FILE}"
    ;;

  rosdep)
    source_ros
    require_command rosdep
    require_command colcon
    if ! rosdep db >/dev/null 2>&1; then
      rosdep update
    fi
    cd "${WORKSPACE_ROOT}"
    mapfile -t package_paths < <(ported_package_paths)
    rosdep install --from-paths "${package_paths[@]}" --ignore-src -r -y --rosdistro "${ROS_DISTRO}" \
      --skip-keys "gtsam_unstable dbow2 xfeat-cpp vilib catkin roscpp message_runtime"
    ;;

  build)
    run_build sb_slam_ros2
    ;;

  build-kimera-vio)
    run_build kimera_vio
    ;;

  build-kimera-vio-ros)
    run_build kimera_vio_ros
    ;;

  test)
    source_ros
    require_command colcon
    ensure_onnxruntime
    ensure_rerun_sdk
    ensure_xfeat_thirdparty
    export_compat_includes
    configure_build_parallelism
    cd "${WORKSPACE_ROOT}"
    colcon test --event-handlers console_direct+ --parallel-workers "${COLCON_PARALLEL_WORKERS}" --packages-up-to sb_slam_ros2
    colcon test-result --verbose
    ;;

  shell-env)
    cat <<EOF
export SB_SLAM_ROS2_WS="${WORKSPACE_ROOT}"
export ROS_DISTRO="${ROS_DISTRO}"
export ONNXRUNTIME_ROOT="${ONNXRUNTIME_ROOT}"
export RERUN_SDK_DIR="${RERUN_SDK_DIR}"
export CUDA_TOOLKIT_ROOT="${CUDA_TOOLKIT_ROOT}"
export CUDA_HOME="${CUDA_TOOLKIT_ROOT}"
export CUDA_PATH="${CUDA_TOOLKIT_ROOT}"
export CUDACXX="${CUDA_TOOLKIT_ROOT}/bin/nvcc"
export CUDA_ARCHITECTURES="${CUDA_ARCHITECTURES}"
export VILIB_CUDA_ARCHITECTURES="${VILIB_CUDA_ARCHITECTURES}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export BUILD_PARALLEL_JOBS="${BUILD_PARALLEL_JOBS}"
export COLCON_PARALLEL_WORKERS="${COLCON_PARALLEL_WORKERS}"
export CMAKE_BUILD_PARALLEL_LEVEL="${BUILD_PARALLEL_JOBS}"
export CPLUS_INCLUDE_PATH="${COMPAT_INCLUDE_DIR}:\${CPLUS_INCLUDE_PATH:-}"
export PATH="${CUDA_TOOLKIT_ROOT}/bin:\${PATH}"
export LD_LIBRARY_PATH="${CUDA_TOOLKIT_ROOT}/lib64:${CUDA_TOOLKIT_ROOT}/targets/x86_64-linux/lib:${ONNXRUNTIME_ROOT}/lib:\${LD_LIBRARY_PATH:-}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "${WORKSPACE_ROOT}/install/setup.bash" ]; then
  source "${WORKSPACE_ROOT}/install/setup.bash"
fi
EOF
    ;;

  *)
    echo "Usage: $0 {import|rosdep|build|build-kimera-vio|build-kimera-vio-ros|test|shell-env}" >&2
    exit 2
    ;;
esac
