#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="${ISAAC_ROS_WS:-${PIXI_PROJECT_ROOT}}"
FAISS_VERSION="${FAISS_VERSION:-1.14.3}"
ONNXRUNTIME_VERSION="${ONNXRUNTIME_VERSION:-1.22.0}"
RERUN_VERSION="${RERUN_VERSION:-0.35.0}"
FAISS_ROOT="${FAISS_ROOT:-${WORKSPACE_ROOT}/.deps/faiss}"
ONNXRUNTIME_ROOT="${ONNXRUNTIME_ROOT:-${WORKSPACE_ROOT}/.deps/onnxruntime}"
RERUN_ROOT="${RERUN_ROOT:-${WORKSPACE_ROOT}/.deps/rerun}"
BUILD_JOBS="${BUILD_JOBS:-4}"

bootstrap_tmp="$(mktemp -d /tmp/sb-slam-sdk-bootstrap.XXXXXX)"
trap 'rm -rf -- "${bootstrap_tmp}"' EXIT

if [[ ! -f "${FAISS_ROOT}/share/faiss/faiss-config.cmake" ]]; then
  archive="${bootstrap_tmp}/faiss.tar.gz"
  curl -fL \
    "https://github.com/facebookresearch/faiss/archive/refs/tags/v${FAISS_VERSION}.tar.gz" \
    -o "${archive}"
  mkdir -p "${bootstrap_tmp}/faiss-source"
  tar -xzf "${archive}" -C "${bootstrap_tmp}/faiss-source" --strip-components=1
  cmake \
    -S "${bootstrap_tmp}/faiss-source" \
    -B "${bootstrap_tmp}/faiss-build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${FAISS_ROOT}" \
    -DBUILD_TESTING=OFF \
    -DFAISS_ENABLE_C_API=OFF \
    -DFAISS_ENABLE_GPU=OFF \
    -DFAISS_ENABLE_PYTHON=OFF \
    -DFAISS_OPT_LEVEL=generic
  cmake --build "${bootstrap_tmp}/faiss-build" --parallel "${BUILD_JOBS}"
  cmake --install "${bootstrap_tmp}/faiss-build"
fi

if [[ ! -f "${ONNXRUNTIME_ROOT}/include/onnxruntime_cxx_api.h" ]]; then
  archive="${bootstrap_tmp}/onnxruntime.tgz"
  curl -fL \
    "https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-linux-aarch64-${ONNXRUNTIME_VERSION}.tgz" \
    -o "${archive}"
  mkdir -p "${bootstrap_tmp}/onnxruntime" "${ONNXRUNTIME_ROOT}"
  tar -xzf "${archive}" -C "${bootstrap_tmp}/onnxruntime" --strip-components=1
  cp -a "${bootstrap_tmp}/onnxruntime/." "${ONNXRUNTIME_ROOT}/"
fi

if [[ ! -f "${RERUN_ROOT}/lib/cmake/rerun_sdk/rerun_sdkConfig.cmake" ]]; then
  archive="${bootstrap_tmp}/rerun_cpp_sdk.zip"
  curl -fL \
    "https://github.com/rerun-io/rerun/releases/download/${RERUN_VERSION}/rerun_cpp_sdk.zip" \
    -o "${archive}"
  unzip -q "${archive}" -d "${bootstrap_tmp}/rerun"
  cmake \
    -S "${bootstrap_tmp}/rerun/rerun_cpp_sdk" \
    -B "${bootstrap_tmp}/rerun-build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DCMAKE_INSTALL_PREFIX="${RERUN_ROOT}"
  cmake --build "${bootstrap_tmp}/rerun-build" --parallel "${BUILD_JOBS}"
  cmake --install "${bootstrap_tmp}/rerun-build"
fi

test -f "${FAISS_ROOT}/share/faiss/faiss-config.cmake"
test -f "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so"
test -f "${RERUN_ROOT}/lib/cmake/rerun_sdk/rerun_sdkConfig.cmake"
printf 'Native SDKs ready: FAISS %s, ONNX Runtime %s, Rerun %s\n' \
  "${FAISS_VERSION}" "${ONNXRUNTIME_VERSION}" "${RERUN_VERSION}"
