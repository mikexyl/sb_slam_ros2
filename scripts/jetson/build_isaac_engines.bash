#!/usr/bin/env bash
set -euo pipefail

SOURCE_DIR="${SOURCE_DIR:-/models/xfeat-lightglue-source}"
OUTPUT_DIR="${OUTPUT_DIR:-/workspaces/isaac_ros-dev/models/isaac_ros_trt_10.13}"
TRTEXEC="${TRTEXEC:-$(command -v trtexec)}"
EXPECTED_TRT_VERSION_PREFIX="${EXPECTED_TRT_VERSION_PREFIX:-10.13.}"

if [[ ! -x "${TRTEXEC}" ]]; then
  echo "trtexec is unavailable: ${TRTEXEC}" >&2
  exit 1
fi

trt_version="$(dpkg-query -W -f='${Version}' libnvinfer10)"
if [[ "${trt_version}" != "${EXPECTED_TRT_VERSION_PREFIX}"* ]]; then
  echo "Expected TensorRT ${EXPECTED_TRT_VERSION_PREFIX}x, found ${trt_version}" >&2
  exit 1
fi
trt_series="${EXPECTED_TRT_VERSION_PREFIX%.}"
TIMING_CACHE="${TIMING_CACHE:-${OUTPUT_DIR}/orin-nx-trt-${trt_series}.timing.cache}"

required_models=(
  xfeat_320x224.onnx
  lg_320x224_dyn.onnx
  JIST_r18_512_seqgem_simplified.onnx
  mixvpr_resnet50_4096d.onnx
)
for model in "${required_models[@]}"; do
  if [[ ! -f "${SOURCE_DIR}/${model}" ]]; then
    echo "Missing ONNX model: ${SOURCE_DIR}/${model}" >&2
    exit 1
  fi
done

mkdir -p "${OUTPUT_DIR}"

common_args=(
  "--timingCacheFile=${TIMING_CACHE}"
  --fp16
  --builderOptimizationLevel=5
  --avgTiming=8
  --memPoolSize=workspace:4096M
  --skipInference
)

build_engine() {
  local model="$1"
  local engine="$2"
  shift 2
  local log="${engine%.engine}.build.log"
  if [[ -s "${engine}" ]]; then
    echo "Engine already exists, preserving it: ${engine}"
    return 0
  fi
  "${TRTEXEC}" \
    "--onnx=${SOURCE_DIR}/${model}" \
    "--saveEngine=${engine}" \
    "${common_args[@]}" \
    "$@" 2>&1 | tee "${log}"
}

build_engine \
  xfeat_320x224.onnx \
  "${OUTPUT_DIR}/xfeat_320x224_fp16.engine"

build_engine \
  lg_320x224_dyn.onnx \
  "${OUTPUT_DIR}/lg_320x224_dyn_n1_o500_m1024_fp16.engine" \
  --minShapes=mkpts0:1x1x2,feats0:1x1x64,mkpts1:1x1x2,feats1:1x1x64 \
  --optShapes=mkpts0:1x500x2,feats0:1x500x64,mkpts1:1x500x2,feats1:1x500x64 \
  --maxShapes=mkpts0:1x1024x2,feats0:1x1024x64,mkpts1:1x1024x2,feats1:1x1024x64

build_engine \
  JIST_r18_512_seqgem_simplified.onnx \
  "${OUTPUT_DIR}/JIST_r18_512_seqgem_simplified_fp16.engine"

build_engine \
  mixvpr_resnet50_4096d.onnx \
  "${OUTPUT_DIR}/mixvpr_resnet50_4096d_fp16.engine"

validate_engine() {
  local engine="$1"
  shift
  local log="${engine%.engine}.benchmark.log"
  "${TRTEXEC}" \
    "--loadEngine=${engine}" \
    --warmUp=1000 \
    --duration=3 \
    --useSpinWait \
    "$@" 2>&1 | tee "${log}"
}

# Deserializing and executing each plan here catches an incompatible or
# truncated artifact before it is consumed by Kimera. These logs also provide
# the per-network TensorRT baseline for the Jetson benchmark.
validate_engine "${OUTPUT_DIR}/xfeat_320x224_fp16.engine"
validate_engine \
  "${OUTPUT_DIR}/lg_320x224_dyn_n1_o500_m1024_fp16.engine" \
  --shapes=mkpts0:1x500x2,feats0:1x500x64,mkpts1:1x500x2,feats1:1x500x64
validate_engine "${OUTPUT_DIR}/JIST_r18_512_seqgem_simplified_fp16.engine"
validate_engine "${OUTPUT_DIR}/mixvpr_resnet50_4096d_fp16.engine"

sha256sum "${OUTPUT_DIR}"/*.engine | tee "${OUTPUT_DIR}/SHA256SUMS"
printf 'TensorRT %s\n' "${trt_version}" | tee "${OUTPUT_DIR}/RUNTIME_VERSION"
