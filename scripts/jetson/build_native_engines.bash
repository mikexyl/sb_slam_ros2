#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="${ISAAC_ROS_WS:-$(cd -- "${SCRIPT_DIR}/../../../.." && pwd)}"

export SOURCE_DIR="${SOURCE_DIR:-/home/mikexyl/xfeat-trt-benchmark/artifacts/xfeat-lightglue}"
export OUTPUT_DIR="${OUTPUT_DIR:-${WORKSPACE_ROOT}/models/jetpack_7.2_trt_10.16}"
export EXPECTED_TRT_VERSION_PREFIX="10.16."

exec "${SCRIPT_DIR}/build_isaac_engines.bash"
