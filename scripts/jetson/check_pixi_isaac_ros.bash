#!/usr/bin/env bash
set -euo pipefail

test "$(uname -m)" = "aarch64"
test -f /opt/ros/jazzy/setup.bash
test -x /usr/local/cuda-13.2/bin/nvcc
test -x "$(command -v trtexec)"

cuda_version="$(/usr/local/cuda-13.2/bin/nvcc --version)"
trt_version="$(dpkg-query -W -f='${Version}' libnvinfer10)"

if [[ "${cuda_version}" != *"release 13.2"* ]]; then
  echo "Expected JetPack CUDA 13.2" >&2
  exit 1
fi
if [[ "${trt_version}" != 10.16.* ]]; then
  echo "Expected JetPack TensorRT 10.16, found ${trt_version}" >&2
  exit 1
fi

python - <<'PY'
import colcon_core
import psutil
import pytest
import yaml

print("Pixi Python dependencies are importable")
PY

printf 'CUDA 13.2\nTensorRT %s\nROS Jazzy\n' "${trt_version}"
