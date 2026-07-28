#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
WORKSPACE_ROOT="$(cd -- "${PACKAGE_ROOT}/../.." && pwd)"
JETSON_TARGET="${JETSON_TARGET:-mikexyl@192.168.0.217}"
JETSON_WORKSPACE="${JETSON_WORKSPACE:-/home/mikexyl/workspaces/isaac_ros-dev}"

command -v colcon >/dev/null
command -v rsync >/dev/null
command -v ssh >/dev/null

ssh "${JETSON_TARGET}" \
  "mkdir -p '${JETSON_WORKSPACE}/src' '${JETSON_WORKSPACE}/docker' '${JETSON_WORKSPACE}/scripts' '${JETSON_WORKSPACE}/.isaac-ros-cli'"

# The workspace manifest is intentionally deployed without the host lock file.
# Its Jetson environment contains PyPI dependencies and must be solved natively
# for linux-aarch64; Pixi then writes a reproducible lock on the board.
rsync -az \
  "${WORKSPACE_ROOT}/pixi.toml" \
  "${JETSON_TARGET}:${JETSON_WORKSPACE}/pixi.toml"

mapfile -t package_paths < <(
  colcon list \
    --base-paths "${WORKSPACE_ROOT}/src" \
    --packages-up-to sb_slam_ros2 \
    --paths-only
)

declare -A source_roots=()
for package_path in "${package_paths[@]}"; do
  absolute_path="$(realpath "${package_path}")"
  relative_path="${absolute_path#${WORKSPACE_ROOT}/src/}"
  source_root="${relative_path%%/*}"
  source_roots["${source_root}"]=1
done

for source_root in "${!source_roots[@]}"; do
  echo "Deploying src/${source_root}"
  rsync -az \
    --exclude=.git/ \
    --exclude=.pixi/ \
    --exclude=build/ \
    --exclude=install/ \
    --exclude=log/ \
    --exclude=__pycache__/ \
    --exclude=onnx_model/ \
    "${WORKSPACE_ROOT}/src/${source_root}/" \
    "${JETSON_TARGET}:${JETSON_WORKSPACE}/src/${source_root}/"
done

rsync -az \
  "${PACKAGE_ROOT}/docker/isaac_ros/Dockerfile.sb_slam" \
  "${JETSON_TARGET}:${JETSON_WORKSPACE}/docker/Dockerfile.sb_slam"
rsync -az \
  "${PACKAGE_ROOT}/docker/isaac_ros/config.yaml" \
  "${JETSON_TARGET}:${JETSON_WORKSPACE}/.isaac-ros-cli/config.yaml"
rsync -az \
  "${PACKAGE_ROOT}/docker/isaac_ros/.isaac_ros_common-config" \
  "${JETSON_TARGET}:${JETSON_WORKSPACE}/scripts/.isaac_ros_common-config"
rsync -az \
  "${PACKAGE_ROOT}/docker/isaac_ros/.isaac_ros_dev-dockerargs" \
  "${JETSON_TARGET}:${JETSON_WORKSPACE}/scripts/.isaac_ros_dev-dockerargs"

echo "Deployment complete: ${JETSON_TARGET}:${JETSON_WORKSPACE}"
