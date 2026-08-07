#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_ROOT="${SB_SLAM_ROS2_WS:-$(cd -- "${PACKAGE_ROOT}/../.." && pwd)}"
SERVER_148_TARGET="${SERVER_148_TARGET:-mikexyl@192.168.0.148}"
SERVER_148_WORKSPACE="${SERVER_148_WORKSPACE:-/home/mikexyl/workspaces/sb_slam_ros2}"

rsync_args=(
  -az
  --delete
  --mkpath
  --safe-links
  --human-readable
  --itemize-changes
  --exclude=.git/
  --exclude=.pixi/
  --exclude=build/
  --exclude=install/
  --exclude=log/
  --exclude=__pycache__/
  --exclude=onnx_model/
)

if [[ "${1:-}" == "--dry-run" ]]; then
  rsync_args+=(--dry-run)
  shift
fi

if (($# > 0)); then
  packages=("$@")
else
  packages=(xfeat-cpp Kimera-VIO Kimera-VIO-ROS2 sb_slam_ros2)
fi

echo "Syncing pixi.toml to ${SERVER_148_TARGET}"
rsync "${rsync_args[@]}" \
  "${WORKSPACE_ROOT}/pixi.toml" \
  "${SERVER_148_TARGET}:${SERVER_148_WORKSPACE}/pixi.toml"

echo "Syncing server-148 Pixi toolchain to ${SERVER_148_TARGET}"
rsync "${rsync_args[@]}" \
  "${WORKSPACE_ROOT}/tools/server-148/" \
  "${SERVER_148_TARGET}:${SERVER_148_WORKSPACE}/tools/server-148/"

for package in "${packages[@]}"; do
  source_path="${WORKSPACE_ROOT}/src/${package}"
  if [[ ! -d "${source_path}" ]]; then
    echo "Package source does not exist: ${source_path}" >&2
    exit 1
  fi

  echo "Syncing src/${package} to ${SERVER_148_TARGET}"
  rsync "${rsync_args[@]}" \
    "${source_path}/" \
    "${SERVER_148_TARGET}:${SERVER_148_WORKSPACE}/src/${package}/"
done
