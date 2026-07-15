# SB-SLAM-ROS2

This package is the ROS 2 workspace anchor for SB-SLAM-ROS2. It intentionally
contains no nodes or libraries yet.

The Docker image provides the ROS 2 Humble and CUDA environment. The workspace
root Pixi project is intentionally dependency-free and is used only as a task
runner for workspace operations.

## Container

Build the development image from the workspace root:

```bash
docker build -t sb-slam-ros2:metapackage -f src/sb_slam_ros2/docker/Dockerfile src/sb_slam_ros2
```

Create or enter the persistent development container:

```bash
src/sb_slam_ros2/docker/start_container.sh
```

Run a command in the persistent container:

```bash
src/sb_slam_ros2/docker/start_container.sh pixi run build
```

Available tasks:

```bash
cd ../..
pixi run import
pixi run rosdep
pixi run build
pixi run build-kimera-vio
pixi run test
pixi run shell-env
```

The helper uses the container name `sb_slam_ros2` by default. Stop it with
`docker stop sb_slam_ros2` when you are done, then start or enter it again with
the same helper.

Ported package sources are tracked in `sb_slam_ros2.repos`. See
`docs/PORTING.md` for the ROS 2 branches checked before importing packages.
