# SB-SLAM-ROS2

This package is the ROS 2 workspace anchor for SB-SLAM-ROS2. It intentionally
contains no nodes or libraries yet.

The desktop Docker image provides the ROS 2 Humble and CUDA environment. The
workspace Pixi project also defines an isolated `isaac-ros-jetson` feature for
JetPack 7.2. That environment locks Python/build tooling from conda-forge and
PyPI while using the board's native ROS, CUDA, TensorRT, and OpenCV libraries.

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
pixi run build-teaserpp
pixi run build-kimera-vio
pixi run test
pixi run shell-env
```

The helper uses the container name `sb_slam_ros2` by default. Stop it with
`docker stop sb_slam_ros2` when you are done, then start or enter it again with
the same helper.

Ported package sources are tracked in `sb_slam_ros2.repos`. See
`docs/PORTING.md` for the ROS 2 branches checked before importing packages.

## Single-robot loop closure

`graco_single_robot_loop_closure.launch.py` runs one namespaced robot through
the complete multi-robot stack: Kimera-VIO with the descriptor bridge enabled,
Kimera-Distributed configured for one robot and intra-robot loop detection, and
CBS online pose-graph optimization. It defaults to the aerial-05 ROS 2 bag.

```bash
MODEL_DIR="$PWD/src/xfeat-cpp/onnx_model"
ros2 launch sb_slam_ros2 graco_single_robot_loop_closure.launch.py \
  models.xfeat:="$MODEL_DIR/xfeat_320x224.onnx" \
  models.xfeat_interp_bilinear:="$MODEL_DIR/interpolator_bilinear_320x224.onnx" \
  models.xfeat_interp_bicubic:="$MODEL_DIR/interpolator_bicubic_320x224.onnx" \
  models.xfeat_interp_nearest:="$MODEL_DIR/interpolator_nearest_320x224.onnx" \
  models.lightglue_frontend:="$MODEL_DIR/lg_320x224_dyn.onnx" \
  models.lightglue_lcd:="$MODEL_DIR/lg_320x224_dyn.onnx" \
  models.jist:="$MODEL_DIR/JIST_r18_512_seqgem_simplified.onnx"
```

The launch fails immediately when a required model, robot-name configuration,
or ROS 2 bag is missing. Its Rerun recording name includes the launch system's
local timestamp.

## GrAco ground g1/g2/g3

`graco_ground_01_02_03_multi_robot.launch.py` mirrors the ROS 1 GrAco g123
experiment with three stereo VIO pipelines, Distributed, and Sim3 CBS. It
defaults to `/data/graco/ground-01`, `ground-02`, and `ground-03_ros2` and
remaps each bag to the `g1`, `g2`, and `g3` namespaces.
The Kimera frontend and loop verification use the native TensorRT XFeat and
LighterGlue engines from `xfeat-cpp/onnx_model`. Dense mapping, DA3, and
dense-mapping keyframe publication are hard-disabled.

```bash
RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 launch sb_slam_ros2 \
  graco_ground_01_02_03_multi_robot.launch.py \
  start_zenoh_router:=true
```

Leave `start_zenoh_router:=false` when a Zenoh router is already running.

Override any missing converted bag directory with, for example,
`ground_01_bag_path:=/path/to/ground-01`.

## Jetson Orin NX (JetPack 7.2)

Deploy the workspace, then use the ARM64-only Pixi environment on the board:

```bash
src/sb_slam_ros2/scripts/jetson/deploy_workspace.bash
ssh mikexyl@192.168.0.217
cd ~/workspaces/isaac_ros-dev
pixi run -e isaac-ros-jetson isaac-install-host
pixi run -e isaac-ros-jetson isaac-bootstrap-sdks
pixi run -e isaac-ros-jetson isaac-check
pixi run -e isaac-ros-jetson isaac-build-engines
pixi run -e isaac-ros-jetson isaac-build
```

`isaac-install-host` refuses to run unless the JetPack 7.2 CUDA 13.2 compiler
and TensorRT 10.16 runtime are present. FAISS, ONNX Runtime, and Rerun are
bootstrapped as native C++ SDKs; Pixi's PyPI dependencies remain isolated build
tools. `cv_bridge` is built from the pinned `vision_opencv` source so installing
ROS does not downgrade NVIDIA OpenCV 4.8.
The full build always includes dense mapping; launch files decide whether its
nodes and publishers are enabled. The Jetson benchmark launch explicitly
disables the complete dense-mapping path.
