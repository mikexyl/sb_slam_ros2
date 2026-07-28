# Porting Notes

Before porting a package manually, check whether an upstream or local ROS 2
branch already exists and prefer importing that branch.

## Imported ROS 2 Branches

| Repository | Branch | Packages currently used |
| --- | --- | --- |
| `https://github.com/mikexyl/Kimera-VIO-ROS2.git` | `package/kimera_vio_ros` | `kimera_common`, `kimera_graph_msgs`, `kimera_graph_visualizer`, `kimera_vio_ros`, `mesh_splat`, `projective_mesher_msgs` |
| `https://github.com/mikexyl/Kimera-VIO.git` | `dev/code-slam` | `kimera_vio` |
| `https://github.com/MIT-SPARK/Kimera-RPGO.git` | `master` | `kimera_rpgo` |
| `https://github.com/ruffsl/DBoW2.git` | `patch-1` | `dbow2` |
| `https://github.com/mikexyl/xfeat-cpp.git` | `develop` | `xfeat-cpp` |
| `https://github.com/mikexyl/vilib.git` | `master` | `vilib` |
| `https://github.com/mikexyl/aria_common.git` | `kimera-ros2` | `aria_common` |
| `https://github.com/mikexyl/aria_visualization.git` | `kimera-ros2` | `aria_viz` |
| `https://github.com/mikexyl/gtsam.git` | `feature/dynamic_noise_smart_factor` | `gtsam`, `gtsam_unstable` CMake exports |
| `https://github.com/ruffsl/opengv.git` | `patch-1` | `opengv` |
| `https://github.com/MIT-SPARK/pose_graph_tools.git` | `ros2` | `pose_graph_tools`, `pose_graph_tools_msgs`, `pose_graph_tools_ros` |
| `https://github.com/mikexyl/Kimera-Multi-LCD.git` | `develop` | `kimera_multi_lcd` |
| `https://github.com/mikexyl/Kimera-Distributed.git` | `main` | `kimera_distributed` |
| `https://github.com/mikexyl/aria_dopt.git` | `dev/code-slam` | `cbs` |
| `https://github.com/mikexyl/cbs_ros.git` | `main` | `cbs_ros` |
| `https://github.com/mikexyl/vio-factors.git` | `main` | `vio-factors` |

Remote branch checks on 2026-05-10 showed:

```text
MIT-SPARK/pose_graph_tools: main, master, ros2
MIT-SPARK/Kimera-VIO-ROS2: foxy, master, package/kimera_vio_ros
mikexyl/Kimera-VIO-ROS2: humble, master, package/kimera_vio_ros
mikexyl/Kimera-VIO: dev/code-slam, dev/deslam-ros2, cbsms/gtsam-4.3-develop, feature/* branches
mikexyl/aria_common: kimera-ros2, main, develop, feature/adaptive-weighting
mikexyl/aria_visualization: kimera-ros2, main, develop, cbsms/gtsam-4.3-develop
```

The current workspace imports the branch sources above and keeps package layout
matching the repository layout so `vcs import` can recreate the same structure.
For packages that already had a ROS 2 branch but also had newer ROS 1 work, the
local port starts from the ROS 1 branch and merges the ROS 2 branch into that
base. This is currently how `Kimera-Multi-LCD` and `Kimera-Distributed` are
ported locally.

## Binary Runtime Dependencies

`xfeat-cpp` is built against the tested ONNX Runtime GPU release
`onnxruntime-linux-x64-gpu-1.22.0`. The workspace task script downloads this
archive from the Microsoft ONNX Runtime GitHub releases into `src/` when it is
missing, then passes that root to CMake as `onnxruntime_DIR`.

`aria_viz` uses the Rerun C++ SDK. The workspace follows the existing Kimera
ROS 2 workflow and downloads `rerun_cpp_sdk.zip` version `0.35.0` from GitHub
into `.deps/rerun/0.35.0`, installs it locally, and passes its
`rerun_sdk_DIR` to CMake.

The ROS 1 `xfeat-cpp` branch also expects third-party sources under
`src/xfeat-cpp/thirdparty`. The workspace task script initializes the tracked
`gms` and `lightstereo` submodules and checks out the tested `libSGM` fork
commit used by the ROS 1 workspace.

The workspace exports `src/sb_slam_ros2/compat/include` during build to cover
small platform compatibility gaps without editing imported ROS 1 sources, such
as the removed oneTBB `tbb/mutex.h` compatibility header.

Kimera is built against the customized `mikexyl/gtsam` branch
`feature/dynamic_noise_smart_factor`, matching the ROS 1 workspace. This keeps
the dynamic smart-factor noise API used by `vio-factors` and Kimera-VIO.

For the compile milestone, `xfeat-cpp` builds its library target only. The
workspace disables its examples and helper scripts because those auxiliary
targets require TensorRT and GPU FAISS headers that are not part of the current
ROS 2 package closure.

`vio-factors` also builds its library target only; its examples exercise older
GTSAM smart-factor APIs and are outside the imported ROS 2 package closure.

`cbs` builds its library target and vendored robust optimization dependencies
only by default. Its utility and example executables are disabled for the ROS 2
compile milestone because they pull in extra offline-tool dependencies such as
GSL and Abseil that are not needed by `cbs_ros`.

## Compile Milestone

The local `kimera_vio` source is kept close to the ROS 1-derived branch for
now. DBoW2 and the other source-only dependencies used by that branch are
imported into the workspace instead of being removed from Kimera-VIO CMake or
package manifests. Dataset runs and LCD behavior validation are deferred until
the ROS 2 package closure compiles cleanly.

DBoW2 is also retained for the Kimera-Multi LCD and distributed loop-closure
packages. The Rerun visualizer path through `aria_viz` is retained for
`kimera_distributed` and `cbs_ros`.

Builds are capped at 12 workers by default through `BUILD_PARALLEL_JOBS` and
`COLCON_PARALLEL_WORKERS` to keep RAM usage bounded on the current machine.
