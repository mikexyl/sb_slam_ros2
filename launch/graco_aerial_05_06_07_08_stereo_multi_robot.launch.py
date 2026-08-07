"""Run the remote-equivalent GrAco aerial-05/06/07/08 stereo experiment.

The VIO and distributed LCD profiles use the validated GrAco A5678 setup:
GrAcoStereoXfeat, JIST, 1.0x
playback, alpha 0.5, and the original batching/submap/adaptive-scoring
settings. XFeat and LightGlue use the deployment TensorRT engines, and dense
stereo defaults to NVIDIA VPI CUDA SGM at the profile's 512x288 working
resolution. Sim3 CBS is retained, and dense mapping is disabled by the
enclosed base launch.
"""

from datetime import datetime
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _timestamped_recording_id():
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%z")
    return f"graco_aerial_05_06_07_08_stereo_{timestamp}"


def _default_model_path(filename):
    workspace_root = Path(os.environ.get("SB_SLAM_ROS2_WS", os.getcwd()))
    return str(workspace_root / "src" / "xfeat-cpp" / "onnx_model" / filename)


def _default_mixvpr_model_path():
    workspace_root = Path(os.environ.get("SB_SLAM_ROS2_WS", os.getcwd()))
    model_root = workspace_root / "src" / "xfeat-cpp" / "onnx_model"
    candidates = (
        model_root / "trt" / "mixvpr_resnet50_512d_fp16_sm120_trt10.13.engine",
        model_root / "mixvpr_resnet50_512d_fp16.engine",
    )
    for model_path in candidates:
        if model_path.is_file():
            return str(model_path)
    return str(candidates[0])


def generate_launch_description():
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("sb_slam_ros2"),
                    "launch",
                    "graco_aerial_05_06_07_08_multi_robot.launch.py",
                ]
            )
        ),
        launch_arguments={
            "active_robot_ids": LaunchConfiguration("active_robot_ids"),
            "robot_names_file": LaunchConfiguration("robot_names_file"),
            "aerial_05_bag_path": LaunchConfiguration("aerial_05_bag_path"),
            "aerial_06_bag_path": LaunchConfiguration("aerial_06_bag_path"),
            "aerial_07_bag_path": LaunchConfiguration("aerial_07_bag_path"),
            "aerial_08_bag_path": LaunchConfiguration("aerial_08_bag_path"),
            "play_bags": LaunchConfiguration("play_bags"),
            "bag_rate": LaunchConfiguration("bag_rate"),
            "bag_playback_duration": LaunchConfiguration(
                "bag_playback_duration"
            ),
            "bag_start_delay": LaunchConfiguration("bag_start_delay"),
            "vio_mode": "stereo",
            "vio_dataset_name": "GrAcoStereoXfeat",
            "distributed_dataset_name": "GrAcoStereo",
            "vpr_model_type": "jist",
            "jist_frame_refinement": LaunchConfiguration(
                "jist_frame_refinement"
            ),
            "pgo_formulation": LaunchConfiguration("pgo_formulation"),
            "stereo_depth.method": LaunchConfiguration(
                "stereo_depth.method"
            ),
            "models.xfeat": LaunchConfiguration("models.xfeat"),
            "models.lightglue_frontend": LaunchConfiguration(
                "models.lightglue_frontend"
            ),
            "models.lightglue_lcd": LaunchConfiguration(
                "models.lightglue_lcd"
            ),
            "models.mixvpr": LaunchConfiguration("models.mixvpr"),
            "loop_closure.alpha": "0.5",
            "loop_closure.bow_skip_num": "1",
            "loop_closure.bow_batch_size": "50",
            "loop_closure.vlc_batch_size": "10",
            "loop_closure.loop_batch_size": "50",
            "loop_closure.loop_sync_sleep_time": "10",
            "loop_closure.comm_sleep_time": "5",
            "loop_closure.detection_batch_size": "50",
            "loop_closure.max_submap_size": "10",
            "loop_closure.max_submap_distance": "5",
            "loop_closure.adaptive_scoring_tau_max": "0.01",
            "loop_closure.adaptive_scoring_tau_min": "0.01",
            "loop_closure.adaptive_scoring_lambda": "1.0",
            "loop_closure.stereo_verification_method": LaunchConfiguration(
                "loop_closure.stereo_verification_method"
            ),
            "loop_closure.teaser_noise_bound_m": LaunchConfiguration(
                "loop_closure.teaser_noise_bound_m"
            ),
            "loop_closure.teaser_min_scale": LaunchConfiguration(
                "loop_closure.teaser_min_scale"
            ),
            "loop_closure.teaser_max_scale": LaunchConfiguration(
                "loop_closure.teaser_max_scale"
            ),
            "loop_closure.orbslam3_reprojection_threshold_px": LaunchConfiguration(
                "loop_closure.orbslam3_reprojection_threshold_px"
            ),
            "loop_closure.orbslam3_min_scale": LaunchConfiguration(
                "loop_closure.orbslam3_min_scale"
            ),
            "loop_closure.orbslam3_max_scale": LaunchConfiguration(
                "loop_closure.orbslam3_max_scale"
            ),
            "loop_closure.verified_scale_sigma": LaunchConfiguration(
                "loop_closure.verified_scale_sigma"
            ),
            "visualization_mode": LaunchConfiguration("visualization_mode"),
            "log_output": LaunchConfiguration("log_output"),
            "log_output_path": LaunchConfiguration("log_output_path"),
            "rerun_application_id": LaunchConfiguration(
                "rerun_application_id"
            ),
            "rerun_recording_id": LaunchConfiguration("rerun_recording_id"),
            "rerun_host": LaunchConfiguration("rerun_host"),
            "start_zenoh_router": LaunchConfiguration("start_zenoh_router"),
            "zenoh_router_startup_delay": LaunchConfiguration(
                "zenoh_router_startup_delay"
            ),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "active_robot_ids", default_value="0,1,2,3"
            ),
            DeclareLaunchArgument(
                "robot_names_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("kimera_distributed"),
                        "params",
                        "robot_names_graco.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "aerial_05_bag_path",
                default_value="/data/graco/aerial-05-40m",
            ),
            DeclareLaunchArgument(
                "aerial_06_bag_path",
                default_value="/data/graco/aerial-06-20m_stereo_ros2",
            ),
            DeclareLaunchArgument(
                "aerial_07_bag_path",
                default_value="/data/graco/aerial-07-25m_stereo_ros2",
            ),
            DeclareLaunchArgument(
                "aerial_08_bag_path",
                default_value="/data/graco/aerial-08-25m_ros2",
            ),
            DeclareLaunchArgument("play_bags", default_value="true"),
            DeclareLaunchArgument("bag_rate", default_value="1.0"),
            DeclareLaunchArgument(
                "bag_playback_duration", default_value="-1"
            ),
            DeclareLaunchArgument(
                "bag_start_delay",
                default_value="30.0",
                description=(
                    "Delay playback until all TensorRT engines initialize."
                ),
            ),
            DeclareLaunchArgument(
                "stereo_depth.method", default_value="VPI_CUDA"
            ),
            DeclareLaunchArgument(
                "jist_frame_refinement", default_value="true"
            ),
            DeclareLaunchArgument(
                "pgo_formulation", default_value="sim3"
            ),
            DeclareLaunchArgument(
                "loop_closure.stereo_verification_method",
                default_value="opengv_pnp",
            ),
            DeclareLaunchArgument(
                "loop_closure.teaser_noise_bound_m", default_value="0.10"
            ),
            DeclareLaunchArgument(
                "loop_closure.teaser_min_scale", default_value="0.5"
            ),
            DeclareLaunchArgument(
                "loop_closure.teaser_max_scale", default_value="2.0"
            ),
            DeclareLaunchArgument(
                "loop_closure.orbslam3_reprojection_threshold_px",
                default_value="15.0",
            ),
            DeclareLaunchArgument(
                "loop_closure.orbslam3_min_scale", default_value="0.5"
            ),
            DeclareLaunchArgument(
                "loop_closure.orbslam3_max_scale", default_value="2.0"
            ),
            DeclareLaunchArgument(
                "loop_closure.verified_scale_sigma", default_value="0.10"
            ),
            DeclareLaunchArgument(
                "models.xfeat",
                default_value=_default_model_path(
                    "xfeat_320x224_fp16.engine"
                ),
            ),
            DeclareLaunchArgument(
                "models.lightglue_frontend",
                default_value=_default_model_path(
                    "lg_320x224_dyn_min1_fp16.engine"
                ),
            ),
            DeclareLaunchArgument(
                "models.lightglue_lcd",
                default_value=_default_model_path(
                    "lg_320x224_dyn_min1_fp16.engine"
                ),
            ),
            DeclareLaunchArgument(
                "models.mixvpr",
                default_value=_default_mixvpr_model_path(),
            ),
            DeclareLaunchArgument(
                "visualization_mode", default_value="minimal"
            ),
            DeclareLaunchArgument("log_output", default_value="false"),
            DeclareLaunchArgument(
                "log_output_path",
                default_value="/tmp/sb_slam_ros2_logs/a5678_stereo",
            ),
            DeclareLaunchArgument(
                "rerun_application_id",
                default_value="graco_aerial_05_06_07_08_stereo",
            ),
            DeclareLaunchArgument(
                "rerun_recording_id",
                default_value=_timestamped_recording_id(),
                description=(
                    "Use the same value on every participating host."
                ),
            ),
            DeclareLaunchArgument(
                "rerun_host",
                default_value="rerun+http://127.0.0.1:9876/proxy",
            ),
            DeclareLaunchArgument("start_zenoh_router", default_value="true"),
            DeclareLaunchArgument(
                "zenoh_router_startup_delay", default_value="1.0"
            ),
            base_launch,
        ]
    )
