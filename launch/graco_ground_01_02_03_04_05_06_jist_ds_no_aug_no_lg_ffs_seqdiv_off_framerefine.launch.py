"""Run Ground 1-6 with FFS, refined JIST 0.8, and sequence diversity off."""

from datetime import datetime
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%z")
    workspace_root = Path(os.environ.get("SB_SLAM_ROS2_WS", os.getcwd()))
    run_name = (
        "jist-ds-noaug-nolg-ffs-boundary0.1-consecutive-disabled-"
        "jist0.8-seqdiv-off-distlocal30-sim3-framerefine-argmax-"
        f"{timestamp}"
    )
    output_path = workspace_root / "src" / "code-logs" / "g123456" / run_name
    recording_id = (
        "graco_g123456_jist08_ds_noaug_nolg_ffs_seqdiv_off_framerefine_"
        f"{timestamp}"
    )
    stereo_engine = (
        workspace_root
        / "src"
        / "xfeat-cpp"
        / "onnx_model"
        / "fast_foundation_stereo"
        / "fast_foundationstereo.engine"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sim3_anchor_scale_prior_sigma", default_value="-1"
            ),
            DeclareLaunchArgument(
                "log_output_path", default_value=str(output_path)
            ),
            DeclareLaunchArgument(
                "rerun_application_id",
                default_value=(
                    "graco_g123456_jist08_ds_noaug_nolg_ffs_"
                    "seqdiv_off_framerefine"
                ),
            ),
            DeclareLaunchArgument(
                "rerun_recording_id", default_value=recording_id
            ),
            DeclareLaunchArgument(
                "rerun_host",
                default_value="rerun+http://192.168.0.206:9876/proxy",
            ),
            DeclareLaunchArgument("start_zenoh_router", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("sb_slam_ros2"),
                            "launch",
                            (
                                "graco_ground_01_02_03_04_05_06_"
                                "jist_ds_no_aug_no_lg.launch.py"
                            ),
                        ]
                    )
                ),
                launch_arguments={
                    "jist_frame_refinement": "true",
                    "loop_closure.min_sim_score": "0.0",
                    "stereo_depth.method": "FastFoundationStereo",
                    "models.stereo_depth": str(stereo_engine),
                    "sim3_anchor_scale_prior_sigma": LaunchConfiguration(
                        "sim3_anchor_scale_prior_sigma"
                    ),
                    "log_output": "true",
                    "log_output_path": LaunchConfiguration(
                        "log_output_path"
                    ),
                    "rerun_application_id": LaunchConfiguration(
                        "rerun_application_id"
                    ),
                    "rerun_recording_id": LaunchConfiguration(
                        "rerun_recording_id"
                    ),
                    "rerun_host": LaunchConfiguration("rerun_host"),
                    "start_zenoh_router": LaunchConfiguration(
                        "start_zenoh_router"
                    ),
                }.items(),
            ),
        ]
    )
