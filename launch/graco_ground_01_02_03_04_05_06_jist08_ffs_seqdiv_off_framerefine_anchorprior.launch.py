"""Run refined Ground 1-6 JIST 0.8 with a 0.05 anchor scale prior."""

from datetime import datetime
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%z")
    workspace_root = Path(os.environ.get("SB_SLAM_ROS2_WS", os.getcwd()))
    run_name = (
        "jist-ds-noaug-nolg-ffs-boundary0.1-consecutive-disabled-"
        "jist0.8-seqdiv-off-distlocal30-sim3-framerefine-argmax-"
        f"anchorprior0.05-{timestamp}"
    )
    output_path = workspace_root / "src" / "code-logs" / "g123456" / run_name
    recording_id = (
        "graco_g123456_jist08_ffs_seqdiv_off_framerefine_"
        f"anchorprior005_{timestamp}"
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("sb_slam_ros2"),
                            "launch",
                            (
                                "graco_ground_01_02_03_04_05_06_jist_ds_"
                                "no_aug_no_lg_ffs_seqdiv_off_framerefine."
                                "launch.py"
                            ),
                        ]
                    )
                ),
                launch_arguments={
                    "sim3_anchor_scale_prior_sigma": "0.05",
                    "log_output_path": str(output_path),
                    "rerun_application_id": (
                        "graco_g123456_jist08_ffs_seqdiv_off_"
                        "framerefine_anchorprior"
                    ),
                    "rerun_recording_id": recording_id,
                    "start_zenoh_router": "false",
                }.items(),
            )
        ]
    )
