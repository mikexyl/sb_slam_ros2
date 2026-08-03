"""Run the A5678 fixed-five-keyframe JIST sequence ablation.

All orchestration, model, loop-closure, CBS, playback, and visualization
settings come from the proven JIST 0.7 profile. Only the VIO parameter folder
changes: it disables dynamic sequence gates and finalizes every five admitted
keyframes.
"""

from datetime import datetime
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _run_identity():
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%z")
    workspace_root = Path(os.environ.get("SB_SLAM_ROS2_WS", os.getcwd()))
    run_name = f"jist-aug-fixed5-distlocal90-jist0.7-sim3-{timestamp}"
    output_path = workspace_root / "src" / "code-logs" / "a5678" / run_name
    recording_id = f"graco_a5678_jist_aug_fixed5_{timestamp}"
    return str(output_path), recording_id


def generate_launch_description():
    output_path, recording_id = _run_identity()
    reference_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("sb_slam_ros2"),
                    "launch",
                    "graco_aerial_05_06_07_08_jist_aug_ds.launch.py",
                ]
            )
        ),
        launch_arguments={
            "vio_dataset_name": "GrAcoStereoXfeatJistAugFixed5",
            "distributed_dataset_name": "GrAcoJistFixed5",
            "log_output_path": output_path,
            "rerun_application_id": "graco_a5678_jist_aug_fixed5",
            "rerun_recording_id": recording_id,
        }.items(),
    )
    return LaunchDescription([reference_launch])
