"""Run the A5678 MixVPR-0.6 stride-five, no-dynamic-sequence ablation.

All VIO, loop-closure, CBS, playback, and visualization settings come from the
proven augmented A5678 reference. MixVPR replaces JIST, uses the 0.6 database
threshold, and processes frame IDs 0, 5, 10, ... for its 4096-D descriptor.
Covisibility and self-similarity sequence gates are disabled in the named VIO
profile.
"""

from datetime import datetime
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _default_bag_path(remote_path, local_path):
    """Prefer the 148 data volume while remaining runnable locally."""
    return remote_path if Path(remote_path).is_dir() else local_path


def _run_identity():
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%z")
    workspace_root = Path(os.environ.get("SB_SLAM_ROS2_WS", os.getcwd()))
    run_name = (
        "mixvpr-0.6-aug-no-dynamic-interval5-"
        f"distlocal90-sim3-{timestamp}"
    )
    output_path = workspace_root / "src" / "code-logs" / "a5678" / run_name
    recording_id = f"graco_a5678_mixvpr06_no_dynamic_i5_{timestamp}"
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
            "aerial_05_bag_path": _default_bag_path(
                "/data3/graco/aerial-05-40m_full_ros2",
                "/data/graco/aerial-05-40m",
            ),
            "aerial_06_bag_path": _default_bag_path(
                "/data3/graco/aerial-06-20m_full_ros2",
                "/data/graco/aerial-06-20m_stereo_ros2",
            ),
            "aerial_07_bag_path": _default_bag_path(
                "/data3/graco/aerial-07-25m_full_ros2",
                "/data/graco/aerial-07-25m_stereo_ros2",
            ),
            "aerial_08_bag_path": _default_bag_path(
                "/data3/graco/aerial-08-25m_full_ros2",
                "/data/graco/aerial-08-25m_ros2",
            ),
            "bag_rate": "1.0",
            "vio_dataset_name": "GrAcoStereoXfeatMixVprAugNoDynamicI5",
            "distributed_dataset_name": "GrAcoMixVprNoDynamic",
            "vpr_model_type": "mixvpr",
            "log_output": "true",
            "log_output_path": output_path,
            "rerun_application_id": "graco_a5678_mixvpr06_no_dynamic_i5",
            "rerun_recording_id": recording_id,
            "rerun_host": "rerun+http://192.168.0.206:9876/proxy",
            "start_zenoh_router": "false",
        }.items(),
    )
    return LaunchDescription([reference_launch])
