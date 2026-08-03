"""Run Ground 1–6 with TensorRT MixVPR-512D and JIST-equivalent sequences."""

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
    run_name = (
        "mixvpr-0.6-512d-sharedseq5-ds-noaug-nolg-"
        "boundary0.1-consecutive-disabled-"
        f"minsim0.85-distlocal30-sim3-{timestamp}"
    )
    output_path = workspace_root / "src" / "code-logs" / "g123456" / run_name
    recording_id = (
        f"graco_g123456_mixvpr06_512d_sharedseq5_ds_noaug_nolg_{timestamp}"
    )
    return str(output_path), recording_id


def generate_launch_description():
    output_path, recording_id = _run_identity()
    reference_launch = IncludeLaunchDescription(
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
            "vio_dataset_name": "GrAcoGndStereoXfeatMixVprDsNoAugNoLg",
            "distributed_dataset_name": "GrAcoMixVprDynamic",
            "vpr_model_type": "mixvpr",
            "log_output": "true",
            "log_output_path": output_path,
            "rerun_application_id": (
                "graco_g123456_mixvpr06_512d_sharedseq5_ds_noaug_nolg"
            ),
            "rerun_recording_id": recording_id,
            "rerun_host": "rerun+http://192.168.0.206:9876/proxy",
            "start_zenoh_router": "false",
        }.items(),
    )
    return LaunchDescription([reference_launch])
