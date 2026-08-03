"""Run the GrAco A1/A2 JIST-0.7 shared-sequence comparison."""

from datetime import datetime
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _timestamp():
    return datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%z")


def _workspace_root():
    return Path(os.environ.get("SB_SLAM_ROS2_WS", os.getcwd()))


def _default_jist_model_path():
    model_root = _workspace_root() / "src" / "xfeat-cpp" / "onnx_model"
    candidates = (
        model_root / "JIST_r18_512_seqgem_simplified_fp32.engine",
        model_root / "JIST_r18_512_seqgem_simplified_fp16.engine",
    )
    for model_path in candidates:
        if model_path.is_file():
            return str(model_path)
    return str(candidates[0])


def generate_launch_description():
    timestamp = _timestamp()
    run_name = (
        "jist-0.7-lg-ds-noaug-sharedseq5-boundary0.1-"
        "consecutive-disabled-minsim0.85-distlocal30-sim3-scale0.05-"
        f"{timestamp}"
    )
    output_path = _workspace_root() / "src" / "code-logs" / "a12" / run_name
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("sb_slam_ros2"),
                    "launch",
                    "graco_aerial_01_02_mixvpr_512d_sharedseq5.launch.py",
                ]
            )
        ),
        launch_arguments={
            "vio_dataset_name": "GrAcoStereoXfeatJistDsNoAug",
            "distributed_dataset_name": "GrAcoJistDynamic",
            "vpr_model_type": "jist",
            "vpr_similarity_threshold": "0.7",
            "models.jist": _default_jist_model_path(),
            "log_output_path": str(output_path),
            "rerun_application_id": "graco_a12_jist07_sharedseq5",
            "rerun_recording_id": (
                f"graco_a12_jist07_sharedseq5_{timestamp}"
            ),
        }.items(),
    )
    return LaunchDescription([base_launch])
