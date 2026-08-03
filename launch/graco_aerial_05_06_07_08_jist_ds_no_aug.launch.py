"""Run A5678 with JIST 0.7, dynamic sequences, and no augmentation."""

from datetime import datetime
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _default_bag_path(remote_path, local_path):
    return remote_path if Path(remote_path).is_dir() else local_path


def _default_jist_model_path():
    workspace_root = Path(os.environ.get("SB_SLAM_ROS2_WS", os.getcwd()))
    model_root = workspace_root / "src" / "xfeat-cpp" / "onnx_model"
    for filename in (
        "JIST_r18_512_seqgem_frames_fp32.engine",
    ):
        model_path = model_root / filename
        if model_path.is_file():
            return str(model_path)
    return str(model_root / "JIST_r18_512_seqgem_frames_fp32.engine")


def _run_identity(similarity_threshold):
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%z")
    workspace_root = Path(os.environ.get("SB_SLAM_ROS2_WS", os.getcwd()))
    run_name = (
        "jist-ds-noaug-boundary0.1-consecutive-disabled-"
        f"jist{similarity_threshold}-minsim0.85-distlocal30-sim3-"
        f"{timestamp}"
    )
    output_path = workspace_root / "src" / "code-logs" / "a5678" / run_name
    threshold_id = similarity_threshold.replace(".", "")
    recording_id = (
        f"graco_a5678_jist{threshold_id}_ds_noaug_{timestamp}"
    )
    return str(output_path), recording_id


def _launch_setup(context):
    similarity_threshold = LaunchConfiguration(
        "jist_similarity_threshold"
    ).perform(context)
    try:
        threshold_value = float(similarity_threshold)
    except ValueError as error:
        raise RuntimeError(
            "jist_similarity_threshold must be a number"
        ) from error
    if not 0.0 <= threshold_value <= 1.0:
        raise RuntimeError(
            "jist_similarity_threshold must be between 0 and 1"
        )
    similarity_threshold = format(threshold_value, "g")
    output_path, recording_id = _run_identity(similarity_threshold)
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
            "vio_dataset_name": "GrAcoStereoXfeatJistDsNoAug",
            "distributed_dataset_name": "GrAcoJistDynamic",
            "vpr_model_type": "jist",
            "jist_frame_refinement": "false",
            "models.jist": _default_jist_model_path(),
            "loop_closure.min_sim_vlad": similarity_threshold,
            "bag_rate": "1.0",
            "visualization_mode": "minimal",
            "log_output": "true",
            "log_output_path": output_path,
            "rerun_application_id": (
                f"graco_a5678_jist{similarity_threshold.replace('.', '')}_"
                "ds_noaug"
            ),
            "rerun_recording_id": recording_id,
            "rerun_host": "rerun+http://192.168.0.206:9876/proxy",
            "start_zenoh_router": "false",
        }.items(),
    )
    return [reference_launch]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "jist_similarity_threshold",
                default_value="0.7",
                description="JIST cosine-similarity candidate threshold.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
