"""Run the apple-to-apple A5678 MixVPR-512D shared-sequence comparison."""

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


def _default_mixvpr_model_path():
    workspace_root = Path(os.environ.get("SB_SLAM_ROS2_WS", os.getcwd()))
    model_root = workspace_root / "src" / "xfeat-cpp" / "onnx_model"
    candidates = (
        model_root
        / "trt"
        / "mixvpr_resnet50_512d_fp16_sm120_trt10.13.engine",
        model_root / "mixvpr_resnet50_512d_fp16.engine",
    )
    for model_path in candidates:
        if model_path.is_file():
            return str(model_path)
    return str(candidates[0])


def _run_identity(similarity_threshold):
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%z")
    workspace_root = Path(os.environ.get("SB_SLAM_ROS2_WS", os.getcwd()))
    run_name = (
        f"mixvpr-{similarity_threshold}-512d-lg-ds-noaug-sharedseq5-"
        "boundary0.1-"
        "consecutive-disabled-minsim0.85-distlocal30-sim3-scale0.05-"
        f"{timestamp}"
    )
    output_path = workspace_root / "src" / "code-logs" / "a5678" / run_name
    threshold_id = similarity_threshold.replace(".", "")
    recording_id = (
        f"graco_a5678_mixvpr{threshold_id}_512d_sharedseq5_{timestamp}"
    )
    return str(output_path), recording_id


def _launch_setup(context):
    similarity_threshold = LaunchConfiguration(
        "mixvpr_similarity_threshold"
    ).perform(context)
    try:
        threshold_value = float(similarity_threshold)
    except ValueError as error:
        raise RuntimeError(
            "mixvpr_similarity_threshold must be a number"
        ) from error
    if not 0.0 <= threshold_value <= 1.0:
        raise RuntimeError(
            "mixvpr_similarity_threshold must be between 0 and 1"
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
            "vio_dataset_name": "GrAcoStereoXfeatMixVprDsNoAugSharedSeq5",
            "distributed_dataset_name": "GrAcoMixVprDynamic",
            "vpr_model_type": "mixvpr",
            "models.mixvpr": _default_mixvpr_model_path(),
            "loop_closure.min_sim_vlad": similarity_threshold,
            "bag_rate": "1.0",
            "visualization_mode": "minimal",
            "log_output": "true",
            "log_output_path": output_path,
            "rerun_application_id": (
                f"graco_a5678_mixvpr{similarity_threshold.replace('.', '')}_"
                "512d_sharedseq5"
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
                "mixvpr_similarity_threshold",
                default_value="0.6",
                description="MixVPR cosine-similarity candidate threshold.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
