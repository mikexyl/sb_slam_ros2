"""Run the A5678 JIST frame-argmax ablation with the endpoint setup."""

from datetime import datetime
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _default_bag_path(remote_path, local_path):
    return remote_path if Path(remote_path).is_dir() else local_path


def _default_jist_model_path():
    workspace_root = Path(os.environ.get("SB_SLAM_ROS2_WS", os.getcwd()))
    return str(
        workspace_root
        / "src"
        / "xfeat-cpp"
        / "onnx_model"
        / "JIST_r18_512_seqgem_frames_fp32.engine"
    )


def _run_identity():
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%z")
    workspace_root = Path(os.environ.get("SB_SLAM_ROS2_WS", os.getcwd()))
    run_name = (
        "jist-ds-noaug-nolg-boundary0.1-consecutive-disabled-"
        f"jist0.7-minsim0.85-distlocal30-sim3-framerefine-argmax-{timestamp}"
    )
    output_path = workspace_root / "src" / "code-logs" / "a5678" / run_name
    recording_id = (
        f"graco_a5678_jist07_ds_noaug_nolg_framerefine_argmax_{timestamp}"
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
            "vio_dataset_name": "GrAcoStereoXfeatJistDsNoAugNoLg",
            "distributed_dataset_name": "GrAcoJistDynamic",
            "vpr_model_type": "jist",
            "jist_frame_refinement": "true",
            "models.jist": _default_jist_model_path(),
            "loop_closure.min_sim_vlad": "0.7",
            "bag_rate": "1.0",
            "visualization_mode": "minimal",
            "log_output": "true",
            "log_output_path": output_path,
            "rerun_application_id": (
                "graco_a5678_jist07_ds_noaug_nolg_framerefine_argmax"
            ),
            "rerun_recording_id": recording_id,
            "rerun_host": "rerun+http://192.168.0.206:9876/proxy",
            "start_zenoh_router": "false",
        }.items(),
    )
    return LaunchDescription([reference_launch])
