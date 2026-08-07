"""Run named TEASER++ Sim3 with the legacy 0.05 anchor-belief profile."""

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
        "jist-ds-noaug-nolg-ffs-boundary0.05-consecutive-disabled-"
        "jist0.8-seqdiv-off-distlocal30-sim3-framerefine-argmax-"
        f"anchorprior0.05-teaser-sim3-{timestamp}"
    )
    output_path = workspace_root / "src" / "code-logs" / "g123456" / run_name
    recording_id = (
        "graco_g123456_jist08_ffs_boundary005_seqdiv_off_framerefine_"
        f"anchorprior005_teaser_sim3_{timestamp}"
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
                    "vio_dataset_name": (
                        "GrAcoGndStereoXfeatJistDsNoAugNoLgBoundary005"
                    ),
                    "jist_frame_refinement": "true",
                    "loop_closure.min_sim_score": "0.0",
                    "stereo_depth.method": "FastFoundationStereo",
                    "models.stereo_depth": str(stereo_engine),
                    "sim3_pose_scale_prior_sigma": "0.05",
                    "loop_closure.stereo_verification_method": "teaser_sim3",
                    "loop_closure.teaser_noise_bound_m": "0.10",
                    "loop_closure.teaser_min_scale": "0.5",
                    "loop_closure.teaser_max_scale": "2.0",
                    "loop_closure.verified_scale_sigma": "0.10",
                    "log_output": "true",
                    "log_output_path": str(output_path),
                    "rerun_application_id": (
                        "graco_g123456_jist08_ffs_boundary005_seqdiv_off_"
                        "framerefine_anchorprior_teaser_sim3"
                    ),
                    "rerun_recording_id": recording_id,
                    "rerun_host": "rerun+http://192.168.0.206:9876/proxy",
                    "start_zenoh_router": "false",
                }.items(),
            )
        ]
    )
