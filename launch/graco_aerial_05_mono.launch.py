"""Run a5 alone with the established GrAco monocular profiles.

The reusable implementation lives in ``graco_aerial_single_robot.launch.py``.
This wrapper fixes the a5 identity, selects monocular VIO with JIST loop
retrieval, and uses a one-robot graph so CBS does not wait for A6-A8.
"""

from datetime import datetime

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _timestamped_recording_id():
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%z")
    return f"graco_a5_mono_{timestamp}"


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("sb_slam_ros2"),
                            "launch",
                            "graco_aerial_single_robot.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "robot_id": "0",
                    "robot_name": "a5",
                    "num_robots": "1",
                    "vio_mode": "mono",
                    "vio_dataset_name": "GrAcoMonoXfeat",
                    "distributed_dataset_name": "GrAco",
                    "vpr_model_type": "jist",
                    "jist_frame_refinement": "false",
                    "image_topic": "/a5/camera_left/image_raw",
                    "imu_topic": "/a5/gnss/imu",
                    "stereo_depth.method": "",
                    "log_output_path": "/tmp/sb_slam_ros2_logs/a5_mono",
                    "rerun_application_id": "graco_a5_mono",
                    "rerun_recording_id": _timestamped_recording_id(),
                    "bag_path": "/data/graco/aerial-05-40m"
                }.items(),
            )
        ]
    )
