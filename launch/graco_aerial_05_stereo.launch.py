"""Compatibility entry point for the A5-only stereo experiment.

The implementation lives in ``graco_aerial_single_robot.launch.py``.  This
wrapper supplies only the physical A5 identity and A5-specific recording
names; all experiment parameters remain directly overridable on the command
line.
"""

from datetime import datetime

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _timestamped_recording_id():
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%z")
    return f"graco_aerial_05_stereo_{timestamp}"


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
                    "num_robots": "4",
                    "image_topic": "/a5/camera_left/image_raw",
                    "right_image_topic": "/a5/camera_right/image_raw",
                    "imu_topic": "/a5/gnss/imu",
                    "log_output_path": "/tmp/sb_slam_ros2_logs/a5_stereo",
                    "rerun_application_id": "graco_aerial_05_stereo",
                    "rerun_recording_id": _timestamped_recording_id(),
                }.items(),
            )
        ]
    )
