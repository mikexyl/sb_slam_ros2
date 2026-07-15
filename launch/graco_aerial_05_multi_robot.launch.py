"""Run aerial-05 as multiple logical robots in one Rerun recording."""

from datetime import datetime
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _timestamped_recording_id():
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%z")
    return f"graco_aerial_05_{timestamp}"


def _launch_setup(context, *args, **kwargs):
    num_robots = int(LaunchConfiguration("num_robots").perform(context))
    if num_robots < 2:
        raise RuntimeError(
            "aerial-05 inter-robot loop closure requires at least two "
            "logical robots"
        )

    names_path = Path(
        LaunchConfiguration("robot_names_file").perform(context)
    )
    if not names_path.is_file():
        raise RuntimeError("robot_names_file must name an existing YAML file")
    with names_path.open("r", encoding="utf-8") as stream:
        names = yaml.safe_load(stream)
    if not isinstance(names, dict):
        raise RuntimeError("robot_names_file must contain a mapping")

    recording_id = LaunchConfiguration("rerun_recording_id").perform(context)
    if not recording_id:
        raise RuntimeError("rerun_recording_id must not be empty")

    robot_launch = PathJoinSubstitution(
        [FindPackageShare("sb_slam_ros2"), "launch", "graco_robot.launch.py"]
    )
    shared_arguments = {
        "num_robots": str(num_robots),
        "robot_names_file": str(names_path),
        "models.xfeat": LaunchConfiguration("models.xfeat"),
        "models.xfeat_interp_bilinear": LaunchConfiguration(
            "models.xfeat_interp_bilinear"
        ),
        "models.xfeat_interp_bicubic": LaunchConfiguration(
            "models.xfeat_interp_bicubic"
        ),
        "models.xfeat_interp_nearest": LaunchConfiguration(
            "models.xfeat_interp_nearest"
        ),
        "models.lightglue_frontend": LaunchConfiguration(
            "models.lightglue_frontend"
        ),
        "models.lightglue_lcd": LaunchConfiguration("models.lightglue_lcd"),
        "models.jist": LaunchConfiguration("models.jist"),
        "vocabulary_path": LaunchConfiguration("vocabulary_path"),
        "descriptor_batch_size": LaunchConfiguration("descriptor_batch_size"),
        "descriptor_stride": LaunchConfiguration("descriptor_stride"),
        "verification_frame_batch_size": LaunchConfiguration(
            "verification_frame_batch_size"
        ),
        "flush_period_s": LaunchConfiguration("flush_period_s"),
        "play_bag": "true",
        "bag_path": LaunchConfiguration("bag_path"),
        "bag_rate": LaunchConfiguration("bag_rate"),
        "bag_source_image_topic": "/camera_left/image_raw",
        "bag_source_imu_topic": "/gnss/imu",
        "log_output_path": LaunchConfiguration("log_output_path"),
        "rerun_application_id": LaunchConfiguration("rerun_application_id"),
        "rerun_recording_id": recording_id,
        "rerun_host": LaunchConfiguration("rerun_host"),
        "start_zenoh_router": "false",
    }

    robot_actions = []
    for robot_id in range(num_robots):
        robot_name = names.get(f"robot{robot_id}_name")
        if not isinstance(robot_name, str) or not robot_name:
            raise RuntimeError(
                f"robot_names_file is missing robot{robot_id}_name"
            )
        robot_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_launch),
                launch_arguments={
                    **shared_arguments,
                    "robot_id": str(robot_id),
                    "robot_name": robot_name,
                    "bag_publish_clock": (
                        "true" if robot_id == 0 else "false"
                    ),
                }.items(),
            )
        )
    return robot_actions


def generate_launch_description():
    zenoh_router = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("start_zenoh_router")),
        cmd=["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"],
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("num_robots", default_value="2"),
            DeclareLaunchArgument(
                "robot_names_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("kimera_distributed"),
                        "params",
                        "robot_names_graco.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("models.xfeat", default_value=""),
            DeclareLaunchArgument(
                "models.xfeat_interp_bilinear", default_value=""
            ),
            DeclareLaunchArgument(
                "models.xfeat_interp_bicubic", default_value=""
            ),
            DeclareLaunchArgument(
                "models.xfeat_interp_nearest", default_value=""
            ),
            DeclareLaunchArgument(
                "models.lightglue_frontend", default_value=""
            ),
            DeclareLaunchArgument("models.lightglue_lcd", default_value=""),
            DeclareLaunchArgument("models.jist", default_value=""),
            DeclareLaunchArgument("vocabulary_path", default_value=""),
            DeclareLaunchArgument("descriptor_batch_size", default_value="5"),
            DeclareLaunchArgument("descriptor_stride", default_value="1"),
            DeclareLaunchArgument(
                "verification_frame_batch_size", default_value="50"
            ),
            DeclareLaunchArgument("flush_period_s", default_value="1.0"),
            DeclareLaunchArgument(
                "bag_path", default_value="/data/graco/aerial-05-40m"
            ),
            DeclareLaunchArgument("bag_rate", default_value="1.0"),
            DeclareLaunchArgument(
                "log_output_path", default_value="/tmp/sb_slam_ros2_logs"
            ),
            DeclareLaunchArgument(
                "rerun_application_id",
                default_value="graco_aerial_05_multi_robot",
            ),
            DeclareLaunchArgument(
                "rerun_recording_id",
                default_value=_timestamped_recording_id(),
                description=(
                    "Shared Rerun recording id. The default contains the "
                    "launching system's local timestamp."
                ),
            ),
            DeclareLaunchArgument(
                "rerun_host",
                default_value="rerun+http://127.0.0.1:9876/proxy",
            ),
            DeclareLaunchArgument("start_zenoh_router", default_value="false"),
            zenoh_router,
            OpaqueFunction(function=_launch_setup),
        ]
    )
