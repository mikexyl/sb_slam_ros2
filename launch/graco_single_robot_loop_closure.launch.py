"""Run one GrAco robot through VIO, Distributed LCD, and CBS."""

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
    return f"graco_single_robot_lcd_{timestamp}"


def _launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    if not robot_name:
        raise RuntimeError("robot_name must not be empty")

    names_path = Path(
        LaunchConfiguration("robot_names_file").perform(context)
    )
    if not names_path.is_file():
        raise RuntimeError("robot_names_file must name an existing YAML file")
    with names_path.open("r", encoding="utf-8") as stream:
        names = yaml.safe_load(stream)
    if not isinstance(names, dict):
        raise RuntimeError("robot_names_file must contain a mapping")
    if names.get("robot0_name") != robot_name:
        raise RuntimeError(
            f"robot_name '{robot_name}' does not match robot0_name "
            f"'{names.get('robot0_name')}'"
        )

    bag_path = Path(LaunchConfiguration("bag_path").perform(context))
    if not bag_path.is_dir() or not (bag_path / "metadata.yaml").is_file():
        raise RuntimeError(
            "bag_path must name a ROS 2 bag directory containing metadata.yaml"
        )

    recording_id = LaunchConfiguration("rerun_recording_id").perform(context)
    if not recording_id:
        raise RuntimeError("rerun_recording_id must not be empty")

    robot_launch = PathJoinSubstitution(
        [FindPackageShare("sb_slam_ros2"), "launch", "graco_robot.launch.py"]
    )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch),
            launch_arguments={
                "robot_id": "0",
                "robot_name": robot_name,
                "num_robots": "1",
                "robot_names_file": str(names_path),
                "models.xfeat": LaunchConfiguration("models.xfeat"),
                "models.lightglue_frontend": LaunchConfiguration(
                    "models.lightglue_frontend"
                ),
                "models.lightglue_lcd": LaunchConfiguration(
                    "models.lightglue_lcd"
                ),
                "models.jist": LaunchConfiguration("models.jist"),
                "models.da3": LaunchConfiguration("models.da3"),
                "dense_mapping.enabled": "false",
                "keyframe_state.publisher_enabled": "true",
                "vocabulary_path": LaunchConfiguration("vocabulary_path"),
                "descriptor_batch_size": LaunchConfiguration(
                    "descriptor_batch_size"
                ),
                "descriptor_stride": LaunchConfiguration("descriptor_stride"),
                "verification_frame_batch_size": LaunchConfiguration(
                    "verification_frame_batch_size"
                ),
                "flush_period_s": LaunchConfiguration("flush_period_s"),
                "play_bag": "true",
                "use_sim_time": "true",
                "bag_publish_clock": "true",
                "bag_path": str(bag_path),
                "bag_rate": LaunchConfiguration("bag_rate"),
                "bag_source_image_topic": LaunchConfiguration(
                    "bag_source_image_topic"
                ),
                "bag_source_imu_topic": LaunchConfiguration(
                    "bag_source_imu_topic"
                ),
                "log_output_path": LaunchConfiguration("log_output_path"),
                "rerun_application_id": LaunchConfiguration(
                    "rerun_application_id"
                ),
                "rerun_recording_id": recording_id,
                "rerun_host": LaunchConfiguration("rerun_host"),
                "start_zenoh_router": "false",
            }.items(),
        )
    ]


def generate_launch_description():
    zenoh_router = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("start_zenoh_router")),
        cmd=["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"],
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="a5"),
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
                "models.lightglue_frontend", default_value=""
            ),
            DeclareLaunchArgument("models.lightglue_lcd", default_value=""),
            DeclareLaunchArgument("models.jist", default_value=""),
            DeclareLaunchArgument(
                "models.da3",
                default_value=(
                    "/home/mikexyl/workspaces/sb_slam_ros2_ws/src/"
                    "xfeat-cpp/onnx_model/"
                    "DA3METRIC-LARGE_280x504_fp16.engine"
                ),
            ),
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
                "bag_source_image_topic", default_value="/camera_left/image_raw"
            ),
            DeclareLaunchArgument(
                "bag_source_imu_topic", default_value="/gnss/imu"
            ),
            DeclareLaunchArgument(
                "log_output_path", default_value="/tmp/sb_slam_ros2_logs"
            ),
            DeclareLaunchArgument(
                "rerun_application_id",
                default_value="graco_single_robot_loop_closure",
            ),
            DeclareLaunchArgument(
                "rerun_recording_id",
                default_value=_timestamped_recording_id(),
                description=(
                    "Shared Rerun recording id generated from the launching "
                    "system's local timestamp."
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
