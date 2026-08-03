"""Run aerial-05 and aerial-07 with distributed loop closure and CBS."""

from datetime import datetime
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


_RECORDED_TOPICS = (
    "/a5/kimera_vio/pose_graph/updates",
    "/a5/kimera_vio/mapping/local_window_poses",
    "/a5/kimera_vio/mapping/keyframes",
    "/a5/kimera_distributed/pose_graph/updates",
    "/a5/kimera_distributed/keyframe_loop_closures",
    "/a7/kimera_vio/pose_graph/updates",
    "/a7/kimera_vio/mapping/local_window_poses",
    "/a7/kimera_vio/mapping/keyframes",
    "/a7/kimera_distributed/pose_graph/updates",
    "/a7/kimera_distributed/keyframe_loop_closures",
)


def _timestamp():
    return datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%z")


def _timestamped_recording_id():
    return f"graco_aerial_05_07_{_timestamp()}"


def _timestamped_output_bag_path():
    return f"/data/graco/kimera-output-aerial05-aerial07-{_timestamp()}"


def _launch_setup(context, *args, **kwargs):
    names_path = Path(LaunchConfiguration("robot_names_file").perform(context))
    if not names_path.is_file():
        raise RuntimeError("robot_names_file must name an existing YAML file")
    with names_path.open("r", encoding="utf-8") as stream:
        names = yaml.safe_load(stream)
    if not isinstance(names, dict):
        raise RuntimeError("robot_names_file must contain a mapping")

    expected_names = ("a5", "a7")
    for robot_id, expected_name in enumerate(expected_names):
        actual_name = names.get(f"robot{robot_id}_name")
        if actual_name != expected_name:
            raise RuntimeError(
                f"robot{robot_id}_name must be {expected_name}, got {actual_name}"
            )

    recording_id = LaunchConfiguration("rerun_recording_id").perform(context)
    if not recording_id:
        raise RuntimeError("rerun_recording_id must not be empty")

    bag_paths = (
        LaunchConfiguration("aerial_05_bag_path").perform(context),
        LaunchConfiguration("aerial_07_bag_path").perform(context),
    )
    for bag_path in bag_paths:
        if not Path(bag_path).is_dir():
            raise RuntimeError(f"ROS 2 bag directory does not exist: {bag_path}")

    output_bag_path = Path(
        LaunchConfiguration("output_bag_path").perform(context)
    )
    if output_bag_path.exists():
        raise RuntimeError(f"output_bag_path already exists: {output_bag_path}")
    if not output_bag_path.parent.is_dir():
        raise RuntimeError(
            f"output_bag_path parent does not exist: {output_bag_path.parent}"
        )

    robot_launch = PathJoinSubstitution(
        [FindPackageShare("sb_slam_ros2"), "launch", "graco_robot.launch.py"]
    )
    shared_arguments = {
        "num_robots": "2",
        "robot_names_file": str(names_path),
        "models.xfeat": LaunchConfiguration("models.xfeat"),
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
        "loop_closure.alpha": LaunchConfiguration("loop_closure.alpha"),
        "loop_closure.adaptive_scoring_tau_max": LaunchConfiguration(
            "loop_closure.adaptive_scoring_tau_max"
        ),
        "loop_closure.adaptive_scoring_tau_min": LaunchConfiguration(
            "loop_closure.adaptive_scoring_tau_min"
        ),
        "loop_closure.adaptive_scoring_lambda": LaunchConfiguration(
            "loop_closure.adaptive_scoring_lambda"
        ),
        "play_bag": "true",
        "use_sim_time": "false",
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
    for robot_id, (robot_name, bag_path) in enumerate(
        zip(expected_names, bag_paths)
    ):
        robot_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_launch),
                launch_arguments={
                    **shared_arguments,
                    "robot_id": str(robot_id),
                    "robot_name": robot_name,
                    "bag_path": bag_path,
                    "bag_publish_clock": "false",
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
    recorder = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("record_output")),
        cmd=[
            "ros2",
            "bag",
            "record",
            "--storage",
            "sqlite3",
            "--output",
            LaunchConfiguration("output_bag_path"),
            *_RECORDED_TOPICS,
        ],
        output="screen",
        on_exit=EmitEvent(
            event=Shutdown(reason="A5/A7 output recorder exited")
        ),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_names_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("kimera_distributed"),
                        "params",
                        "robot_names_graco_57.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("models.xfeat", default_value=""),
            DeclareLaunchArgument("models.lightglue_frontend", default_value=""),
            DeclareLaunchArgument("models.lightglue_lcd", default_value=""),
            DeclareLaunchArgument("models.jist", default_value=""),
            DeclareLaunchArgument("vocabulary_path", default_value=""),
            DeclareLaunchArgument("descriptor_batch_size", default_value="5"),
            DeclareLaunchArgument("descriptor_stride", default_value="1"),
            DeclareLaunchArgument(
                "verification_frame_batch_size", default_value="50"
            ),
            DeclareLaunchArgument("flush_period_s", default_value="1.0"),
            DeclareLaunchArgument("loop_closure.alpha", default_value="0.7"),
            DeclareLaunchArgument(
                "loop_closure.adaptive_scoring_tau_max",
                default_value="0.7",
            ),
            DeclareLaunchArgument(
                "loop_closure.adaptive_scoring_tau_min",
                default_value="0.7",
            ),
            DeclareLaunchArgument(
                "loop_closure.adaptive_scoring_lambda",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "aerial_05_bag_path", default_value="/data/graco/aerial-05-40m"
            ),
            DeclareLaunchArgument(
                "aerial_07_bag_path",
                default_value="/data/graco/aerial-07-25m_ros2",
            ),
            DeclareLaunchArgument("bag_rate", default_value="1.0"),
            DeclareLaunchArgument("record_output", default_value="true"),
            DeclareLaunchArgument(
                "output_bag_path", default_value=_timestamped_output_bag_path()
            ),
            DeclareLaunchArgument(
                "log_output_path", default_value="/tmp/sb_slam_ros2_logs"
            ),
            DeclareLaunchArgument(
                "rerun_application_id",
                default_value="graco_aerial_05_07_multi_robot",
            ),
            DeclareLaunchArgument(
                "rerun_recording_id",
                default_value=_timestamped_recording_id(),
                description="Shared timestamped Rerun recording id.",
            ),
            DeclareLaunchArgument(
                "rerun_host",
                default_value="rerun+http://127.0.0.1:9876/proxy",
            ),
            DeclareLaunchArgument("start_zenoh_router", default_value="false"),
            zenoh_router,
            recorder,
            OpaqueFunction(function=_launch_setup),
        ]
    )
