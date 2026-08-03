"""Run GrAco ground-01, ground-02, and ground-03 as g1/g2/g3.

This mirrors the ROS 1 ``code_slam_graco_g123.launch`` experiment with three
independent bag players, stereo GrAco ground VIO, Distributed, and Sim3 CBS.
Dense mapping and mono-depth processing are deliberately disabled.
"""

from datetime import datetime
import os
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


_EXPECTED_ROBOT_NAMES = ("g1", "g2", "g3")
_BAG_ARGUMENTS = (
    "ground_01_bag_path",
    "ground_02_bag_path",
    "ground_03_bag_path",
)
_SOURCE_LEFT_IMAGE_TOPIC = "/camera_left/image_raw"
_SOURCE_RIGHT_IMAGE_TOPIC = "/camera_right/image_raw"
_SOURCE_IMU_TOPIC = "/gnss/imu"


def _timestamped_recording_id():
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%z")
    return f"graco_ground_01_02_03_{timestamp}"


def _default_model_path(filename):
    workspace_root = Path(os.environ.get("SB_SLAM_ROS2_WS", os.getcwd()))
    return str(workspace_root / "src" / "xfeat-cpp" / "onnx_model" / filename)


def _bag_player(robot_name, bag_path, rate):
    target_left = f"/{robot_name}/camera_left/image_raw"
    target_right = f"/{robot_name}/camera_right/image_raw"
    target_imu = f"/{robot_name}/gnss/imu"
    return ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            bag_path,
            "-r",
            rate,
            "--topics",
            _SOURCE_LEFT_IMAGE_TOPIC,
            _SOURCE_RIGHT_IMAGE_TOPIC,
            _SOURCE_IMU_TOPIC,
            "--remap",
            f"{_SOURCE_LEFT_IMAGE_TOPIC}:={target_left}",
            f"{_SOURCE_RIGHT_IMAGE_TOPIC}:={target_right}",
            f"{_SOURCE_IMU_TOPIC}:={target_imu}",
        ],
        output="screen",
    )


def _launch_setup(context, *args, **kwargs):
    names_path = Path(LaunchConfiguration("robot_names_file").perform(context))
    if not names_path.is_file():
        raise RuntimeError("robot_names_file must name an existing YAML file")
    with names_path.open("r", encoding="utf-8") as stream:
        names = yaml.safe_load(stream)
    if not isinstance(names, dict):
        raise RuntimeError("robot_names_file must contain a mapping")
    for robot_id, expected_name in enumerate(_EXPECTED_ROBOT_NAMES):
        actual_name = names.get(f"robot{robot_id}_name")
        if actual_name != expected_name:
            raise RuntimeError(
                f"robot{robot_id}_name must be {expected_name}, got {actual_name}"
            )

    recording_id = LaunchConfiguration("rerun_recording_id").perform(context)
    if not recording_id:
        raise RuntimeError("rerun_recording_id must not be empty")

    bag_paths = tuple(
        LaunchConfiguration(argument).perform(context)
        for argument in _BAG_ARGUMENTS
    )
    for bag_path in bag_paths:
        path = Path(bag_path)
        if not path.is_dir() or not (path / "metadata.yaml").is_file():
            raise RuntimeError(
                f"ROS 2 bag directory does not contain metadata.yaml: {path}"
            )

    bag_start_delay = float(
        LaunchConfiguration("bag_start_delay").perform(context)
    )
    if bag_start_delay < 0.0:
        raise RuntimeError("bag_start_delay must be non-negative")

    robot_launch = PathJoinSubstitution(
        [
            FindPackageShare("sb_slam_ros2"),
            "launch",
            "graco_ground_robot.launch.py",
        ]
    )
    shared_arguments = {
        "num_robots": "3",
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
        "sim3_scale_sigma": LaunchConfiguration("sim3_scale_sigma"),
        "sim3_odom_scale_sigma": LaunchConfiguration(
            "sim3_odom_scale_sigma"
        ),
        "sim3_loop_scale_sigma": LaunchConfiguration(
            "sim3_loop_scale_sigma"
        ),
        "sim3_inter_loop_scale_sigma": LaunchConfiguration(
            "sim3_inter_loop_scale_sigma"
        ),
        "target_hellinger": LaunchConfiguration("target_hellinger"),
        "belief_stage_switch_strategy": LaunchConfiguration(
            "belief_stage_switch_strategy"
        ),
        "belief_stage_fixed_iterations": LaunchConfiguration(
            "belief_stage_fixed_iterations"
        ),
        "belief_republish_hellinger_threshold": LaunchConfiguration(
            "belief_republish_hellinger_threshold"
        ),
        "log_output_path": LaunchConfiguration("log_output_path"),
        "rerun_application_id": LaunchConfiguration("rerun_application_id"),
        "rerun_recording_id": recording_id,
        "rerun_host": LaunchConfiguration("rerun_host"),
        "start_zenoh_router": "false",
    }

    actions = []
    bag_players = []
    rate = LaunchConfiguration("bag_rate")
    for robot_id, (robot_name, bag_path) in enumerate(
        zip(_EXPECTED_ROBOT_NAMES, bag_paths)
    ):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_launch),
                launch_arguments={
                    **shared_arguments,
                    "robot_id": str(robot_id),
                    "robot_name": robot_name,
                    "left_image_topic": f"/{robot_name}/camera_left/image_raw",
                    "right_image_topic": (
                        f"/{robot_name}/camera_right/image_raw"
                    ),
                    "imu_topic": f"/{robot_name}/gnss/imu",
                }.items(),
            )
        )
        bag_players.append(_bag_player(robot_name, bag_path, rate))

    actions.append(TimerAction(period=bag_start_delay, actions=bag_players))
    return actions


def generate_launch_description():
    zenoh_router = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("start_zenoh_router")),
        cmd=["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"],
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_names_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("kimera_distributed"),
                        "params",
                        "robot_names_graco_gnd.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "ground_01_bag_path",
                default_value="/data/graco/ground-01",
            ),
            DeclareLaunchArgument(
                "ground_02_bag_path",
                default_value="/data/graco/ground-02",
            ),
            DeclareLaunchArgument(
                "ground_03_bag_path",
                default_value="/data/graco/ground-03_ros2",
            ),
            DeclareLaunchArgument("bag_rate", default_value="0.8"),
            DeclareLaunchArgument("bag_start_delay", default_value="20.0"),
            DeclareLaunchArgument(
                "models.xfeat",
                default_value=_default_model_path(
                    "xfeat_320x224_fp16.engine"
                ),
            ),
            DeclareLaunchArgument(
                "models.lightglue_frontend",
                default_value=_default_model_path(
                    "lg_320x224_dyn_min1_fp16.engine"
                ),
            ),
            DeclareLaunchArgument(
                "models.lightglue_lcd",
                default_value=_default_model_path(
                    "lg_320x224_dyn_min1_fp16.engine"
                ),
            ),
            DeclareLaunchArgument(
                "models.jist",
                default_value=_default_model_path(
                    "JIST_r18_512_seqgem_simplified_fp32.engine"
                ),
            ),
            DeclareLaunchArgument("vocabulary_path", default_value=""),
            DeclareLaunchArgument("descriptor_batch_size", default_value="5"),
            DeclareLaunchArgument("descriptor_stride", default_value="1"),
            DeclareLaunchArgument(
                "verification_frame_batch_size", default_value="50"
            ),
            DeclareLaunchArgument("flush_period_s", default_value="1.0"),
            DeclareLaunchArgument("loop_closure.alpha", default_value="0.7"),
            DeclareLaunchArgument(
                "loop_closure.adaptive_scoring_tau_max", default_value="0.8"
            ),
            DeclareLaunchArgument(
                "loop_closure.adaptive_scoring_tau_min", default_value="0.8"
            ),
            DeclareLaunchArgument(
                "loop_closure.adaptive_scoring_lambda", default_value="0.0"
            ),
            DeclareLaunchArgument("sim3_scale_sigma", default_value="0.1"),
            DeclareLaunchArgument(
                "sim3_odom_scale_sigma", default_value="-1"
            ),
            DeclareLaunchArgument(
                "sim3_loop_scale_sigma", default_value="-1"
            ),
            DeclareLaunchArgument(
                "sim3_inter_loop_scale_sigma", default_value="-1"
            ),
            DeclareLaunchArgument("target_hellinger", default_value="-1"),
            DeclareLaunchArgument(
                "belief_stage_switch_strategy", default_value="random"
            ),
            DeclareLaunchArgument(
                "belief_stage_fixed_iterations", default_value="10"
            ),
            DeclareLaunchArgument(
                "belief_republish_hellinger_threshold", default_value="0.01"
            ),
            DeclareLaunchArgument(
                "log_output_path",
                default_value="/tmp/sb_slam_ros2_logs/g123",
            ),
            DeclareLaunchArgument(
                "rerun_application_id",
                default_value="graco_ground_01_02_03_multi_robot",
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
            OpaqueFunction(function=_launch_setup),
        ]
    )
