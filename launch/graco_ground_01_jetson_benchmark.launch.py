"""Benchmark one GrAco ground VIO pipeline on a Jetson.

This profile isolates Kimera's stereo frontend, backend, and internal JIST
loop-closure detector. Dense mapping is fully disabled: no dense-mapping node
is launched, keyframe-state publication is off, and mono-depth is off.
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


_SOURCE_LEFT_IMAGE_TOPIC = "/camera_left/image_raw"
_SOURCE_RIGHT_IMAGE_TOPIC = "/camera_right/image_raw"
_SOURCE_IMU_TOPIC = "/gnss/imu"


def _workspace_root():
    return Path(
        os.environ.get(
            "ISAAC_ROS_WS",
            os.environ.get("SB_SLAM_ROS2_WS", "/workspaces/isaac_ros-dev"),
        )
    )


def _default_model_path(filename):
    return str(
        _workspace_root() / "models" / "jetpack_7.2_trt_10.16" / filename
    )


def _require_file(context, argument):
    value = LaunchConfiguration(argument).perform(context)
    if not value or not Path(value).is_file():
        raise RuntimeError(f"{argument} must name an existing model file")
    return value


def _launch_setup(context, *args, **kwargs):
    bag_path = Path(LaunchConfiguration("bag_path").perform(context))
    if not bag_path.is_dir() or not (bag_path / "metadata.yaml").is_file():
        raise RuntimeError(
            "bag_path must name a ROS 2 bag directory containing metadata.yaml"
        )

    models = {
        argument: _require_file(context, argument)
        for argument in (
            "models.xfeat",
            "models.lightglue_frontend",
            "models.lightglue_lcd",
            "models.jist",
        )
    }
    log_output_path = Path(
        LaunchConfiguration("log_output_path").perform(context)
    )
    log_output_path.mkdir(parents=True, exist_ok=True)

    vio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("kimera_vio_ros"),
                    "launch",
                    "kimera_vio_ros.launch.py",
                ]
            )
        ),
        launch_arguments={
            "dataset_name": "GrAcoGndStereoXfeat",
            "robot_id": "0",
            "robot_name": "g1",
            "robot_namespace": "g1",
            "use_lcd": "2",
            "multi_robot_bridge.enabled": "false",
            "publish_vlc_frames": "false",
            "models.xfeat": models["models.xfeat"],
            "models.lightglue_frontend": models["models.lightglue_frontend"],
            "models.lightglue_lcd": models["models.lightglue_lcd"],
            "models.jist": models["models.jist"],
            "frame_id.base_link": "g1/base_link",
            "frame_id.odom": "g1/odom",
            "frame_id.map": "g1/map",
            "frame_id.world": "world",
            "topic.left.image": "/g1/camera_left/image_raw",
            "topic.right.image": "/g1/camera_right/image_raw",
            "topic.imu.data": "/g1/gnss/imu",
            "use_camera_info": "false",
            "log_output": "true",
            "log_output_path": str(log_output_path),
            "dense_mapping.publisher_enabled": "false",
            "mono_depth.enabled": "false",
            "use_sim_time": "false",
            "visualize": "false",
            "use_rerun_visualizer": "false",
        }.items(),
    )

    bag_player = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            str(bag_path),
            "-r",
            LaunchConfiguration("bag_rate"),
            "--topics",
            _SOURCE_LEFT_IMAGE_TOPIC,
            _SOURCE_RIGHT_IMAGE_TOPIC,
            _SOURCE_IMU_TOPIC,
            "--remap",
            f"{_SOURCE_LEFT_IMAGE_TOPIC}:=/g1/camera_left/image_raw",
            f"{_SOURCE_RIGHT_IMAGE_TOPIC}:=/g1/camera_right/image_raw",
            f"{_SOURCE_IMU_TOPIC}:=/g1/gnss/imu",
        ],
        output="screen",
    )
    stop_after_bag = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_player,
            on_exit=EmitEvent(
                event=Shutdown(reason="GrAco ground-01 playback completed")
            ),
        )
    )

    return [
        vio_launch,
        stop_after_bag,
        TimerAction(
            period=LaunchConfiguration("bag_start_delay"),
            actions=[bag_player],
        ),
    ]


def generate_launch_description():
    zenoh_router = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("start_zenoh_router")),
        cmd=["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"],
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "bag_path", default_value="/data/graco/ground-01"
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
                    "lg_320x224_dyn_n1_o500_m1024_fp16.engine"
                ),
            ),
            DeclareLaunchArgument(
                "models.lightglue_lcd",
                default_value=_default_model_path(
                    "lg_320x224_dyn_n1_o500_m1024_fp16.engine"
                ),
            ),
            DeclareLaunchArgument(
                "models.jist",
                default_value=_default_model_path(
                    "JIST_r18_512_seqgem_simplified_fp16.engine"
                ),
            ),
            DeclareLaunchArgument(
                "log_output_path",
                default_value=str(
                    _workspace_root() / "benchmark" / "graco_ground_01"
                ),
            ),
            DeclareLaunchArgument(
                "start_zenoh_router", default_value="true"
            ),
            zenoh_router,
            OpaqueFunction(function=_launch_setup),
        ]
    )
