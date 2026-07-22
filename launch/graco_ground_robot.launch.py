"""Launch one GrAco ground robot with stereo VIO, Distributed, and CBS.

Dense mapping is intentionally unavailable in this profile: neither a dense
mapping node nor DA3 is launched, and VIO keyframe-state publication is off.
"""

from datetime import datetime
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


def _timestamped_recording_id():
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%z")
    return f"graco_ground_01_02_03_{timestamp}"


def _require_file(context, argument):
    value = LaunchConfiguration(argument).perform(context)
    if not value or not Path(value).is_file():
        raise RuntimeError(f"{argument} must name an existing model file")
    return value


def _launch_setup(context, *args, **kwargs):
    robot_id = int(LaunchConfiguration("robot_id").perform(context))
    robot_name = LaunchConfiguration("robot_name").perform(context)
    num_robots = int(LaunchConfiguration("num_robots").perform(context))
    if robot_id < 0 or robot_id >= num_robots:
        raise RuntimeError("robot_id must be in [0, num_robots)")

    names_path = Path(LaunchConfiguration("robot_names_file").perform(context))
    if not names_path.is_file():
        raise RuntimeError("robot_names_file must name an existing YAML file")
    with names_path.open("r", encoding="utf-8") as stream:
        names = yaml.safe_load(stream) or {}
    if names.get(f"robot{robot_id}_name") != robot_name:
        raise RuntimeError(
            f"robot_name {robot_name!r} does not match robot{robot_id}_name"
        )
    for index in range(num_robots):
        if not names.get(f"robot{index}_name"):
            raise RuntimeError(
                f"robot_names_file is missing robot{index}_name"
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
    vocabulary = LaunchConfiguration("vocabulary_path").perform(context)
    if vocabulary and not Path(vocabulary).is_file():
        raise RuntimeError("vocabulary_path does not exist")

    log_output_path = LaunchConfiguration("log_output_path").perform(context)
    Path(log_output_path).mkdir(parents=True, exist_ok=True)
    rerun_application_id = LaunchConfiguration(
        "rerun_application_id"
    ).perform(context)
    rerun_recording_id = LaunchConfiguration("rerun_recording_id").perform(
        context
    )
    rerun_host = LaunchConfiguration("rerun_host").perform(context)
    if not rerun_application_id or not rerun_recording_id or not rerun_host:
        raise RuntimeError(
            "Rerun application id, recording id, and host are required"
        )

    left_image_topic = LaunchConfiguration("left_image_topic").perform(context)
    right_image_topic = LaunchConfiguration("right_image_topic").perform(
        context
    )
    imu_topic = LaunchConfiguration("imu_topic").perform(context)
    world_frame = LaunchConfiguration("world_frame").perform(context)
    base_frame = f"{robot_name}/base_link"
    odom_frame = f"{robot_name}/odom"
    map_frame = f"{robot_name}/map"
    latest_kf_frame = f"{robot_name}/latest_kf"

    distributed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("kimera_distributed"),
                    "launch",
                    "kimera_distributed_loop_closure_ros.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_id": str(robot_id),
            "robot_name": robot_name,
            "num_robots": str(num_robots),
            "robot_names_file": str(names_path),
            "dataset_name": "GrAcoGnd",
            "frame_id": map_frame,
            "world_frame_id": world_frame,
            "odom_frame_id": odom_frame,
            "latest_kf_frame_id": latest_kf_frame,
            "vocab_path": vocabulary,
            "lightglue_model_path": models["models.lightglue_lcd"],
            "alpha": LaunchConfiguration("loop_closure.alpha"),
            "sparse_bow_ids": "true",
            "adaptive_scoring_tau_max": LaunchConfiguration(
                "loop_closure.adaptive_scoring_tau_max"
            ),
            "adaptive_scoring_tau_min": LaunchConfiguration(
                "loop_closure.adaptive_scoring_tau_min"
            ),
            "adaptive_scoring_lambda": LaunchConfiguration(
                "loop_closure.adaptive_scoring_lambda"
            ),
            "log_output_path": log_output_path,
            "rerun_application_id": rerun_application_id,
            "rerun_recording_id": rerun_recording_id,
            "rerun_host": rerun_host,
            "rerun_enabled": "true",
        }.items(),
    )

    cbs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("cbs_ros"), "launch", "cbs_ros_node.launch.py"]
            )
        ),
        launch_arguments={
            "robot_id": str(robot_id),
            "robot_name": robot_name,
            "num_robots": str(num_robots),
            "robot_names_file": str(names_path),
            "pose_graph_topic": (
                f"/{robot_name}/kimera_distributed/pose_graph/updates"
            ),
            "log_dir": log_output_path,
            "online": "true",
            "pgo_formulation": "sim3",
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
            "rerun_application_id": rerun_application_id,
            "rerun_recording_id": rerun_recording_id,
            "rerun_host": rerun_host,
        }.items(),
    )

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
            "robot_id": str(robot_id),
            "robot_name": robot_name,
            "robot_namespace": robot_name,
            "use_lcd": "2",
            "multi_robot_bridge.enabled": "true",
            "multi_robot_bridge.descriptor_batch_size": LaunchConfiguration(
                "descriptor_batch_size"
            ),
            "multi_robot_bridge.descriptor_stride": LaunchConfiguration(
                "descriptor_stride"
            ),
            "multi_robot_bridge.verification_frame_batch_size": (
                LaunchConfiguration("verification_frame_batch_size")
            ),
            "multi_robot_bridge.publish_verification_frames": "true",
            "multi_robot_bridge.flush_period_s": LaunchConfiguration(
                "flush_period_s"
            ),
            "models.xfeat": models["models.xfeat"],
            "models.lightglue_frontend": models["models.lightglue_frontend"],
            "models.lightglue_lcd": models["models.lightglue_lcd"],
            "models.jist": models["models.jist"],
            "frame_id.base_link": base_frame,
            "frame_id.odom": odom_frame,
            "frame_id.map": map_frame,
            "frame_id.world": world_frame,
            "topic.left.image": left_image_topic,
            "topic.right.image": right_image_topic,
            "topic.imu.data": imu_topic,
            "use_camera_info": "false",
            "dense_mapping.publisher_enabled": "false",
            "mono_depth.enabled": "false",
            "use_sim_time": "false",
            "use_rerun_visualizer": "true",
            "rerun_application_id": rerun_application_id,
            "rerun_recording_id": rerun_recording_id,
            "rerun_host": rerun_host,
        }.items(),
    )

    return [
        distributed_launch,
        cbs_launch,
        TimerAction(period=1.0, actions=[vio_launch]),
    ]


def generate_launch_description():
    zenoh_router = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("start_zenoh_router")),
        cmd=["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"],
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_id", default_value="0"),
            DeclareLaunchArgument("robot_name", default_value="g1"),
            DeclareLaunchArgument("num_robots", default_value="3"),
            DeclareLaunchArgument("robot_names_file", default_value=""),
            DeclareLaunchArgument("models.xfeat", default_value=""),
            DeclareLaunchArgument("models.lightglue_frontend", default_value=""),
            DeclareLaunchArgument("models.lightglue_lcd", default_value=""),
            DeclareLaunchArgument("models.jist", default_value=""),
            DeclareLaunchArgument("vocabulary_path", default_value=""),
            DeclareLaunchArgument("world_frame", default_value="world"),
            DeclareLaunchArgument(
                "left_image_topic", default_value="/g1/camera_left/image_raw"
            ),
            DeclareLaunchArgument(
                "right_image_topic", default_value="/g1/camera_right/image_raw"
            ),
            DeclareLaunchArgument("imu_topic", default_value="/g1/gnss/imu"),
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
            DeclareLaunchArgument("sim3_scale_sigma", default_value="0.05"),
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
                "log_output_path", default_value="/tmp/sb_slam_ros2_logs/g123"
            ),
            DeclareLaunchArgument(
                "rerun_application_id",
                default_value="graco_ground_01_02_03_multi_robot",
            ),
            DeclareLaunchArgument(
                "rerun_recording_id",
                default_value=_timestamped_recording_id(),
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
