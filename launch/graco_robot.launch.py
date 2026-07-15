"""Launch one namespaced GrAco robot: Distributed, CBS, then delayed VIO."""

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
    return f"graco_aerial_05_{timestamp}"


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

    names_path = Path(
        LaunchConfiguration("robot_names_file").perform(context)
    )
    if not names_path.is_file():
        raise RuntimeError("robot_names_file is required for a multi-robot launch")
    with names_path.open("r", encoding="utf-8") as stream:
        names = yaml.safe_load(stream) or {}
    expected_name = names.get(f"robot{robot_id}_name")
    if expected_name != robot_name:
        raise RuntimeError(
            f"robot_name '{robot_name}' does not match robot_names_file "
            f"entry '{expected_name}' for robot {robot_id}"
        )
    for index in range(num_robots):
        if not names.get(f"robot{index}_name"):
            raise RuntimeError(
                f"robot_names_file is missing robot{index}_name"
            )

    models = {
        name: _require_file(context, name)
        for name in (
            "models.xfeat",
            "models.xfeat_interp_bilinear",
            "models.xfeat_interp_bicubic",
            "models.xfeat_interp_nearest",
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
        raise RuntimeError("Rerun application id, recording id, and host are required")

    image_topic = LaunchConfiguration("image_topic").perform(context)
    if not image_topic:
        image_topic = f"/{robot_name}/cam0/image_raw"
    imu_topic = LaunchConfiguration("imu_topic").perform(context)
    if not imu_topic:
        imu_topic = f"/{robot_name}/imu0"
    play_bag = LaunchConfiguration("play_bag").perform(context)
    bag_publish_clock = LaunchConfiguration(
        "bag_publish_clock"
    ).perform(context)
    bag_path = LaunchConfiguration("bag_path").perform(context)
    bag_rate = LaunchConfiguration("bag_rate").perform(context)
    bag_source_image_topic = LaunchConfiguration(
        "bag_source_image_topic"
    ).perform(context)
    bag_source_imu_topic = LaunchConfiguration(
        "bag_source_imu_topic"
    ).perform(context)

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
            "dataset_name": "GrAco",
            "frame_id": map_frame,
            "world_frame_id": world_frame,
            "odom_frame_id": odom_frame,
            "latest_kf_frame_id": latest_kf_frame,
            "vocab_path": vocabulary,
            "lightglue_model_path": models["models.lightglue_lcd"],
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
                    "kimera_vio_ros_mono.launch.py",
                ]
            )
        ),
        launch_arguments={
            "dataset_name": "GrAcoMonoXfeat",
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
            "multi_robot_bridge.verification_frame_batch_size": LaunchConfiguration(
                "verification_frame_batch_size"
            ),
            "multi_robot_bridge.publish_verification_frames": "true",
            "multi_robot_bridge.flush_period_s": LaunchConfiguration(
                "flush_period_s"
            ),
            "models.xfeat": models["models.xfeat"],
            "models.xfeat_interp_bilinear": models[
                "models.xfeat_interp_bilinear"
            ],
            "models.xfeat_interp_bicubic": models[
                "models.xfeat_interp_bicubic"
            ],
            "models.xfeat_interp_nearest": models[
                "models.xfeat_interp_nearest"
            ],
            "models.lightglue_frontend": models["models.lightglue_frontend"],
            "models.lightglue_lcd": models["models.lightglue_lcd"],
            "models.jist": models["models.jist"],
            "frame_id.base_link": base_frame,
            "frame_id.odom": odom_frame,
            "frame_id.map": map_frame,
            "frame_id.world": world_frame,
            "topic.image": image_topic,
            "topic.imu.data": imu_topic,
            "mono_depth.enabled": "false",
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_rerun_visualizer": "true",
            "rerun_application_id": rerun_application_id,
            "rerun_recording_id": rerun_recording_id,
            "rerun_host": rerun_host,
            "rosbag_play": play_bag,
            "rosbag_publish_clock": bag_publish_clock,
            "rosbag_path": bag_path,
            "rosbag_rate": bag_rate,
            "vio_start_delay": "0.0",
            "rosbag_source_image_topic": bag_source_image_topic,
            "rosbag_source_imu_topic": bag_source_imu_topic,
            "start_zenoh_router": "false",
        }.items(),
    )

    # Keep delayed substitutions isolated: the generic VIO launch starts its
    # node immediately, while its bag callback captures concrete topic names.
    delayed_vio_launch = TimerAction(period=1.0, actions=[vio_launch])
    return [distributed_launch, cbs_launch, delayed_vio_launch]


def generate_launch_description():
    zenoh_router = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("start_zenoh_router")),
        cmd=["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"],
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_id", default_value="0"),
            DeclareLaunchArgument("robot_name", default_value="a5"),
            DeclareLaunchArgument("num_robots", default_value="2"),
            DeclareLaunchArgument("robot_names_file", default_value=""),
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
            DeclareLaunchArgument("models.lightglue_frontend", default_value=""),
            DeclareLaunchArgument("models.lightglue_lcd", default_value=""),
            DeclareLaunchArgument("models.jist", default_value=""),
            DeclareLaunchArgument("vocabulary_path", default_value=""),
            DeclareLaunchArgument("world_frame", default_value="world"),
            DeclareLaunchArgument("image_topic", default_value=""),
            DeclareLaunchArgument("imu_topic", default_value=""),
            DeclareLaunchArgument("descriptor_batch_size", default_value="5"),
            DeclareLaunchArgument("descriptor_stride", default_value="1"),
            DeclareLaunchArgument(
                "verification_frame_batch_size", default_value="50"
            ),
            DeclareLaunchArgument("flush_period_s", default_value="1.0"),
            DeclareLaunchArgument("play_bag", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("bag_publish_clock", default_value="true"),
            DeclareLaunchArgument("bag_path", default_value=""),
            DeclareLaunchArgument("bag_rate", default_value="1.0"),
            DeclareLaunchArgument(
                "bag_source_image_topic", default_value="/cam0/image_raw"
            ),
            DeclareLaunchArgument(
                "bag_source_imu_topic", default_value="/imu0"
            ),
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
