"""Run the six-robot Campus experiment under ROS 2.

The robot identities, topics, bag offsets, D455 profile, and loop-closure
settings mirror the ROS 1 ``code_slam_campus.launch`` experiment.  Converted
bags can contain either compressed or raw infrared images; select the layout
with ``bag_image_transport``.

As in the tested ROS 1 experiment, wheel odometry is fused by each robot's VIO
through the D455 ``ExternalOdometryParams.yaml`` profile.
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
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


_ROBOTS = (
    ("hathor", "hathor_bag_path", 65.0),
    ("sparkal1", "sparkal1_bag_path", 20.0),
    ("sparkal2", "sparkal2_bag_path", 20.0),
    ("thoth", "thoth_bag_path", 20.0),
    ("acl_jackal", "acl_jackal_bag_path", 20.0),
    ("acl_jackal2", "acl_jackal2_bag_path", 20.0),
)


def _as_bool(value):
    return str(value).lower() in ("1", "true", "yes", "on")


def _timestamped_recording_id():
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%z")
    return f"campus_six_robot_{timestamp}"


def _workspace_path(*parts):
    workspace_root = Path(os.environ.get("SB_SLAM_ROS2_WS", os.getcwd()))
    return str(workspace_root.joinpath(*parts))


def _default_model_path(filename):
    return _workspace_path("src", "xfeat-cpp", "onnx_model", filename)


def _image_topics(robot_name):
    base = f"/{robot_name}/forward"
    return (
        f"{base}/infra1/image_rect_raw",
        f"{base}/infra2/image_rect_raw",
    )


def _image_republisher(robot_name, camera_name, raw_topic):
    return Node(
        package="image_transport",
        executable="republish",
        name=f"{robot_name}_{camera_name}_decompressor",
        parameters=[
            {
                "in_transport": "compressed",
                "out_transport": "raw",
            }
        ],
        remappings=[
            ("in/compressed", f"{raw_topic}/compressed"),
            ("out", raw_topic),
        ],
        output="screen",
    )


def _bag_player(
    robot_name,
    bag_path,
    rate,
    start_offset,
    playback_duration,
    image_transport,
    vio_mode,
):
    left_topic, right_topic = _image_topics(robot_name)
    if image_transport == "compressed":
        left_topic += "/compressed"
        right_topic += "/compressed"
    image_topics = [left_topic]
    if vio_mode == "stereo":
        image_topics.append(right_topic)

    command = [
        "ros2",
        "bag",
        "play",
        bag_path,
        "--rate",
        str(rate),
        "--start-offset",
        str(start_offset),
        "--disable-keyboard-controls",
    ]
    if playback_duration >= 0.0:
        command.extend(
            [
                "--playback-duration",
                str(playback_duration),
            ]
        )
    command.extend(
        [
            "--topics",
            *image_topics,
            f"/{robot_name}/forward/imu",
            f"/{robot_name}/jackal_velocity_controller/odom",
        ]
    )
    return ExecuteProcess(cmd=command, output="screen")


def _require_file(context, argument):
    value = LaunchConfiguration(argument).perform(context)
    if not value or not Path(value).is_file():
        raise RuntimeError(f"{argument} must name an existing model file")
    return value


def _launch_setup(context, *args, **kwargs):
    num_robots = len(_ROBOTS)
    names_path = Path(LaunchConfiguration("robot_names_file").perform(context))
    if not names_path.is_file():
        raise RuntimeError("robot_names_file must name an existing YAML file")
    with names_path.open("r", encoding="utf-8") as stream:
        names = yaml.safe_load(stream)
    if not isinstance(names, dict):
        raise RuntimeError("robot_names_file must contain a mapping")
    for robot_id, (robot_name, _, _) in enumerate(_ROBOTS):
        actual_name = names.get(f"robot{robot_id}_name")
        if actual_name != robot_name:
            raise RuntimeError(
                f"robot{robot_id}_name must be {robot_name}, got {actual_name}"
            )

    models = {
        argument: _require_file(context, argument)
        for argument in (
            "models.xfeat",
            "models.lightglue_frontend",
            "models.lightglue_lcd",
        )
    }
    vpr_model_type = LaunchConfiguration(
        "vpr_model_type"
    ).perform(context).strip().lower()
    if vpr_model_type not in ("jist", "mixvpr"):
        raise RuntimeError("vpr_model_type must be jist or mixvpr")
    models["models.jist"] = ""
    models["models.mixvpr"] = ""
    selected_vpr_argument = f"models.{vpr_model_type}"
    models[selected_vpr_argument] = _require_file(
        context, selected_vpr_argument
    )
    vocabulary = LaunchConfiguration("vocabulary_path").perform(context)
    if vocabulary and not Path(vocabulary).is_file():
        raise RuntimeError("vocabulary_path does not exist")

    vio_mode = LaunchConfiguration("vio_mode").perform(context).strip().lower()
    if vio_mode not in ("stereo", "mono"):
        raise RuntimeError("vio_mode must be stereo or mono")
    vio_params_argument = (
        "vio_params_folder"
        if vio_mode == "stereo"
        else "vio_mono_params_folder"
    )
    vio_params_folder = Path(
        LaunchConfiguration(vio_params_argument).perform(context)
    )
    required_vio_params = (
        "BackendParams.yaml",
        "DisplayParams.yaml",
        "ExternalOdometryParams.yaml",
        "FrontendParams.yaml",
        "ImuParams.yaml",
        "LcdParams.yaml",
        "LeftCameraParams.yaml",
        "PipelineParams.yaml",
        "RightCameraParams.yaml",
        "flags/Mesher.flags",
        "flags/RegularVioBackend.flags",
        "flags/VioBackend.flags",
        "flags/Visualizer3D.flags",
    )
    missing_vio_params = [
        relative
        for relative in required_vio_params
        if not (vio_params_folder / relative).is_file()
    ]
    if missing_vio_params:
        raise RuntimeError(
            f"D455 VIO profile is incomplete at {vio_params_folder}: "
            f"{missing_vio_params}"
        )

    play_bags = _as_bool(LaunchConfiguration("play_bags").perform(context))
    bag_image_transport = LaunchConfiguration("bag_image_transport").perform(
        context
    )
    if bag_image_transport not in ("compressed", "raw"):
        raise RuntimeError("bag_image_transport must be compressed or raw")
    bag_rate = float(LaunchConfiguration("bag_rate").perform(context))
    if bag_rate <= 0.0:
        raise RuntimeError("bag_rate must be positive")
    bag_start_delay = float(
        LaunchConfiguration("bag_start_delay").perform(context)
    )
    if bag_start_delay < 0.0:
        raise RuntimeError("bag_start_delay must be non-negative")
    playback_duration = float(
        LaunchConfiguration("bag_playback_duration").perform(context)
    )

    bag_paths = {}
    if play_bags:
        for robot_name, bag_argument, _ in _ROBOTS:
            bag_path = Path(LaunchConfiguration(bag_argument).perform(context))
            if not bag_path.is_dir() or not (bag_path / "metadata.yaml").is_file():
                raise RuntimeError(
                    "ROS 2 bag directory does not contain metadata.yaml: "
                    f"{bag_path}"
                )
            bag_paths[robot_name] = str(bag_path)

    log_root = Path(LaunchConfiguration("log_output_path").perform(context))
    log_root.mkdir(parents=True, exist_ok=True)
    recording_id = LaunchConfiguration("rerun_recording_id").perform(context)
    application_id = LaunchConfiguration("rerun_application_id").perform(context)
    rerun_host = LaunchConfiguration("rerun_host").perform(context)
    if not recording_id or not application_id or not rerun_host:
        raise RuntimeError(
            "Rerun application id, recording id, and host are required"
        )
    visualization_mode = LaunchConfiguration("visualization_mode").perform(
        context
    ).strip().lower()
    if visualization_mode not in ("full", "minimal"):
        raise RuntimeError("visualization_mode must be full or minimal")
    vio_rerun_visualization_profile = visualization_mode

    distributed_launch = PathJoinSubstitution(
        [
            FindPackageShare("kimera_distributed"),
            "launch",
            "kimera_distributed_loop_closure_ros.launch.py",
        ]
    )
    cbs_launch = PathJoinSubstitution(
        [FindPackageShare("cbs_ros"), "launch", "cbs_ros_node.launch.py"]
    )
    vio_launch = PathJoinSubstitution(
        [
            FindPackageShare("kimera_vio_ros"),
            "launch",
            (
                "kimera_vio_ros.launch.py"
                if vio_mode == "stereo"
                else "kimera_vio_ros_mono.launch.py"
            ),
        ]
    )

    actions = []
    bag_players = []
    rerun_enabled = LaunchConfiguration("rerun_enabled")
    for robot_id, (robot_name, _, start_offset) in enumerate(_ROBOTS):
        robot_log_dir = str(log_root / robot_name)
        for component in ("distributed", "cbs", "vio"):
            (Path(robot_log_dir) / component).mkdir(parents=True, exist_ok=True)
        left_topic, right_topic = _image_topics(robot_name)
        base_frame = f"{robot_name}/base_link"
        odom_frame = f"{robot_name}/odom"
        map_frame = f"{robot_name}/map"
        latest_kf_frame = f"{robot_name}/latest_kf"

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(distributed_launch),
                launch_arguments={
                    "robot_id": str(robot_id),
                    "robot_name": robot_name,
                    "num_robots": str(num_robots),
                    "robot_names_file": str(names_path),
                    "dataset_name": "Campus",
                    "frame_id": map_frame,
                    "world_frame_id": LaunchConfiguration("world_frame"),
                    "odom_frame_id": odom_frame,
                    "latest_kf_frame_id": latest_kf_frame,
                    "vocab_path": vocabulary,
                    "lightglue_model_path": models["models.lightglue_lcd"],
                    "alpha": LaunchConfiguration("loop_closure.alpha"),
                    "min_sim_vlad": LaunchConfiguration(
                        "loop_closure.min_sim_vlad"
                    ),
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
                    "verified_loop_rotation_sigma_rad": LaunchConfiguration(
                        "loop_closure.rotation_sigma_rad"
                    ),
                    "verified_loop_translation_sigma_m": LaunchConfiguration(
                        "loop_closure.translation_sigma_m"
                    ),
                    "max_submap_size": LaunchConfiguration(
                        "loop_closure.max_submap_size"
                    ),
                    "max_submap_distance": LaunchConfiguration(
                        "loop_closure.max_submap_distance"
                    ),
                    "log_output_path": f"{robot_log_dir}/distributed",
                    "rerun_enabled": rerun_enabled,
                    "rerun_application_id": application_id,
                    "rerun_recording_id": recording_id,
                    "rerun_host": rerun_host,
                }.items(),
            )
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(cbs_launch),
                launch_arguments={
                    "robot_id": str(robot_id),
                    "robot_name": robot_name,
                    "num_robots": str(num_robots),
                    "robot_names_file": str(names_path),
                    "pose_graph_topic": (
                        f"/{robot_name}/kimera_distributed/pose_graph/updates"
                    ),
                    "log_dir": f"{robot_log_dir}/cbs",
                    "online": "true",
                    "pgo_formulation": LaunchConfiguration("pgo_formulation"),
                    "sim3_scale_sigma": LaunchConfiguration(
                        "sim3_scale_sigma"
                    ),
                    "sim3_odom_scale_sigma": LaunchConfiguration(
                        "sim3_odom_scale_sigma"
                    ),
                    "sim3_loop_scale_sigma": LaunchConfiguration(
                        "sim3_loop_scale_sigma"
                    ),
                    "sim3_inter_loop_scale_sigma": LaunchConfiguration(
                        "sim3_inter_loop_scale_sigma"
                    ),
                    "sim3_inter_loop_has_scale_measurement": LaunchConfiguration(
                        "sim3_inter_loop_has_scale_measurement"
                    ),
                    "sim3_pose_scale_prior_sigma": LaunchConfiguration(
                        "sim3_pose_scale_prior_sigma"
                    ),
                    "sim3_anchor_scale_prior_sigma": LaunchConfiguration(
                        "sim3_anchor_scale_prior_sigma"
                    ),
                    "communication_topology_mode": "dynamic_factors",
                    "belief_stage_switch_strategy": LaunchConfiguration(
                        "belief_stage_switch_strategy"
                    ),
                    "belief_stage_fixed_iterations": LaunchConfiguration(
                        "belief_stage_fixed_iterations"
                    ),
                    "belief_republish_hellinger_threshold": LaunchConfiguration(
                        "belief_republish_hellinger_threshold"
                    ),
                    "rerun_application_id": application_id,
                    "rerun_recording_id": recording_id,
                    "rerun_host": rerun_host,
                }.items(),
            )
        )

        if bag_image_transport == "compressed":
            actions.append(
                _image_republisher(robot_name, "infra1", left_topic)
            )
            if vio_mode == "stereo":
                actions.append(
                    _image_republisher(robot_name, "infra2", right_topic)
                )

        vio = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vio_launch),
            launch_arguments={
                "dataset_name": "D455",
                "params_folder": str(vio_params_folder),
                "robot_id": str(robot_id),
                "robot_name": robot_name,
                "robot_namespace": robot_name,
                "use_lcd": "2",
                "multi_robot_bridge.enabled": "true",
                "multi_robot_bridge.descriptor_batch_size": (
                    LaunchConfiguration("descriptor_batch_size")
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
                "models.lightglue_frontend": models[
                    "models.lightglue_frontend"
                ],
                "models.lightglue_lcd": models["models.lightglue_lcd"],
                "models.jist": models["models.jist"],
                "models.mixvpr": models["models.mixvpr"],
                "frame_id.base_link": base_frame,
                "frame_id.odom": odom_frame,
                "frame_id.map": map_frame,
                "frame_id.world": LaunchConfiguration("world_frame"),
                "topic.image": left_topic,
                "topic.left.image": left_topic,
                "topic.right.image": right_topic,
                "topic.imu.data": f"/{robot_name}/forward/imu",
                "use_external_odom": LaunchConfiguration(
                    "use_external_odom"
                ),
                "topic.external_odom": (
                    f"/{robot_name}/jackal_velocity_controller/odom"
                ),
                "use_camera_info": "false",
                "log_output": "true",
                "log_output_path": f"{robot_log_dir}/vio",
                "dense_mapping.publisher_enabled": "false",
                "mono_depth.enabled": "false",
                "use_sim_time": "false",
                "use_rerun_visualizer": rerun_enabled,
                "rerun_application_id": application_id,
                "rerun_recording_id": recording_id,
                "rerun_host": rerun_host,
                "rerun_visualization_profile": (
                    vio_rerun_visualization_profile
                ),
                "rerun_tracking_image_jpeg_quality": LaunchConfiguration(
                    "rerun_tracking_image_jpeg_quality"
                ),
                "start_zenoh_router": "false",
                "vio_start_delay": "0.0",
            }.items(),
        )
        actions.append(TimerAction(period=1.0, actions=[vio]))

        if play_bags:
            bag_players.append(
                _bag_player(
                    robot_name,
                    bag_paths[robot_name],
                    bag_rate,
                    start_offset,
                    playback_duration,
                    bag_image_transport,
                    vio_mode,
                )
            )

    if bag_players:
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
                        "robot_names_campus.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "hathor_bag_path",
                default_value="/data3/campus/ros2/10_14_hathor",
            ),
            DeclareLaunchArgument(
                "sparkal1_bag_path",
                default_value="/data3/campus/ros2/10_14_sparkal1",
            ),
            DeclareLaunchArgument(
                "sparkal2_bag_path",
                default_value="/data3/campus/ros2/10_14_sparkal2",
            ),
            DeclareLaunchArgument(
                "thoth_bag_path",
                default_value="/data3/campus/ros2/10_14_thoth",
            ),
            DeclareLaunchArgument(
                "acl_jackal_bag_path",
                default_value="/data3/campus/ros2/10_14_acl_jackal",
            ),
            DeclareLaunchArgument(
                "acl_jackal2_bag_path",
                default_value="/data3/campus/ros2/10_14_acl_jackal2",
            ),
            DeclareLaunchArgument("play_bags", default_value="true"),
            DeclareLaunchArgument(
                "bag_image_transport",
                default_value="compressed",
                description="Converted bag image layout: compressed or raw.",
            ),
            DeclareLaunchArgument("bag_rate", default_value="1.0"),
            DeclareLaunchArgument("bag_start_delay", default_value="20.0"),
            DeclareLaunchArgument(
                "bag_playback_duration",
                default_value="-1",
                description="Playback seconds after each offset; negative means all.",
            ),
            DeclareLaunchArgument(
                "models.xfeat",
                default_value=_default_model_path("xfeat_320x224_fp16.engine"),
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
            DeclareLaunchArgument(
                "models.mixvpr",
                default_value=_default_model_path(
                    "trt/mixvpr_resnet50_512d_fp16_sm120_trt10.13.engine"
                ),
            ),
            DeclareLaunchArgument(
                "vpr_model_type",
                default_value="jist",
                description=(
                    "VPR backend selected by the chosen VIO parameter profile."
                ),
            ),
            DeclareLaunchArgument("vocabulary_path", default_value=""),
            DeclareLaunchArgument(
                "vio_params_folder",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("kimera_vio_ros"),
                        "param",
                        "D455",
                    ]
                ),
                description="D455 profile installed by kimera_vio_ros.",
            ),
            DeclareLaunchArgument(
                "vio_mono_params_folder",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("kimera_vio_ros"),
                        "param",
                        "D455_Mono",
                    ]
                ),
                description=(
                    "D455 monocular profile installed by kimera_vio_ros."
                ),
            ),
            DeclareLaunchArgument(
                "vio_mode",
                default_value="stereo",
                description="VIO frontend mode: stereo or mono.",
            ),
            DeclareLaunchArgument(
                "use_external_odom",
                default_value="true",
                description="Fuse the bag's wheel-odometry measurements in VIO.",
            ),
            DeclareLaunchArgument("world_frame", default_value="world"),
            DeclareLaunchArgument("descriptor_batch_size", default_value="5"),
            DeclareLaunchArgument("descriptor_stride", default_value="1"),
            DeclareLaunchArgument(
                "verification_frame_batch_size", default_value="50"
            ),
            DeclareLaunchArgument("flush_period_s", default_value="1.0"),
            DeclareLaunchArgument("loop_closure.alpha", default_value="0.7"),
            DeclareLaunchArgument(
                "loop_closure.min_sim_vlad",
                default_value="0.8",
                description="Global-descriptor similarity candidate threshold.",
            ),
            DeclareLaunchArgument(
                "loop_closure.adaptive_scoring_tau_max", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "loop_closure.adaptive_scoring_tau_min", default_value="0.0"
            ),
            DeclareLaunchArgument(
                "loop_closure.adaptive_scoring_lambda", default_value="0.1"
            ),
            DeclareLaunchArgument(
                "loop_closure.rotation_sigma_rad", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "loop_closure.translation_sigma_m", default_value="10.0"
            ),
            DeclareLaunchArgument(
                "loop_closure.max_submap_size", default_value="10"
            ),
            DeclareLaunchArgument(
                "loop_closure.max_submap_distance", default_value="5.0"
            ),
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
                "pgo_formulation",
                default_value="sim3",
                description=(
                    "CBS optimization formulation. Use pose3 only when "
                    "explicitly requested."
                ),
            ),
            DeclareLaunchArgument(
                "sim3_scale_sigma",
                default_value="1e-6",
                description=(
                    "Campus fallback scale sigma for lifted Sim3 factors."
                ),
            ),
            DeclareLaunchArgument(
                "sim3_odom_scale_sigma",
                default_value="1e-6",
                description=(
                    "Near-fixed odometry scale because Campus has external "
                    "odometry."
                ),
            ),
            DeclareLaunchArgument(
                "sim3_loop_scale_sigma",
                default_value="1e-6",
                description="Near-fixed intra-robot loop scale.",
            ),
            DeclareLaunchArgument(
                "sim3_inter_loop_scale_sigma",
                default_value="1e-6",
                description="Near-fixed inter-robot loop scale.",
            ),
            DeclareLaunchArgument(
                "sim3_inter_loop_has_scale_measurement",
                default_value="false",
                description=(
                    "Whether inter-robot measurements explicitly observe "
                    "relative scale. Campus loop measurements do not."
                ),
            ),
            DeclareLaunchArgument(
                "sim3_pose_scale_prior_sigma",
                default_value="-1",
                description="Negative disables absolute local-pose scale priors.",
            ),
            DeclareLaunchArgument(
                "sim3_anchor_scale_prior_sigma",
                default_value="-1",
                description="Negative disables absolute robot-anchor scale priors.",
            ),
            DeclareLaunchArgument(
                "log_output_path",
                default_value="/tmp/sb_slam_ros2_logs/campus_six_robot",
            ),
            DeclareLaunchArgument(
                "rerun_enabled",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "visualization_mode",
                default_value="minimal",
                description=(
                    "minimal keeps distributed/CBS maps plus compressed VIO "
                    "tracking images and a throttled raw VIO trajectory; full "
                    "enables all VIO Rerun entities"
                ),
            ),
            DeclareLaunchArgument(
                "rerun_tracking_image_jpeg_quality",
                default_value="80",
            ),
            DeclareLaunchArgument(
                "rerun_application_id",
                default_value="campus_six_robot",
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
