"""Run one GrAco aerial robot using the reusable aerial robot stack.

This launch owns single-experiment concerns (one bag player, one optional
Zenoh router, logging metadata, and shutdown timing) and includes
``graco_aerial_robot.launch.py`` exactly once.  The A5678 multi-robot launch
includes the same primitive once per selected robot.

Defaults reproduce the established A5 stereo/MixVPR/OpenGV experiment.  Set
``num_robots:=1`` for an isolated A5 graph, or retain ``num_robots:=4`` when
running one partition of an A5/A6/A7/A8 distributed experiment.
"""

import os
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
    TimerAction,
)
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

_SOURCE_IMAGE_TOPIC = "/camera_left/image_raw"
_SOURCE_RIGHT_IMAGE_TOPIC = "/camera_right/image_raw"
_SOURCE_IMU_TOPIC = "/gnss/imu"


def _timestamped_recording_id():
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%z")
    return f"graco_aerial_single_robot_{timestamp}"


def _default_model_path(filename):
    workspace_root = Path(os.environ.get("SB_SLAM_ROS2_WS", os.getcwd()))
    return str(workspace_root / "src" / "xfeat-cpp" / "onnx_model" / filename)


def _default_mixvpr_model_path():
    workspace_root = Path(os.environ.get("SB_SLAM_ROS2_WS", os.getcwd()))
    model_root = workspace_root / "src" / "xfeat-cpp" / "onnx_model"
    candidates = (
        model_root / "trt" / "mixvpr_resnet50_512d_fp16_sm120_trt10.13.engine",
        model_root / "mixvpr_resnet50_512d_fp16.engine",
    )
    for model_path in candidates:
        if model_path.is_file():
            return str(model_path)
    return str(candidates[0])


def _as_bool(value):
    normalized = str(value).strip().lower()
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    raise RuntimeError(f"expected a boolean value, got {value!r}")


def _bag_topic_names(metadata_path):
    with metadata_path.open("r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream) or {}
    bag_info = metadata.get("rosbag2_bagfile_information", metadata)
    topics = bag_info.get("topics_with_message_count", [])
    return {
        item.get("topic_metadata", {}).get("name")
        for item in topics
        if isinstance(item, dict)
    }


def _resolved(context, name):
    return LaunchConfiguration(name).perform(context)


def _prepare_single_run(context, *args, **kwargs):
    robot_id = int(_resolved(context, "robot_id"))
    robot_name = _resolved(context, "robot_name")
    num_robots = int(_resolved(context, "num_robots"))
    if robot_id < 0 or robot_id >= num_robots:
        raise RuntimeError("robot_id must be in [0, num_robots)")

    names_path = Path(_resolved(context, "robot_names_file"))
    if not names_path.is_file():
        raise RuntimeError("robot_names_file must name an existing YAML file")
    with names_path.open("r", encoding="utf-8") as stream:
        names = yaml.safe_load(stream) or {}
    expected_name = names.get(f"robot{robot_id}_name")
    if expected_name != robot_name:
        raise RuntimeError(
            f"robot_name {robot_name!r} does not match robot{robot_id}_name "
            f"{expected_name!r} in robot_names_file"
        )

    vio_mode = _resolved(context, "vio_mode").strip().lower()
    if vio_mode not in ("mono", "stereo"):
        raise RuntimeError("vio_mode must be mono or stereo")
    bag_path = Path(_resolved(context, "bag_path"))
    if _as_bool(_resolved(context, "play_bag")):
        metadata_path = bag_path / "metadata.yaml"
        if not bag_path.is_dir() or not metadata_path.is_file():
            raise RuntimeError(
                f"ROS 2 bag directory does not contain metadata.yaml: {bag_path}"
            )
        required_topics = {_SOURCE_IMAGE_TOPIC, _SOURCE_IMU_TOPIC}
        if vio_mode == "stereo":
            required_topics.add(_SOURCE_RIGHT_IMAGE_TOPIC)
        missing_topics = required_topics - _bag_topic_names(metadata_path)
        if missing_topics:
            raise RuntimeError(
                f"ROS 2 bag {bag_path} is missing required {vio_mode} "
                f"topics: {sorted(missing_topics)}"
            )

    start_delay = float(_resolved(context, "bag_start_delay"))
    playback_duration = float(_resolved(context, "bag_playback_duration"))
    if start_delay < 0.0:
        raise RuntimeError("bag_start_delay must be non-negative")
    if playback_duration < -1.0:
        raise RuntimeError("bag_playback_duration must be -1 or non-negative")

    log_root = Path(_resolved(context, "log_output_path"))
    log_root.mkdir(parents=True, exist_ok=True)
    manifest_arguments = (
        "robot_id",
        "robot_name",
        "num_robots",
        "vio_mode",
        "vio_dataset_name",
        "distributed_dataset_name",
        "vpr_model_type",
        "jist_frame_refinement",
        "play_bag",
        "bag_path",
        "bag_rate",
        "bag_start_delay",
        "bag_playback_duration",
        "stereo_depth.method",
        "loop_closure.alpha",
        "loop_closure.min_sim_vlad",
        "loop_closure.stereo_verification_method",
        "pgo_formulation",
        "sim3_pose_scale_prior_sigma",
        "visualization_mode",
        "rerun_application_id",
        "rerun_recording_id",
        "rerun_host",
    )
    manifest = {
        "launch_file": "graco_aerial_single_robot.launch.py",
        "robot_names_file": str(names_path),
        "resolved_launch_arguments": {
            name: _resolved(context, name) for name in manifest_arguments
        },
    }
    with (log_root / "experiment_setup.yaml").open("w", encoding="utf-8") as stream:
        yaml.safe_dump(manifest, stream, sort_keys=False)
    return []


def _schedule_bag_player(context, *args, **kwargs):
    if not _as_bool(_resolved(context, "play_bag")):
        return []

    start_delay = float(_resolved(context, "bag_start_delay"))
    robot_name = _resolved(context, "robot_name")
    vio_mode = _resolved(context, "vio_mode").strip().lower()
    command = [
        "ros2",
        "bag",
        "play",
        _resolved(context, "bag_path"),
        "--rate",
        _resolved(context, "bag_rate"),
        "--read-ahead-queue-size",
        "1000",
        "--disable-keyboard-controls",
    ]
    source_topics = [_SOURCE_IMAGE_TOPIC]
    remappings = [f"{_SOURCE_IMAGE_TOPIC}:=/{robot_name}/camera_left/image_raw"]
    if vio_mode == "stereo":
        source_topics.append(_SOURCE_RIGHT_IMAGE_TOPIC)
        remappings.append(
            f"{_SOURCE_RIGHT_IMAGE_TOPIC}:=/{robot_name}/camera_right/image_raw"
        )
    source_topics.append(_SOURCE_IMU_TOPIC)
    remappings.append(f"{_SOURCE_IMU_TOPIC}:=/{robot_name}/gnss/imu")
    command.extend(["--topics", *source_topics, "--remap", *remappings])
    return [
        TimerAction(
            period=start_delay,
            actions=[ExecuteProcess(cmd=command, output="screen")],
        )
    ]


def _shutdown_timer(context, *args, **kwargs):
    if not _as_bool(_resolved(context, "play_bag")):
        return []
    playback_duration = float(_resolved(context, "bag_playback_duration"))
    if playback_duration < 0.0:
        return []
    start_delay = float(_resolved(context, "bag_start_delay"))
    return [
        TimerAction(
            period=start_delay + playback_duration + 5.0,
            actions=[
                EmitEvent(
                    event=Shutdown(reason="GrAco aerial single-robot replay completed")
                )
            ],
        )
    ]


def generate_launch_description():
    robot_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("sb_slam_ros2"),
                    "launch",
                    "graco_aerial_robot.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_id": LaunchConfiguration("robot_id"),
            "robot_name": LaunchConfiguration("robot_name"),
            "num_robots": LaunchConfiguration("num_robots"),
            "robot_names_file": LaunchConfiguration("robot_names_file"),
            "image_topic": LaunchConfiguration("image_topic"),
            "right_image_topic": LaunchConfiguration("right_image_topic"),
            "imu_topic": LaunchConfiguration("imu_topic"),
            "log_output_path": LaunchConfiguration("log_output_path"),
        }.items(),
    )

    zenoh_router = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("start_zenoh_router")),
        cmd=["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_id", default_value="0"),
            DeclareLaunchArgument("robot_name", default_value="a5"),
            DeclareLaunchArgument("num_robots", default_value="4"),
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
            DeclareLaunchArgument(
                "image_topic",
                default_value=[
                    "/",
                    LaunchConfiguration("robot_name"),
                    "/camera_left/image_raw",
                ],
            ),
            DeclareLaunchArgument(
                "right_image_topic",
                default_value=[
                    "/",
                    LaunchConfiguration("robot_name"),
                    "/camera_right/image_raw",
                ],
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value=[
                    "/",
                    LaunchConfiguration("robot_name"),
                    "/gnss/imu",
                ],
            ),
            DeclareLaunchArgument("vio_mode", default_value="stereo"),
            DeclareLaunchArgument("vio_dataset_name", default_value="GrAcoStereoXfeat"),
            DeclareLaunchArgument(
                "distributed_dataset_name", default_value="GrAcoStereo"
            ),
            # Compatibility-only experiment metadata. LcdParams.yaml selects
            # the active VPR backend.
            DeclareLaunchArgument("vpr_model_type", default_value=""),
            DeclareLaunchArgument("jist_frame_refinement", default_value="false"),
            DeclareLaunchArgument(
                "models.xfeat",
                default_value=_default_model_path("xfeat_320x224_fp16.engine"),
            ),
            DeclareLaunchArgument(
                "models.lightglue_frontend",
                default_value=_default_model_path("lg_320x224_dyn_min1_fp16.engine"),
            ),
            DeclareLaunchArgument(
                "models.lightglue_lcd",
                default_value=_default_model_path("lg_320x224_dyn_min1_fp16.engine"),
            ),
            DeclareLaunchArgument(
                "models.jist",
                default_value=_default_model_path(
                    "JIST_r18_512_seqgem_frames_fp32.engine"
                ),
            ),
            DeclareLaunchArgument(
                "models.mixvpr",
                default_value=_default_mixvpr_model_path(),
            ),
            DeclareLaunchArgument(
                "stereo_depth.method", default_value="VPI_CUDA"
            ),
            DeclareLaunchArgument(
                "models.stereo_depth",
                default_value=_default_model_path("fast_foundation_stereo/fast_foundationstereo.engine"),
            ),
            DeclareLaunchArgument("descriptor_batch_size", default_value="5"),
            DeclareLaunchArgument("descriptor_stride", default_value="1"),
            DeclareLaunchArgument("verification_frame_batch_size", default_value="50"),
            DeclareLaunchArgument("flush_period_s", default_value="1.0"),
            DeclareLaunchArgument("loop_closure.alpha", default_value="0.5"),
            DeclareLaunchArgument("loop_closure.min_sim_vlad", default_value=""),
            DeclareLaunchArgument("loop_closure.bow_skip_num", default_value="1"),
            DeclareLaunchArgument("loop_closure.bow_batch_size", default_value="50"),
            DeclareLaunchArgument("loop_closure.vlc_batch_size", default_value="10"),
            DeclareLaunchArgument("loop_closure.loop_batch_size", default_value="50"),
            DeclareLaunchArgument(
                "loop_closure.loop_sync_sleep_time", default_value="10"
            ),
            DeclareLaunchArgument("loop_closure.comm_sleep_time", default_value="5"),
            DeclareLaunchArgument(
                "loop_closure.detection_batch_size", default_value="50"
            ),
            DeclareLaunchArgument("loop_closure.max_submap_size", default_value="10"),
            DeclareLaunchArgument(
                "loop_closure.max_submap_distance", default_value="5"
            ),
            DeclareLaunchArgument(
                "loop_closure.adaptive_scoring_tau_max",
                default_value="0.01",
            ),
            DeclareLaunchArgument(
                "loop_closure.adaptive_scoring_tau_min",
                default_value="0.01",
            ),
            DeclareLaunchArgument(
                "loop_closure.adaptive_scoring_lambda",
                default_value="1.0",
            ),
            DeclareLaunchArgument(
                "loop_closure.stereo_verification_method",
                default_value="opengv_pnp",
            ),
            DeclareLaunchArgument(
                "loop_closure.teaser_noise_bound_m", default_value="0.10"
            ),
            DeclareLaunchArgument("loop_closure.teaser_min_scale", default_value="0.5"),
            DeclareLaunchArgument("loop_closure.teaser_max_scale", default_value="2.0"),
            DeclareLaunchArgument(
                "loop_closure.orbslam3_reprojection_threshold_px",
                default_value="15.0",
            ),
            DeclareLaunchArgument(
                "loop_closure.orbslam3_min_scale", default_value="0.5"
            ),
            DeclareLaunchArgument(
                "loop_closure.orbslam3_max_scale", default_value="2.0"
            ),
            DeclareLaunchArgument(
                "loop_closure.verified_scale_sigma", default_value="0.10"
            ),
            DeclareLaunchArgument("pgo_formulation", default_value="sim3"),
            DeclareLaunchArgument("sim3_scale_sigma", default_value="0.1"),
            DeclareLaunchArgument("sim3_odom_scale_sigma", default_value="-1"),
            DeclareLaunchArgument("sim3_loop_scale_sigma", default_value="-1"),
            DeclareLaunchArgument("sim3_inter_loop_scale_sigma", default_value="-1"),
            DeclareLaunchArgument("sim3_pose_scale_prior_sigma", default_value="0.1"),
            DeclareLaunchArgument("play_bag", default_value="true"),
            DeclareLaunchArgument(
                "bag_path", default_value="/data/graco/aerial-05-40m"
            ),
            DeclareLaunchArgument("bag_rate", default_value="1.0"),
            DeclareLaunchArgument("bag_start_delay", default_value="30.0"),
            DeclareLaunchArgument("bag_playback_duration", default_value="-1"),
            DeclareLaunchArgument("visualization_mode", default_value="full"),
            DeclareLaunchArgument("log_output", default_value="false"),
            DeclareLaunchArgument(
                "log_output_path",
                default_value="/tmp/sb_slam_ros2_logs/aerial_single",
            ),
            DeclareLaunchArgument(
                "rerun_application_id",
                default_value="graco_aerial_single_robot",
            ),
            DeclareLaunchArgument(
                "rerun_recording_id",
                default_value=_timestamped_recording_id(),
            ),
            DeclareLaunchArgument(
                "rerun_host",
                default_value="rerun+http://127.0.0.1:9876/proxy",
            ),
            DeclareLaunchArgument("start_zenoh_router", default_value="true"),
            DeclareLaunchArgument("zenoh_router_startup_delay", default_value="1.0"),
            OpaqueFunction(function=_prepare_single_run),
            # Resolve the parent-owned bag settings before the delayed robot
            # include sets its nodes-only ``play_bag`` argument to false.
            OpaqueFunction(function=_schedule_bag_player),
            zenoh_router,
            TimerAction(
                period=LaunchConfiguration("zenoh_router_startup_delay"),
                actions=[robot_stack],
            ),
            OpaqueFunction(function=_shutdown_timer),
        ]
    )
