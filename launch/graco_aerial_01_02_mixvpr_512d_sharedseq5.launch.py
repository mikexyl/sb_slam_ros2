"""Run the GrAco A1/A2 two-drone shared-sequence VPR experiment."""

from datetime import datetime
import os
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
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


_ROBOT_NAMES = ("a1", "a2")
_BAG_ARGUMENTS = ("aerial_01_bag_path", "aerial_02_bag_path")
_SOURCE_LEFT_IMAGE_TOPIC = "/camera_left/image_raw"
_SOURCE_RIGHT_IMAGE_TOPIC = "/camera_right/image_raw"
_SOURCE_IMU_TOPIC = "/gnss/imu"


def _timestamp():
    return datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%z")


def _workspace_root():
    return Path(os.environ.get("SB_SLAM_ROS2_WS", os.getcwd()))


def _default_model_path(filename):
    return str(_workspace_root() / "src" / "xfeat-cpp" / "onnx_model" / filename)


def _default_mixvpr_model_path():
    model_root = _workspace_root() / "src" / "xfeat-cpp" / "onnx_model"
    candidates = (
        model_root
        / "trt"
        / "mixvpr_resnet50_512d_fp16_sm120_trt10.13.engine",
        model_root / "mixvpr_resnet50_512d_fp16.engine",
    )
    for model_path in candidates:
        if model_path.is_file():
            return str(model_path)
    return str(candidates[0])


def _default_jist_model_path():
    model_root = _workspace_root() / "src" / "xfeat-cpp" / "onnx_model"
    candidates = (
        model_root / "JIST_r18_512_seqgem_simplified_fp32.engine",
        model_root / "JIST_r18_512_seqgem_simplified_fp16.engine",
    )
    for model_path in candidates:
        if model_path.is_file():
            return str(model_path)
    return str(candidates[0])


def _default_log_output_path():
    run_name = (
        "mixvpr-0.4-512d-lg-ds-noaug-sharedseq5-boundary0.1-"
        "consecutive-disabled-minsim0.85-distlocal30-sim3-scale0.05-"
        f"{_timestamp()}"
    )
    return str(_workspace_root() / "src" / "code-logs" / "a12" / run_name)


def _default_recording_id():
    return f"graco_a12_mixvpr04_512d_sharedseq5_{_timestamp()}"


def _as_bool(value):
    normalized = str(value).strip().lower()
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    raise RuntimeError(f"expected a boolean value, got {value!r}")


def _bag_topics(metadata_path):
    with metadata_path.open("r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream) or {}
    bag_info = metadata.get("rosbag2_bagfile_information", metadata)
    return {
        item.get("topic_metadata", {}).get("name")
        for item in bag_info.get("topics_with_message_count", [])
        if isinstance(item, dict)
    }


def _bag_duration_seconds(metadata_path):
    with metadata_path.open("r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream) or {}
    bag_info = metadata.get("rosbag2_bagfile_information", metadata)
    duration = bag_info.get("duration", {})
    if not isinstance(duration, dict) or "nanoseconds" not in duration:
        raise RuntimeError(
            f"ROS 2 bag metadata has no duration: {metadata_path}"
        )
    duration_seconds = float(duration["nanoseconds"]) / 1e9
    if duration_seconds <= 0.0:
        raise RuntimeError(
            f"ROS 2 bag duration must be positive: {metadata_path}"
        )
    return duration_seconds


def _bag_player(robot_name, bag_path, rate):
    source_topics = (
        _SOURCE_LEFT_IMAGE_TOPIC,
        _SOURCE_RIGHT_IMAGE_TOPIC,
        _SOURCE_IMU_TOPIC,
    )
    target_topics = (
        f"/{robot_name}/camera_left/image_raw",
        f"/{robot_name}/camera_right/image_raw",
        f"/{robot_name}/gnss/imu",
    )
    return ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            bag_path,
            "--rate",
            rate,
            "--read-ahead-queue-size",
            "1000",
            "--disable-keyboard-controls",
            "--topics",
            *source_topics,
            "--remap",
            *(
                f"{source}:={target}"
                for source, target in zip(source_topics, target_topics)
            ),
        ],
        output="screen",
    )


def _launch_setup(context, *args, **kwargs):
    names_path = Path(LaunchConfiguration("robot_names_file").perform(context))
    if not names_path.is_file():
        raise RuntimeError("robot_names_file must name an existing YAML file")
    with names_path.open("r", encoding="utf-8") as stream:
        names = yaml.safe_load(stream) or {}
    for robot_id, robot_name in enumerate(_ROBOT_NAMES):
        actual_name = names.get(f"robot{robot_id}_name")
        if actual_name != robot_name:
            raise RuntimeError(
                f"robot{robot_id}_name must be {robot_name}, got {actual_name}"
            )

    vio_dataset_name = LaunchConfiguration("vio_dataset_name").perform(context)
    distributed_dataset_name = LaunchConfiguration(
        "distributed_dataset_name"
    ).perform(context)
    vpr_model_type = LaunchConfiguration("vpr_model_type").perform(context)
    if vpr_model_type not in ("jist", "mixvpr"):
        raise RuntimeError(
            "vpr_model_type must be either 'jist' or 'mixvpr'"
        )
    vpr_similarity_threshold = LaunchConfiguration(
        "vpr_similarity_threshold"
    ).perform(context)
    try:
        threshold_value = float(vpr_similarity_threshold)
    except ValueError as error:
        raise RuntimeError(
            "vpr_similarity_threshold must be a number"
        ) from error
    if not 0.0 <= threshold_value <= 1.0:
        raise RuntimeError(
            "vpr_similarity_threshold must be between 0 and 1"
        )

    recording_id = LaunchConfiguration("rerun_recording_id").perform(context)
    if not recording_id:
        raise RuntimeError("rerun_recording_id must not be empty")

    play_bags = _as_bool(LaunchConfiguration("play_bags").perform(context))
    bag_paths = tuple(
        LaunchConfiguration(argument).perform(context)
        for argument in _BAG_ARGUMENTS
    )
    if play_bags:
        required_topics = {
            _SOURCE_LEFT_IMAGE_TOPIC,
            _SOURCE_RIGHT_IMAGE_TOPIC,
            _SOURCE_IMU_TOPIC,
        }
        for bag_path in bag_paths:
            path = Path(bag_path)
            metadata_path = path / "metadata.yaml"
            if not path.is_dir() or not metadata_path.is_file():
                raise RuntimeError(
                    "Full ROS 2 bag directory does not contain metadata.yaml: "
                    f"{path}"
                )
            missing_topics = required_topics - _bag_topics(metadata_path)
            if missing_topics:
                raise RuntimeError(
                    f"ROS 2 bag {path} is missing stereo topics: "
                    f"{sorted(missing_topics)}"
                )

    bag_start_delay = float(
        LaunchConfiguration("bag_start_delay").perform(context)
    )
    playback_duration = float(
        LaunchConfiguration("bag_playback_duration").perform(context)
    )
    playback_rate = float(LaunchConfiguration("bag_rate").perform(context))
    if bag_start_delay < 0.0:
        raise RuntimeError("bag_start_delay must be non-negative")
    if playback_duration < -1.0:
        raise RuntimeError(
            "bag_playback_duration must be -1 or non-negative"
        )
    if playback_rate <= 0.0:
        raise RuntimeError("bag_rate must be positive")

    log_root = Path(LaunchConfiguration("log_output_path").perform(context))
    log_root.mkdir(parents=True, exist_ok=True)
    manifest_arguments = (
        "aerial_01_bag_path",
        "aerial_02_bag_path",
        "play_bags",
        "bag_rate",
        "bag_start_delay",
        "bag_playback_duration",
        "models.xfeat",
        "models.lightglue_frontend",
        "models.lightglue_lcd",
        "models.jist",
        "models.mixvpr",
        "vio_dataset_name",
        "distributed_dataset_name",
        "vpr_model_type",
        "vpr_similarity_threshold",
        "rerun_application_id",
        "rerun_recording_id",
        "rerun_host",
    )
    run_manifest = {
        "launch_file": (
            "graco_aerial_01_02_mixvpr_512d_sharedseq5.launch.py"
        ),
        "robot_names_file": str(names_path),
        "robots": list(_ROBOT_NAMES),
        "resolved_launch_arguments": {
            argument: LaunchConfiguration(argument).perform(context)
            for argument in manifest_arguments
        },
        "fixed_setup": {
            "vio_mode": "stereo",
            "vio_dataset_name": vio_dataset_name,
            "distributed_dataset_name": distributed_dataset_name,
            "vpr_model_type": vpr_model_type,
            "vpr_similarity_threshold": threshold_value,
            "use_external_odom": False,
            "descriptor_batch_size": 5,
            "descriptor_stride": 1,
            "dynamic_sequence_boundary": 0.1,
            "consecutive_covisibility_gate": "disabled",
            "sequence_diversity_threshold": 0.85,
            "keyframe_diversity_threshold": 0.0,
            "landmark_projection": False,
            "lightglue_rematching": True,
            "dist_local": 30,
            "pgo_formulation": "sim3",
            "sim3_scale_sigma": 0.05,
            "dense_mapping": False,
            "visualization_mode": "minimal",
        },
    }
    with (log_root / "experiment_setup.yaml").open(
        "w", encoding="utf-8"
    ) as stream:
        yaml.safe_dump(run_manifest, stream, sort_keys=False)

    robot_launch = PathJoinSubstitution(
        [FindPackageShare("sb_slam_ros2"), "launch", "graco_robot.launch.py"]
    )
    shared_arguments = {
        "num_robots": "2",
        "robot_names_file": str(names_path),
        "vio_mode": "stereo",
        "vio_dataset_name": vio_dataset_name,
        "distributed_dataset_name": distributed_dataset_name,
        "vpr_model_type": vpr_model_type,
        "use_external_odom": "false",
        "models.xfeat": LaunchConfiguration("models.xfeat"),
        "models.lightglue_frontend": LaunchConfiguration(
            "models.lightglue_frontend"
        ),
        "models.lightglue_lcd": LaunchConfiguration("models.lightglue_lcd"),
        "models.jist": LaunchConfiguration("models.jist"),
        "models.mixvpr": LaunchConfiguration("models.mixvpr"),
        "log_output": "true",
        "descriptor_batch_size": "5",
        "descriptor_stride": "1",
        "verification_frame_batch_size": "50",
        "flush_period_s": "1.0",
        "loop_closure.alpha": "0.5",
        "loop_closure.min_sim_vlad": vpr_similarity_threshold,
        "loop_closure.bow_skip_num": "1",
        "loop_closure.bow_batch_size": "50",
        "loop_closure.vlc_batch_size": "10",
        "loop_closure.loop_batch_size": "50",
        "loop_closure.loop_sync_sleep_time": "10",
        "loop_closure.comm_sleep_time": "5",
        "loop_closure.detection_batch_size": "50",
        "loop_closure.max_submap_size": "10",
        "loop_closure.max_submap_distance": "5",
        "loop_closure.adaptive_scoring_tau_max": "0.0",
        "loop_closure.adaptive_scoring_tau_min": "0.0",
        "loop_closure.adaptive_scoring_lambda": "0.0",
        "pgo_formulation": "sim3",
        "sim3_scale_sigma": "0.05",
        "sim3_odom_scale_sigma": "-1",
        "sim3_loop_scale_sigma": "-1",
        "sim3_inter_loop_scale_sigma": "-1",
        "belief_stage_switch_strategy": "random",
        "belief_stage_fixed_iterations": "10",
        "belief_republish_hellinger_threshold": "0.01",
        "visualization_mode": "minimal",
        "dense_mapping.enabled": "false",
        "keyframe_state.publisher_enabled": "false",
        "play_bag": "false",
        "use_sim_time": "false",
        "rerun_application_id": LaunchConfiguration(
            "rerun_application_id"
        ),
        "rerun_recording_id": recording_id,
        "rerun_host": LaunchConfiguration("rerun_host"),
        "start_zenoh_router": "false",
    }

    actions = []
    players = []
    rate = LaunchConfiguration("bag_rate").perform(context)
    for robot_id, (robot_name, bag_path) in enumerate(
        zip(_ROBOT_NAMES, bag_paths)
    ):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_launch),
                launch_arguments={
                    **shared_arguments,
                    "robot_id": str(robot_id),
                    "robot_name": robot_name,
                    "log_output_path": str(log_root / robot_name),
                    "image_topic": f"/{robot_name}/camera_left/image_raw",
                    "right_image_topic": (
                        f"/{robot_name}/camera_right/image_raw"
                    ),
                    "imu_topic": f"/{robot_name}/gnss/imu",
                }.items(),
            )
        )
        if play_bags:
            players.append(_bag_player(robot_name, bag_path, rate))

    if players:
        actions.append(TimerAction(period=bag_start_delay, actions=players))
        if playback_duration >= 0.0:
            replay_wall_duration = playback_duration / playback_rate
        else:
            replay_wall_duration = max(
                _bag_duration_seconds(Path(bag_path) / "metadata.yaml")
                for bag_path in bag_paths
            ) / playback_rate
        actions.append(
            TimerAction(
                period=bag_start_delay + replay_wall_duration + 10.0,
                actions=[
                    EmitEvent(
                        event=Shutdown(
                            reason="A1/A2 complete bag replay finished"
                        )
                    )
                ],
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_names_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("kimera_distributed"),
                        "params",
                        "robot_names_graco_12.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "aerial_01_bag_path",
                default_value="/data3/graco/aerial-01-40m_full_ros2",
            ),
            DeclareLaunchArgument(
                "aerial_02_bag_path",
                default_value="/data3/graco/aerial-02-20m_full_ros2",
            ),
            DeclareLaunchArgument("play_bags", default_value="true"),
            DeclareLaunchArgument("bag_rate", default_value="1.0"),
            DeclareLaunchArgument("bag_start_delay", default_value="30.0"),
            DeclareLaunchArgument(
                "bag_playback_duration",
                default_value="-1",
                description="-1 plays complete bags; otherwise seconds.",
            ),
            DeclareLaunchArgument(
                "vio_dataset_name",
                default_value="GrAcoStereoXfeatMixVprDsNoAugSharedSeq5",
            ),
            DeclareLaunchArgument(
                "distributed_dataset_name",
                default_value="GrAcoMixVprDynamic",
            ),
            DeclareLaunchArgument(
                "vpr_model_type", default_value="mixvpr"
            ),
            DeclareLaunchArgument(
                "vpr_similarity_threshold", default_value="0.4"
            ),
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
                default_value=_default_jist_model_path(),
            ),
            DeclareLaunchArgument(
                "models.mixvpr",
                default_value=_default_mixvpr_model_path(),
            ),
            DeclareLaunchArgument(
                "log_output_path", default_value=_default_log_output_path()
            ),
            DeclareLaunchArgument(
                "rerun_application_id",
                default_value="graco_a12_mixvpr04_512d_sharedseq5",
            ),
            DeclareLaunchArgument(
                "rerun_recording_id",
                default_value=_default_recording_id(),
            ),
            DeclareLaunchArgument(
                "rerun_host",
                default_value="rerun+http://192.168.0.206:9876/proxy",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
