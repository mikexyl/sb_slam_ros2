"""Run the GrAco Ground 1/2/3/4/5/6 six-robot experiment.

Robot IDs and names match the tested ROS 1
``code_slam_graco_g123456.launch`` experiment. Each active robot runs stereo
VIO, Distributed loop closure, and Sim3 CBS. Dense mapping and its
keyframe-state publisher are deliberately disabled.

``active_robot_ids`` partitions the experiment across hosts.  For example,
run IDs ``1,2,3,4,5`` on the workstation and ID ``0`` on the Jetson while
keeping ``num_robots`` equal to six on both hosts.
"""

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
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


_EXPECTED_ROBOT_NAMES = ("g1", "g2", "g3", "g4", "g5", "g6")
_BAG_ARGUMENTS = (
    "ground_01_bag_path",
    "ground_02_bag_path",
    "ground_03_bag_path",
    "ground_04_bag_path",
    "ground_05_bag_path",
    "ground_06_bag_path",
)
_SOURCE_IMAGE_TOPIC = "/camera_left/image_raw"
_SOURCE_RIGHT_IMAGE_TOPIC = "/camera_right/image_raw"
_SOURCE_IMU_TOPIC = "/gnss/imu"


def _timestamped_recording_id():
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%z")
    return f"graco_ground_01_02_03_04_05_06_{timestamp}"


def _default_model_path(filename):
    workspace_root = Path(os.environ.get("SB_SLAM_ROS2_WS", os.getcwd()))
    return str(workspace_root / "src" / "xfeat-cpp" / "onnx_model" / filename)


def _as_bool(value):
    normalized = str(value).strip().lower()
    if normalized in ("1", "true", "yes", "on"):
        return True
    if normalized in ("0", "false", "no", "off"):
        return False
    raise RuntimeError(f"expected a boolean value, got {value!r}")


def _parse_active_robot_ids(value):
    tokens = [token.strip() for token in str(value).split(",")]
    if not tokens or any(not token for token in tokens):
        raise RuntimeError(
            "active_robot_ids must be a comma-separated list such as "
            "0,1,2,3,4,5"
        )
    try:
        robot_ids = tuple(int(token) for token in tokens)
    except ValueError as error:
        raise RuntimeError("active_robot_ids must contain integers") from error
    if len(set(robot_ids)) != len(robot_ids):
        raise RuntimeError("active_robot_ids must not contain duplicates")
    invalid = [
        robot_id
        for robot_id in robot_ids
        if robot_id < 0 or robot_id >= len(_EXPECTED_ROBOT_NAMES)
    ]
    if invalid:
        raise RuntimeError("active_robot_ids entries must be in [0, 6)")
    return robot_ids


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


def _bag_player(robot_name, bag_path, rate, stereo):
    target_image = f"/{robot_name}/camera_left/image_raw"
    target_right_image = f"/{robot_name}/camera_right/image_raw"
    target_imu = f"/{robot_name}/gnss/imu"
    command = [
        "ros2",
        "bag",
        "play",
        bag_path,
        "--rate",
        str(rate),
        "--read-ahead-queue-size",
        "1000",
        "--disable-keyboard-controls",
    ]
    source_topics = [_SOURCE_IMAGE_TOPIC]
    remappings = [f"{_SOURCE_IMAGE_TOPIC}:={target_image}"]
    if stereo:
        source_topics.append(_SOURCE_RIGHT_IMAGE_TOPIC)
        remappings.append(
            f"{_SOURCE_RIGHT_IMAGE_TOPIC}:={target_right_image}"
        )
    source_topics.append(_SOURCE_IMU_TOPIC)
    remappings.append(f"{_SOURCE_IMU_TOPIC}:={target_imu}")
    command.extend(["--topics", *source_topics, "--remap", *remappings])
    return ExecuteProcess(cmd=command, output="screen")


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
                f"robot{robot_id}_name must be {expected_name}, "
                f"got {actual_name}"
            )

    active_robot_ids = _parse_active_robot_ids(
        LaunchConfiguration("active_robot_ids").perform(context)
    )
    recording_id = LaunchConfiguration("rerun_recording_id").perform(context)
    if not recording_id:
        raise RuntimeError("rerun_recording_id must not be empty")
    vio_mode = LaunchConfiguration("vio_mode").perform(context).strip().lower()
    if vio_mode not in ("mono", "stereo"):
        raise RuntimeError("vio_mode must be mono or stereo")
    stereo = vio_mode == "stereo"

    play_bags = _as_bool(LaunchConfiguration("play_bags").perform(context))
    bag_paths = tuple(
        LaunchConfiguration(argument).perform(context)
        for argument in _BAG_ARGUMENTS
    )
    if play_bags:
        for robot_id in active_robot_ids:
            path = Path(bag_paths[robot_id])
            if not path.is_dir() or not (path / "metadata.yaml").is_file():
                raise RuntimeError(
                    "ROS 2 bag directory does not contain metadata.yaml: "
                    f"{path}"
                )
            required_topics = {_SOURCE_IMAGE_TOPIC, _SOURCE_IMU_TOPIC}
            if stereo:
                required_topics.add(_SOURCE_RIGHT_IMAGE_TOPIC)
            missing_topics = required_topics - _bag_topic_names(
                path / "metadata.yaml"
            )
            if missing_topics:
                raise RuntimeError(
                    f"ROS 2 bag {path} is missing required {vio_mode} "
                    f"topics: {sorted(missing_topics)}"
                )

    bag_start_delay = float(
        LaunchConfiguration("bag_start_delay").perform(context)
    )
    playback_duration = float(
        LaunchConfiguration("bag_playback_duration").perform(context)
    )
    if bag_start_delay < 0.0:
        raise RuntimeError("bag_start_delay must be non-negative")
    if playback_duration < -1.0:
        raise RuntimeError("bag_playback_duration must be -1 or non-negative")

    log_root = Path(
        LaunchConfiguration("log_output_path").perform(context)
    )
    log_root.mkdir(parents=True, exist_ok=True)

    # Preserve the exact resolved experiment setup beside every result. This
    # includes launch defaults and any intentional command-line overrides.
    manifest_arguments = (
        "vio_mode",
        "vio_dataset_name",
        "distributed_dataset_name",
        "vpr_model_type",
        "use_external_odom",
        "play_bags",
        "bag_rate",
        "bag_start_delay",
        "bag_playback_duration",
        "models.xfeat",
        "models.lightglue_frontend",
        "models.lightglue_lcd",
        "models.jist",
        "models.mixvpr",
        "stereo_depth.method",
        "models.stereo_depth",
        "descriptor_batch_size",
        "descriptor_stride",
        "verification_frame_batch_size",
        "flush_period_s",
        "loop_closure.alpha",
        "loop_closure.bow_skip_num",
        "loop_closure.bow_batch_size",
        "loop_closure.vlc_batch_size",
        "loop_closure.loop_batch_size",
        "loop_closure.loop_sync_sleep_time",
        "loop_closure.comm_sleep_time",
        "loop_closure.detection_batch_size",
        "loop_closure.max_submap_size",
        "loop_closure.max_submap_distance",
        "loop_closure.adaptive_scoring_tau_max",
        "loop_closure.adaptive_scoring_tau_min",
        "loop_closure.adaptive_scoring_lambda",
        "pgo_formulation",
        "sim3_scale_sigma",
        "sim3_odom_scale_sigma",
        "sim3_loop_scale_sigma",
        "sim3_inter_loop_scale_sigma",
        "belief_stage_switch_strategy",
        "belief_stage_fixed_iterations",
        "belief_republish_hellinger_threshold",
        "visualization_mode",
        "rerun_application_id",
        "rerun_recording_id",
        "rerun_host",
        "start_zenoh_router",
    )
    run_manifest = {
        "launch_file": "graco_ground_01_02_03_04_05_06_multi_robot.launch.py",
        "active_robot_ids": list(active_robot_ids),
        "robot_names_file": str(names_path),
        "bag_paths": {
            robot_name: bag_paths[robot_id]
            for robot_id, robot_name in enumerate(_EXPECTED_ROBOT_NAMES)
        },
        "resolved_launch_arguments": {
            argument: LaunchConfiguration(argument).perform(context)
            for argument in manifest_arguments
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
        "num_robots": "6",
        "robot_names_file": str(names_path),
        "vio_mode": vio_mode,
        "vio_dataset_name": LaunchConfiguration("vio_dataset_name"),
        "distributed_dataset_name": LaunchConfiguration(
            "distributed_dataset_name"
        ),
        "vpr_model_type": LaunchConfiguration("vpr_model_type"),
        "use_external_odom": LaunchConfiguration("use_external_odom"),
        "models.xfeat": LaunchConfiguration("models.xfeat"),
        "models.lightglue_frontend": LaunchConfiguration(
            "models.lightglue_frontend"
        ),
        "models.lightglue_lcd": LaunchConfiguration("models.lightglue_lcd"),
        "models.jist": LaunchConfiguration("models.jist"),
        "models.mixvpr": LaunchConfiguration("models.mixvpr"),
        "stereo_depth.method": LaunchConfiguration("stereo_depth.method"),
        "models.stereo_depth": LaunchConfiguration("models.stereo_depth"),
        "vocabulary_path": LaunchConfiguration("vocabulary_path"),
        "log_output": LaunchConfiguration("log_output"),
        "descriptor_batch_size": LaunchConfiguration("descriptor_batch_size"),
        "descriptor_stride": LaunchConfiguration("descriptor_stride"),
        "verification_frame_batch_size": LaunchConfiguration(
            "verification_frame_batch_size"
        ),
        "flush_period_s": LaunchConfiguration("flush_period_s"),
        "loop_closure.alpha": LaunchConfiguration("loop_closure.alpha"),
        "loop_closure.bow_skip_num": LaunchConfiguration(
            "loop_closure.bow_skip_num"
        ),
        "loop_closure.bow_batch_size": LaunchConfiguration(
            "loop_closure.bow_batch_size"
        ),
        "loop_closure.vlc_batch_size": LaunchConfiguration(
            "loop_closure.vlc_batch_size"
        ),
        "loop_closure.loop_batch_size": LaunchConfiguration(
            "loop_closure.loop_batch_size"
        ),
        "loop_closure.loop_sync_sleep_time": LaunchConfiguration(
            "loop_closure.loop_sync_sleep_time"
        ),
        "loop_closure.comm_sleep_time": LaunchConfiguration(
            "loop_closure.comm_sleep_time"
        ),
        "loop_closure.detection_batch_size": LaunchConfiguration(
            "loop_closure.detection_batch_size"
        ),
        "loop_closure.max_submap_size": LaunchConfiguration(
            "loop_closure.max_submap_size"
        ),
        "loop_closure.max_submap_distance": LaunchConfiguration(
            "loop_closure.max_submap_distance"
        ),
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
        "pgo_formulation": LaunchConfiguration("pgo_formulation"),
        "sim3_odom_scale_sigma": LaunchConfiguration(
            "sim3_odom_scale_sigma"
        ),
        "sim3_loop_scale_sigma": LaunchConfiguration(
            "sim3_loop_scale_sigma"
        ),
        "sim3_inter_loop_scale_sigma": LaunchConfiguration(
            "sim3_inter_loop_scale_sigma"
        ),
        "belief_stage_switch_strategy": LaunchConfiguration(
            "belief_stage_switch_strategy"
        ),
        "belief_stage_fixed_iterations": LaunchConfiguration(
            "belief_stage_fixed_iterations"
        ),
        "belief_republish_hellinger_threshold": LaunchConfiguration(
            "belief_republish_hellinger_threshold"
        ),
        "visualization_mode": LaunchConfiguration("visualization_mode"),
        "dense_mapping.enabled": "false",
        "keyframe_state.publisher_enabled": "false",
        "play_bag": "false",
        "use_sim_time": "false",
        "log_output_path": LaunchConfiguration("log_output_path"),
        "rerun_application_id": LaunchConfiguration("rerun_application_id"),
        "rerun_recording_id": recording_id,
        "rerun_host": LaunchConfiguration("rerun_host"),
        "start_zenoh_router": "false",
    }

    actions = []
    bag_players = []
    rate = LaunchConfiguration("bag_rate").perform(context)
    for robot_id in active_robot_ids:
        robot_name = _EXPECTED_ROBOT_NAMES[robot_id]
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_launch),
                launch_arguments={
                    **shared_arguments,
                    "robot_id": str(robot_id),
                    "robot_name": robot_name,
                    "log_output_path": str(log_root / robot_name),
                    "image_topic": (
                        f"/{robot_name}/camera_left/image_raw"
                    ),
                    "right_image_topic": (
                        f"/{robot_name}/camera_right/image_raw"
                    ),
                    "imu_topic": f"/{robot_name}/gnss/imu",
                }.items(),
            )
        )
        if play_bags:
            bag_players.append(
                _bag_player(
                    robot_name,
                    bag_paths[robot_id],
                    rate,
                    stereo,
                )
            )

    if bag_players:
        actions.append(
            TimerAction(period=bag_start_delay, actions=bag_players)
        )
        if playback_duration >= 0.0:
            actions.append(
                TimerAction(
                    period=bag_start_delay + playback_duration + 5.0,
                    actions=[
                        EmitEvent(
                            event=Shutdown(
                                reason=(
                                    "Ground 1/2/3/4/5/6 smoke replay completed"
                                )
                            )
                        )
                    ],
                )
            )
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
                "active_robot_ids",
                default_value="0,1,2,3,4,5",
                description=(
                    "Comma-separated robot IDs to launch on this host."
                ),
            ),
            DeclareLaunchArgument(
                "robot_names_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("kimera_distributed"),
                        "params",
                        "robot_names_graco_gnd_123456.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("vio_mode", default_value="stereo"),
            DeclareLaunchArgument(
                "vio_dataset_name", default_value="GrAcoGndStereoXfeat"
            ),
            DeclareLaunchArgument(
                "distributed_dataset_name", default_value="GrAcoGnd"
            ),
            DeclareLaunchArgument("vpr_model_type", default_value="jist"),
            DeclareLaunchArgument(
                "use_external_odom", default_value="false"
            ),
            DeclareLaunchArgument(
                "ground_01_bag_path",
                default_value="/data3/graco/ground-01_full_ros2",
            ),
            DeclareLaunchArgument(
                "ground_02_bag_path",
                default_value="/data3/mikexyl/graco/ground-02",
            ),
            DeclareLaunchArgument(
                "ground_03_bag_path",
                default_value="/data3/mikexyl/graco/ground-03_ros2",
            ),
            DeclareLaunchArgument(
                "ground_04_bag_path",
                default_value="/data3/graco/ground-04_full_ros2",
            ),
            DeclareLaunchArgument(
                "ground_05_bag_path",
                default_value="/data3/graco/ground-05_full_ros2",
            ),
            DeclareLaunchArgument(
                "ground_06_bag_path",
                default_value="/data3/graco/ground-06_full_ros2",
            ),
            DeclareLaunchArgument("play_bags", default_value="true"),
            DeclareLaunchArgument("bag_rate", default_value="1.0"),
            DeclareLaunchArgument("bag_start_delay", default_value="20.0"),
            DeclareLaunchArgument(
                "bag_playback_duration",
                default_value="-1",
                description="-1 plays complete bags; otherwise seconds.",
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
                "stereo_depth.method", default_value=""
            ),
            DeclareLaunchArgument(
                "models.stereo_depth", default_value=""
            ),
            DeclareLaunchArgument("vocabulary_path", default_value=""),
            DeclareLaunchArgument("log_output", default_value="false"),
            DeclareLaunchArgument("descriptor_batch_size", default_value="5"),
            DeclareLaunchArgument("descriptor_stride", default_value="1"),
            DeclareLaunchArgument(
                "verification_frame_batch_size", default_value="50"
            ),
            DeclareLaunchArgument("flush_period_s", default_value="1.0"),
            DeclareLaunchArgument("loop_closure.alpha", default_value="0.7"),
            DeclareLaunchArgument(
                "loop_closure.bow_skip_num", default_value="1"
            ),
            DeclareLaunchArgument(
                "loop_closure.bow_batch_size", default_value="100"
            ),
            DeclareLaunchArgument(
                "loop_closure.vlc_batch_size", default_value="10"
            ),
            DeclareLaunchArgument(
                "loop_closure.loop_batch_size", default_value="100"
            ),
            DeclareLaunchArgument(
                "loop_closure.loop_sync_sleep_time", default_value="5"
            ),
            DeclareLaunchArgument(
                "loop_closure.comm_sleep_time", default_value="5"
            ),
            DeclareLaunchArgument(
                "loop_closure.detection_batch_size", default_value="30"
            ),
            DeclareLaunchArgument(
                "loop_closure.max_submap_size", default_value="100"
            ),
            DeclareLaunchArgument(
                "loop_closure.max_submap_distance", default_value="0.5"
            ),
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
                "pgo_formulation", default_value="sim3"
            ),
            DeclareLaunchArgument(
                "sim3_odom_scale_sigma", default_value="-1"
            ),
            DeclareLaunchArgument(
                "sim3_loop_scale_sigma", default_value="-1"
            ),
            DeclareLaunchArgument(
                "sim3_inter_loop_scale_sigma", default_value="-1"
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
                "visualization_mode", default_value="minimal"
            ),
            DeclareLaunchArgument(
                "log_output_path",
                default_value="/tmp/sb_slam_ros2_logs/g123456",
            ),
            DeclareLaunchArgument(
                "rerun_application_id",
                default_value="graco_ground_01_02_03_04_05_06_multi_robot",
            ),
            DeclareLaunchArgument(
                "rerun_recording_id",
                default_value=_timestamped_recording_id(),
                description=(
                    "Use the same value on every participating host."
                ),
            ),
            DeclareLaunchArgument(
                "rerun_host",
                default_value="rerun+http://127.0.0.1:9876/proxy",
            ),
            DeclareLaunchArgument("start_zenoh_router", default_value="true"),
            DeclareLaunchArgument(
                "zenoh_router_startup_delay",
                default_value="1.0",
                description=(
                    "Seconds to let the Zenoh router begin listening before "
                    "starting ROS nodes."
                ),
            ),
            zenoh_router,
            TimerAction(
                period=LaunchConfiguration("zenoh_router_startup_delay"),
                actions=[OpaqueFunction(function=_launch_setup)],
            ),
        ]
    )
