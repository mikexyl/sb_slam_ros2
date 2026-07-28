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
    dense_mapping_enabled_text = LaunchConfiguration(
        "dense_mapping.enabled"
    ).perform(context).lower()
    if dense_mapping_enabled_text not in ("true", "false"):
        raise RuntimeError("dense_mapping.enabled must be true or false")
    dense_mapping_enabled = dense_mapping_enabled_text == "true"
    vio_mode = LaunchConfiguration("vio_mode").perform(context).strip().lower()
    if vio_mode not in ("mono", "stereo"):
        raise RuntimeError("vio_mode must be mono or stereo")
    if vio_mode == "stereo" and dense_mapping_enabled:
        raise RuntimeError(
            "dense mapping must be disabled for the stereo experiment"
        )
    vpr_model_type = LaunchConfiguration(
        "vpr_model_type"
    ).perform(context).strip().lower()
    if vpr_model_type not in ("jist", "mixvpr"):
        raise RuntimeError("vpr_model_type must be jist or mixvpr")

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
        )
    }
    models["models.jist"] = ""
    models["models.mixvpr"] = ""
    selected_vpr_argument = f"models.{vpr_model_type}"
    models[selected_vpr_argument] = _require_file(
        context, selected_vpr_argument
    )
    da3_engine = ""
    if dense_mapping_enabled:
        da3_engine = _require_file(context, "models.da3")
    vocabulary = LaunchConfiguration("vocabulary_path").perform(context)
    if vocabulary and not Path(vocabulary).is_file():
        raise RuntimeError("vocabulary_path does not exist")
    log_output_path = LaunchConfiguration("log_output_path").perform(context)
    Path(log_output_path).mkdir(parents=True, exist_ok=True)
    distributed_log_output_path = Path(log_output_path) / "distributed"
    distributed_log_output_path.mkdir(parents=True, exist_ok=True)
    rerun_application_id = LaunchConfiguration(
        "rerun_application_id"
    ).perform(context)
    rerun_recording_id = LaunchConfiguration("rerun_recording_id").perform(
        context
    )
    rerun_host = LaunchConfiguration("rerun_host").perform(context)
    if not rerun_application_id or not rerun_recording_id or not rerun_host:
        raise RuntimeError("Rerun application id, recording id, and host are required")
    visualization_mode = LaunchConfiguration("visualization_mode").perform(
        context
    ).strip().lower()
    if visualization_mode not in ("full", "minimal"):
        raise RuntimeError("visualization_mode must be full or minimal")
    detailed_rerun_enabled = (
        "true" if visualization_mode == "full" else "false"
    )

    image_topic = LaunchConfiguration("image_topic").perform(context)
    if not image_topic:
        image_topic = f"/{robot_name}/cam0/image_raw"
    right_image_topic = LaunchConfiguration(
        "right_image_topic"
    ).perform(context)
    if not right_image_topic:
        right_image_topic = f"/{robot_name}/cam1/image_raw"
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
            "dataset_name": LaunchConfiguration(
                "distributed_dataset_name"
            ),
            "frame_id": map_frame,
            "world_frame_id": world_frame,
            "odom_frame_id": odom_frame,
            "latest_kf_frame_id": latest_kf_frame,
            "vocab_path": vocabulary,
            "lightglue_model_path": models["models.lightglue_lcd"],
            "alpha": LaunchConfiguration("loop_closure.alpha"),
            "bow_skip_num": LaunchConfiguration(
                "loop_closure.bow_skip_num"
            ),
            "sparse_bow_ids": "true",
            "bow_batch_size": LaunchConfiguration(
                "loop_closure.bow_batch_size"
            ),
            "vlc_batch_size": LaunchConfiguration(
                "loop_closure.vlc_batch_size"
            ),
            "loop_batch_size": LaunchConfiguration(
                "loop_closure.loop_batch_size"
            ),
            "loop_sync_sleep_time": LaunchConfiguration(
                "loop_closure.loop_sync_sleep_time"
            ),
            "comm_sleep_time": LaunchConfiguration(
                "loop_closure.comm_sleep_time"
            ),
            "detection_batch_size": LaunchConfiguration(
                "loop_closure.detection_batch_size"
            ),
            "max_submap_size": LaunchConfiguration(
                "loop_closure.max_submap_size"
            ),
            "max_submap_distance": LaunchConfiguration(
                "loop_closure.max_submap_distance"
            ),
            "adaptive_scoring_tau_max": LaunchConfiguration(
                "loop_closure.adaptive_scoring_tau_max"
            ),
            "adaptive_scoring_tau_min": LaunchConfiguration(
                "loop_closure.adaptive_scoring_tau_min"
            ),
            "adaptive_scoring_lambda": LaunchConfiguration(
                "loop_closure.adaptive_scoring_lambda"
            ),
            "log_output_path": str(distributed_log_output_path),
            "rerun_application_id": rerun_application_id,
            "rerun_recording_id": rerun_recording_id,
            "rerun_host": rerun_host,
            "rerun_enabled": detailed_rerun_enabled,
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
            "pgo_formulation": LaunchConfiguration("pgo_formulation"),
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

    vio_launch_file = (
        "kimera_vio_ros_mono.launch.py"
        if vio_mode == "mono"
        else "kimera_vio_ros.launch.py"
    )
    vio_launch_arguments = {
        "dataset_name": LaunchConfiguration("vio_dataset_name"),
        "robot_id": str(robot_id),
        "robot_name": robot_name,
        "robot_namespace": robot_name,
        "use_lcd": "2",
        "log_output": LaunchConfiguration("log_output"),
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
        "models.mixvpr": models["models.mixvpr"],
        "frame_id.base_link": base_frame,
        "frame_id.odom": odom_frame,
        "frame_id.map": map_frame,
        "frame_id.world": world_frame,
        "topic.imu.data": imu_topic,
        "dense_mapping.publisher_enabled": LaunchConfiguration(
            "keyframe_state.publisher_enabled"
        ),
        "mono_depth.enabled": dense_mapping_enabled_text,
        "mono_depth.engine_path": da3_engine,
        "mono_depth.mode": "multi_view",
        "mono_depth.da3_keyframe_selection_method": "distance",
        "mono_depth.min_keyframe_distance_m": "10.0",
        "mono_depth.min_confidence": LaunchConfiguration(
            "mono_depth.min_confidence"
        ),
        "mono_depth.scale_alignment_method": "landmarks",
        "mono_depth.da3_essential_factors_enabled": "false",
        "mono_depth.da3_baseline_ratio_factors_enabled": "false",
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "log_output_path": log_output_path,
        "use_rerun_visualizer": detailed_rerun_enabled,
        "rerun_application_id": rerun_application_id,
        "rerun_recording_id": rerun_recording_id,
        "rerun_host": rerun_host,
        "start_zenoh_router": "false",
    }
    if vio_mode == "mono":
        vio_launch_arguments.update(
            {
                "topic.image": image_topic,
                "rosbag_play": play_bag,
                "rosbag_publish_clock": bag_publish_clock,
                "rosbag_path": bag_path,
                "rosbag_rate": bag_rate,
                "vio_start_delay": "0.0",
                "rosbag_source_image_topic": bag_source_image_topic,
                "rosbag_source_imu_topic": bag_source_imu_topic,
            }
        )
    else:
        vio_launch_arguments.update(
            {
                "topic.left.image": image_topic,
                "topic.right.image": right_image_topic,
            }
        )

    vio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("kimera_vio_ros"),
                    "launch",
                    vio_launch_file,
                ]
            )
        ),
        launch_arguments=vio_launch_arguments.items(),
    )

    dense_mapping_launch = None
    if dense_mapping_enabled:
        dense_mapping_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("dense_mapping"),
                        "launch",
                        "dense_mapping.launch.py",
                    ]
                )
            ),
            launch_arguments={
                "robot": robot_name,
                "frame_id.map": map_frame,
                "frame_id.odometry": world_frame,
                "point_stride": LaunchConfiguration(
                    "dense_mapping.point_stride"
                ),
                "max_points_per_view": LaunchConfiguration(
                    "dense_mapping.max_points_per_view"
                ),
                "max_points_per_submap": LaunchConfiguration(
                    "dense_mapping.max_points_per_submap"
                ),
                "max_runs_per_submap": LaunchConfiguration(
                    "dense_mapping.max_runs_per_submap"
                ),
                "min_depth_m": LaunchConfiguration(
                    "dense_mapping.min_depth_m"
                ),
                "max_depth_m": LaunchConfiguration(
                    "dense_mapping.max_depth_m"
                ),
                "submap.metric_scale_method": LaunchConfiguration(
                    "dense_mapping.submap_metric_scale_method"
                ),
                "submap.anchor_method": LaunchConfiguration(
                    "dense_mapping.submap_anchor_method"
                ),
                "geometry_filter.enabled": LaunchConfiguration(
                    "dense_mapping.geometry_filter_enabled"
                ),
                "geometry_filter.pose_source": LaunchConfiguration(
                    "dense_mapping.geometry_filter_pose_source"
                ),
                "geometry_filter.max_relative_depth_error": (
                    LaunchConfiguration(
                        "dense_mapping.geometry_filter_max_relative_depth_error"
                    )
                ),
                "geometry_filter.visualization_max_relative_error": (
                    LaunchConfiguration(
                        "dense_mapping.geometry_filter_visualization_max_relative_error"
                    )
                ),
                "rerun.enabled": "true",
                "rerun.application_id": rerun_application_id,
                "rerun.recording_id": rerun_recording_id,
                "rerun.host": rerun_host,
                "rerun.entity_prefix": f"{robot_name}/dense_mapping",
                "rerun.point_radius": LaunchConfiguration(
                    "dense_mapping.rerun_point_radius"
                ),
            }.items(),
        )

    # Keep delayed substitutions isolated: the generic VIO launch starts its
    # node immediately, while its bag callback captures concrete topic names.
    delayed_vio_launch = TimerAction(period=1.0, actions=[vio_launch])
    actions = [distributed_launch, cbs_launch]
    if dense_mapping_launch is not None:
        actions.append(dense_mapping_launch)
    actions.append(delayed_vio_launch)
    return actions


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
            DeclareLaunchArgument("vio_mode", default_value="mono"),
            DeclareLaunchArgument(
                "vio_dataset_name", default_value="GrAcoMonoXfeat"
            ),
            DeclareLaunchArgument(
                "distributed_dataset_name", default_value="GrAco"
            ),
            DeclareLaunchArgument("vpr_model_type", default_value="jist"),
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
            DeclareLaunchArgument("models.mixvpr", default_value=""),
            DeclareLaunchArgument("models.da3", default_value=""),
            DeclareLaunchArgument("vocabulary_path", default_value=""),
            DeclareLaunchArgument("log_output", default_value="false"),
            DeclareLaunchArgument("world_frame", default_value="world"),
            DeclareLaunchArgument("image_topic", default_value=""),
            DeclareLaunchArgument("right_image_topic", default_value=""),
            DeclareLaunchArgument("imu_topic", default_value=""),
            DeclareLaunchArgument("descriptor_batch_size", default_value="5"),
            DeclareLaunchArgument("descriptor_stride", default_value="1"),
            DeclareLaunchArgument(
                "verification_frame_batch_size", default_value="50"
            ),
            DeclareLaunchArgument("flush_period_s", default_value="1.0"),
            DeclareLaunchArgument(
                "loop_closure.alpha", default_value="0.7"
            ),
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
            DeclareLaunchArgument("sim3_scale_sigma", default_value="0.05"),
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
                "visualization_mode", default_value="full"
            ),
            DeclareLaunchArgument(
                "dense_mapping.enabled", default_value="false"
            ),
            DeclareLaunchArgument(
                "keyframe_state.publisher_enabled", default_value="true"
            ),
            DeclareLaunchArgument(
                "dense_mapping.point_stride", default_value="4"
            ),
            DeclareLaunchArgument(
                "dense_mapping.max_points_per_view", default_value="100000"
            ),
            DeclareLaunchArgument(
                "dense_mapping.max_points_per_submap",
                default_value="200000",
            ),
            DeclareLaunchArgument(
                "dense_mapping.max_runs_per_submap", default_value="5"
            ),
            DeclareLaunchArgument(
                "dense_mapping.submap_metric_scale_method",
                default_value="odometry",
            ),
            DeclareLaunchArgument(
                "dense_mapping.submap_anchor_method",
                default_value="odometry",
            ),
            DeclareLaunchArgument(
                "dense_mapping.min_depth_m", default_value="0.1"
            ),
            DeclareLaunchArgument(
                "dense_mapping.max_depth_m", default_value="100.0"
            ),
            DeclareLaunchArgument(
                "dense_mapping.geometry_filter_enabled",
                default_value="true",
            ),
            DeclareLaunchArgument(
                "dense_mapping.geometry_filter_pose_source",
                default_value="da3",
            ),
            DeclareLaunchArgument(
                "dense_mapping.geometry_filter_max_relative_depth_error",
                default_value="0.15",
            ),
            DeclareLaunchArgument(
                "dense_mapping.geometry_filter_visualization_max_relative_error",
                default_value="0.5",
            ),
            DeclareLaunchArgument(
                "dense_mapping.rerun_point_radius", default_value="1.0"
            ),
            DeclareLaunchArgument(
                "mono_depth.min_confidence", default_value="1.2"
            ),
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
