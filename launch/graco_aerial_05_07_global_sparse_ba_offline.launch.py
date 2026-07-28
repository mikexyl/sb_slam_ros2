"""Replay A5/A7 sparse maps into two local BA nodes and CBS."""

from datetime import datetime
from pathlib import Path

from dense_mapping_launch import process_exit_handler
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_DEFAULT_INPUT_BAG = (
    "/data/graco/"
    "kimera-output-aerial05-aerial07-loop-landmarks-"
    "20260721_154000_+0300"
)

_REPLAY_TOPICS = (
    "/a5/kimera_vio/mapping/keyframes",
    "/a7/kimera_vio/mapping/keyframes",
    "/a5/kimera_vio/mapping/local_window_poses",
    "/a7/kimera_vio/mapping/local_window_poses",
    "/a5/kimera_vio/pose_graph/updates",
    "/a7/kimera_vio/pose_graph/updates",
    "/a5/kimera_distributed/pose_graph/updates",
    "/a7/kimera_distributed/pose_graph/updates",
    "/a5/kimera_distributed/keyframe_loop_closures",
    "/a7/kimera_distributed/keyframe_loop_closures",
)

def _timestamped_recording_id():
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%z")
    return f"graco-aerial-05-07-two-ba-offline-{timestamp}"


def _validate(context):
    bag_path = Path(LaunchConfiguration("input_bag_path").perform(context))
    if not bag_path.is_dir() or not (bag_path / "metadata.yaml").is_file():
        raise RuntimeError(
            "input_bag_path must name a finalized ROS 2 bag directory"
        )
    for name in (
        "playback_rate",
        "playback_delay_s",
        "playback_timeout_s",
        "optimization_delay_s",
        "experiment_timeout_s",
    ):
        if float(LaunchConfiguration(name).perform(context)) <= 0.0:
            raise RuntimeError(f"{name} must be positive")
    if int(LaunchConfiguration("read_ahead_queue_size").perform(context)) <= 0:
        raise RuntimeError("read_ahead_queue_size must be positive")
    return []


def _ba_node(robot, robot_id):
    prefix = f"/{robot}/global_sparse_ba"
    return Node(
        package="dense_mapping",
        executable="global_sparse_ba_node",
        name="global_sparse_ba",
        namespace=robot,
        output="screen",
        parameters=[
            {
                "robot_id": robot_id,
                "frame_id": f"{robot}/map",
                "loop_closure.required": False,
                "shutdown_after_optimization": True,
                "topics.keyframes": (
                    f"/{robot}/kimera_vio/mapping/keyframes"
                ),
                # The recorded closure is inter-robot. Keep local BA loop input
                # separate so it is never misinterpreted as an intra-map edge.
                "topics.loop_closures": f"{prefix}/local_loop_closures",
                "topics.refined_pose_graph": f"{prefix}/refined_pose_graph",
                "topics.pose_graph_only": f"{prefix}/pose_graph_only",
                "topics.refined_landmarks": f"{prefix}/refined_landmarks",
                "association.minimum_matches_per_closure": 3,
                "association.maximum_pixel_distance_px": 2.0,
                "synchronization.maximum_keyframes": 2000,
                "synchronization.maximum_closures": 100,
                "synchronization.maximum_pending_age_s": 30.0,
                "optimization_delay_s": LaunchConfiguration(
                    "optimization_delay_s"
                ),
                "minimum_track_observations": 2,
                "maximum_iterations": 15,
                "pose_graph.maximum_iterations": 100,
                "pose_graph.odometry_rotation_sigma_rad": 0.02,
                "pose_graph.odometry_translation_sigma_m": 0.05,
                "pose_graph.anchor_rotation_sigma_rad": 0.000001,
                "pose_graph.anchor_translation_sigma_m": 0.000001,
                "default_pixel_sigma": 1.0,
                "huber_threshold": 1.345,
                "optimize_poses": True,
                "pose_prior_rotation_sigma_rad": 0.05,
                "pose_prior_translation_sigma_m": 0.10,
                "maximum_final_track_rmse_px": 5.0,
                "minimum_triangulation_angle_deg": 1.0,
                "rerun.enabled": LaunchConfiguration("rerun.enabled"),
                "rerun.application_id": LaunchConfiguration(
                    "rerun.application_id"
                ),
                "rerun.recording_id": LaunchConfiguration(
                    "rerun.recording_id"
                ),
                "rerun.host": LaunchConfiguration("rerun.host"),
                "rerun.entity_prefix": (
                    f"experiments/{robot}/global_sparse_ba"
                ),
                "rerun.point_radius": 1.5,
            }
        ],
    )


def _cbs_node(robot, robot_id):
    return Node(
        package="cbs_ros",
        executable="cbs_ros_node",
        name=f"cbs_ros_node_{robot}",
        namespace=f"{robot}/cbs",
        output="screen",
        on_exit=EmitEvent(
            event=Shutdown(reason=f"{robot} CBS node exited")
        ),
        parameters=[
            {
                "robot_name": robot,
                "robot_id": robot_id,
                "num_robots": 2,
                "enable_gkcm": False,
                "d_reset": 0.5,
                "contract_alpha": 0.5,
                "loop_rate": 1.0,
                "log_dir": LaunchConfiguration("cbs_log_dir"),
                "enable_soft_reset": False,
                "enable_dcs": False,
                "max_iterations": -1,
                "online": True,
                "pose_graph_topic": f"/{robot}/kimera_vio/pose_graph/updates",
                "additional_pose_graph_topics": [
                    f"/{robot}/kimera_vio/mapping/local_window_poses",
                    f"/{robot}/kimera_distributed/pose_graph/updates",
                ],
                # Recorded VIO pose-graph and local-window publishers are
                # volatile; all inputs are replayed after CBS starts.
                "pose_graph_transient_local": False,
                "rerun_application_id": LaunchConfiguration(
                    "rerun.application_id"
                ),
                "rerun_recording_id": LaunchConfiguration(
                    "rerun.recording_id"
                ),
                "rerun_host": LaunchConfiguration("rerun.host"),
                "robot0_name": "a5",
                "robot1_name": "a7",
            }
        ],
    )


def _ba_completion_handlers(nodes):
    completed = set()
    handlers = []

    for label, node in nodes.items():

        def _on_exit(event, _context, *, completed_label=label):
            if event.returncode != 0:
                return [
                    EmitEvent(
                        event=Shutdown(
                            reason=(
                                f"{completed_label} global BA exited "
                                f"unexpectedly with code {event.returncode}"
                            )
                        )
                    )
                ]
            completed.add(completed_label)
            if completed == set(nodes):
                return [
                    EmitEvent(
                        event=Shutdown(
                            reason="A5 and A7 global sparse BA completed"
                        )
                    )
                ]
            return []

        handlers.append(
            RegisterEventHandler(
                OnProcessExit(target_action=node, on_exit=_on_exit)
            )
        )
    return handlers


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("input_bag_path", default_value=_DEFAULT_INPUT_BAG),
        DeclareLaunchArgument("start_zenoh_router", default_value="true"),
        DeclareLaunchArgument("zenoh_router_startup_delay_s", default_value="1.0"),
        DeclareLaunchArgument("playback_rate", default_value="10.0"),
        DeclareLaunchArgument("playback_delay_s", default_value="3.0"),
        DeclareLaunchArgument("playback_timeout_s", default_value="60.0"),
        DeclareLaunchArgument("read_ahead_queue_size", default_value="2000"),
        DeclareLaunchArgument("optimization_delay_s", default_value="2.0"),
        DeclareLaunchArgument("experiment_timeout_s", default_value="600.0"),
        DeclareLaunchArgument(
            "cbs_log_dir",
            default_value="",
            description="Empty disables CBS filesystem logs.",
        ),
        DeclareLaunchArgument("rerun.enabled", default_value="true"),
        DeclareLaunchArgument(
            "rerun.host",
            default_value="rerun+http://127.0.0.1:9876/proxy",
        ),
        DeclareLaunchArgument(
            "rerun.application_id",
            default_value="graco_aerial_05_07_distributed_optimization",
        ),
        DeclareLaunchArgument(
            "rerun.recording_id", default_value=_timestamped_recording_id()
        ),
    ]

    cbs_nodes = [_cbs_node("a5", 0), _cbs_node("a7", 1)]
    ba_nodes = {"a5": _ba_node("a5", 0), "a7": _ba_node("a7", 1)}
    player = ExecuteProcess(
        cmd=[
            "timeout",
            "--signal=INT",
            LaunchConfiguration("playback_timeout_s"),
            "ros2",
            "bag",
            "play",
            LaunchConfiguration("input_bag_path"),
            "--rate",
            LaunchConfiguration("playback_rate"),
            "--start-paused",
            "--read-ahead-queue-size",
            LaunchConfiguration("read_ahead_queue_size"),
            "--disable-keyboard-controls",
            "--wait-for-all-acked",
            "10000",
            "--topics",
            *_REPLAY_TOPICS,
        ],
        output="screen",
    )
    resume_request = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/rosbag2_player/resume",
            "rosbag2_interfaces/srv/Resume",
            "{}",
        ],
        output="screen",
    )
    player_exit = process_exit_handler(
        player, "A5/A7 offline rosbag player", expected_return_codes=(0,)
    )
    resume_exit = process_exit_handler(
        resume_request,
        "A5/A7 rosbag resume request",
        expected_return_codes=(0,),
    )
    zenoh_router = ExecuteProcess(
        cmd=["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_zenoh_router")),
        on_exit=EmitEvent(event=Shutdown(reason="ROS 2 Zenoh router exited")),
    )
    resume_player = TimerAction(
        period=LaunchConfiguration("playback_delay_s"),
        actions=[resume_request],
    )
    pipeline = TimerAction(
        period=LaunchConfiguration("zenoh_router_startup_delay_s"),
        actions=[*cbs_nodes, *ba_nodes.values(), player, resume_player],
    )
    experiment_timeout = TimerAction(
        period=LaunchConfiguration("experiment_timeout_s"),
        actions=[
            EmitEvent(
                event=Shutdown(
                    reason="timed out waiting for both global BA nodes"
                )
            )
        ],
    )

    return LaunchDescription(
        arguments
        + [
            OpaqueFunction(function=_validate),
            zenoh_router,
            player_exit,
            resume_exit,
            *_ba_completion_handlers(ba_nodes),
            pipeline,
            experiment_timeout,
        ]
    )
