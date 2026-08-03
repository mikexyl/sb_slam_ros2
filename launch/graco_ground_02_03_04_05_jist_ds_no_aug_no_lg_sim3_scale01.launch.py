"""Run Ground 2–5 with ROS1-matched tracking and Sim3 scale sigma 0.1."""

from datetime import datetime
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _timestamped_recording_id():
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%z")
    return f"graco_g2345_jist08_ds_noaug_nolg_sim3_scale01_{timestamp}"


def _timestamped_output_path():
    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%z")
    workspace_root = Path(os.environ.get("SB_SLAM_ROS2_WS", os.getcwd()))
    run_name = (
        "jist-ds-noaug-nolg-boundary0.1-consecutive-disabled-"
        f"jist0.8-minsim0.85-distlocal30-sim3-scale0.1-{timestamp}"
    )
    return str(workspace_root / "src" / "code-logs" / "g2345" / run_name)


def _default_model_path(filename):
    workspace_root = Path(os.environ.get("SB_SLAM_ROS2_WS", os.getcwd()))
    return str(workspace_root / "src" / "xfeat-cpp" / "onnx_model" / filename)


def generate_launch_description():
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("sb_slam_ros2"),
                    "launch",
                    "graco_ground_02_03_04_05_multi_robot.launch.py",
                ]
            )
        ),
        launch_arguments={
            "active_robot_ids": LaunchConfiguration("active_robot_ids"),
            "robot_names_file": LaunchConfiguration("robot_names_file"),
            "ground_02_bag_path": LaunchConfiguration("ground_02_bag_path"),
            "ground_03_bag_path": LaunchConfiguration("ground_03_bag_path"),
            "ground_04_bag_path": LaunchConfiguration("ground_04_bag_path"),
            "ground_05_bag_path": LaunchConfiguration("ground_05_bag_path"),
            "play_bags": LaunchConfiguration("play_bags"),
            "bag_rate": LaunchConfiguration("bag_rate"),
            "bag_playback_duration": LaunchConfiguration(
                "bag_playback_duration"
            ),
            "bag_start_delay": LaunchConfiguration("bag_start_delay"),
            "vio_mode": "stereo",
            "vio_dataset_name": LaunchConfiguration("vio_dataset_name"),
            "distributed_dataset_name": LaunchConfiguration(
                "distributed_dataset_name"
            ),
            "vpr_model_type": LaunchConfiguration("vpr_model_type"),
            "use_external_odom": "false",
            "models.xfeat": LaunchConfiguration("models.xfeat"),
            "models.lightglue_frontend": LaunchConfiguration(
                "models.lightglue_frontend"
            ),
            "models.lightglue_lcd": LaunchConfiguration(
                "models.lightglue_lcd"
            ),
            "models.jist": LaunchConfiguration("models.jist"),
            "models.mixvpr": LaunchConfiguration("models.mixvpr"),
            "descriptor_batch_size": "5",
            "descriptor_stride": "1",
            "verification_frame_batch_size": "50",
            "flush_period_s": "1.0",
            "loop_closure.alpha": "0.5",
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
            "sim3_scale_sigma": "0.1",
            "sim3_odom_scale_sigma": "-1",
            "sim3_loop_scale_sigma": "-1",
            "sim3_inter_loop_scale_sigma": "-1",
            "belief_stage_switch_strategy": "random",
            "belief_stage_fixed_iterations": "10",
            "belief_republish_hellinger_threshold": "0.01",
            "visualization_mode": LaunchConfiguration("visualization_mode"),
            "log_output": LaunchConfiguration("log_output"),
            "log_output_path": LaunchConfiguration("log_output_path"),
            "rerun_application_id": LaunchConfiguration(
                "rerun_application_id"
            ),
            "rerun_recording_id": LaunchConfiguration("rerun_recording_id"),
            "rerun_host": LaunchConfiguration("rerun_host"),
            "start_zenoh_router": LaunchConfiguration("start_zenoh_router"),
            "zenoh_router_startup_delay": LaunchConfiguration(
                "zenoh_router_startup_delay"
            ),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "active_robot_ids", default_value="0,1,2,3"
            ),
            DeclareLaunchArgument(
                "robot_names_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("kimera_distributed"),
                        "params",
                        "robot_names_graco_gnd_2345.yaml",
                    ]
                ),
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
            DeclareLaunchArgument("play_bags", default_value="true"),
            DeclareLaunchArgument("bag_rate", default_value="1.0"),
            DeclareLaunchArgument(
                "bag_playback_duration", default_value="-1"
            ),
            DeclareLaunchArgument(
                "bag_start_delay",
                default_value="30.0",
                description=(
                    "Delay playback until all TensorRT engines initialize."
                ),
            ),
            DeclareLaunchArgument(
                "vio_dataset_name",
                default_value="GrAcoGndStereoXfeatJistDsNoAugNoLg",
                description=(
                    "VIO parameter profile. Override only for a named "
                    "sequence-strategy ablation."
                ),
            ),
            DeclareLaunchArgument(
                "distributed_dataset_name",
                default_value="GrAcoGndJistDynamic",
                description=(
                    "Distributed loop-closure parameter profile. Override "
                    "only for a named ablation."
                ),
            ),
            DeclareLaunchArgument(
                "vpr_model_type",
                default_value="jist",
                description=(
                    "Global descriptor model. Override only from a named "
                    "model ablation launch."
                ),
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
                "visualization_mode", default_value="minimal"
            ),
            DeclareLaunchArgument("log_output", default_value="true"),
            DeclareLaunchArgument(
                "log_output_path",
                default_value=_timestamped_output_path(),
            ),
            DeclareLaunchArgument(
                "rerun_application_id",
                default_value="graco_g2345_jist08_ds_noaug_nolg_sim3_scale01",
            ),
            DeclareLaunchArgument(
                "rerun_recording_id",
                default_value=_timestamped_recording_id(),
            ),
            DeclareLaunchArgument(
                "rerun_host",
                default_value="rerun+http://192.168.0.206:9876/proxy",
            ),
            DeclareLaunchArgument("start_zenoh_router", default_value="false"),
            DeclareLaunchArgument(
                "zenoh_router_startup_delay", default_value="1.0"
            ),
            base_launch,
        ]
    )
