"""Run M2DGR Gate 1/2/3 RealSense mono with refined JIST 0.8.

This follows the ROS 1 M2DGRRS color-camera/handsfree-IMU setup, including its
40-state backend, 0.8x playback, and optical-flow-only frontend tracking. The
newer loop-closure path uses no sequence-descriptor diversity gate, a 0.1
sequence boundary, and distributed LightGlue verification after refinement.
"""

from datetime import datetime
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _timestamp():
    return datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%z")


def _workspace_root():
    return Path(os.environ.get("SB_SLAM_ROS2_WS", os.getcwd()))


def _default_model_path(filename):
    return str(_workspace_root() / "src" / "xfeat-cpp" / "onnx_model" / filename)


def generate_launch_description():
    timestamp = _timestamp()
    run_name = (
        "jist-ds-noaug-nolg-realsense-mono-boundary0.1-consecutive-disabled-"
        "jist0.8-seqdiv-off-distlocal90-sim3-framerefine-argmax-"
        f"{timestamp}"
    )
    output_path = (
        _workspace_root() / "src" / "code-logs" / "m2dgr" / "gate123" / run_name
    )
    recording_id = f"m2dgr_gate123_jist08_rsmono_framerefine_{timestamp}"

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("sb_slam_ros2"),
                    "launch",
                    "m2dgr_gate_01_02_03_multi_robot.launch.py",
                ]
            )
        ),
        launch_arguments={
            "active_robot_ids": LaunchConfiguration("active_robot_ids"),
            "robot_names_file": LaunchConfiguration("robot_names_file"),
            "gate_01_bag_path": LaunchConfiguration("gate_01_bag_path"),
            "gate_02_bag_path": LaunchConfiguration("gate_02_bag_path"),
            "gate_03_bag_path": LaunchConfiguration("gate_03_bag_path"),
            "play_bags": LaunchConfiguration("play_bags"),
            "bag_rate": LaunchConfiguration("bag_rate"),
            "bag_playback_duration": LaunchConfiguration(
                "bag_playback_duration"
            ),
            "bag_start_delay": LaunchConfiguration("bag_start_delay"),
            "vio_mode": "mono",
            "vio_dataset_name": "M2DGRRSXfeatJistDsNoAugNoLg",
            "distributed_dataset_name": "M2DGR",
            "vpr_model_type": "jist",
            "jist_frame_refinement": "true",
            "loop_closure.min_sim_score": "0.0",
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
            "stereo_depth.method": "",
            "models.stereo_depth": "",
            "descriptor_batch_size": "5",
            "descriptor_stride": "1",
            "verification_frame_batch_size": "50",
            "flush_period_s": "1.0",
            "loop_closure.alpha": "0.7",
            "loop_closure.min_sim_vlad": "0.8",
            "loop_closure.bow_skip_num": "1",
            "loop_closure.bow_batch_size": "100",
            "loop_closure.vlc_batch_size": "10",
            "loop_closure.loop_batch_size": "100",
            "loop_closure.loop_sync_sleep_time": "5",
            "loop_closure.comm_sleep_time": "5",
            "loop_closure.detection_batch_size": "30",
            "loop_closure.max_submap_size": "100",
            "loop_closure.max_submap_distance": "0.5",
            "loop_closure.adaptive_scoring_tau_max": "0.0",
            "loop_closure.adaptive_scoring_tau_min": "0.0",
            "loop_closure.adaptive_scoring_lambda": "0.1",
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
            "rerun_application_id": "m2dgr_gate123_jist08_rsmono_framerefine",
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
            DeclareLaunchArgument("active_robot_ids", default_value="0,1,2"),
            DeclareLaunchArgument(
                "robot_names_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("kimera_distributed"),
                        "params",
                        "robot_names_m2dgr_gate.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "gate_01_bag_path",
                default_value="/data/m2dgr/gate_01_realsense_ros2",
            ),
            DeclareLaunchArgument(
                "gate_02_bag_path",
                default_value="/data/m2dgr/gate_02_realsense_ros2",
            ),
            DeclareLaunchArgument(
                "gate_03_bag_path",
                default_value="/data/m2dgr/gate_03_realsense_ros2",
            ),
            DeclareLaunchArgument("play_bags", default_value="true"),
            DeclareLaunchArgument("bag_rate", default_value="0.8"),
            DeclareLaunchArgument("bag_playback_duration", default_value="-1"),
            DeclareLaunchArgument(
                "bag_start_delay",
                default_value="30.0",
                description="Wait for all TensorRT engines before playback.",
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
                    "JIST_r18_512_seqgem_frames_fp32.engine"
                ),
            ),
            DeclareLaunchArgument(
                "models.mixvpr",
                default_value=_default_model_path(
                    "trt/mixvpr_resnet50_512d_fp16_sm120_trt10.13.engine"
                ),
            ),
            DeclareLaunchArgument("visualization_mode", default_value="minimal"),
            DeclareLaunchArgument("log_output", default_value="true"),
            DeclareLaunchArgument(
                "log_output_path", default_value=str(output_path)
            ),
            DeclareLaunchArgument(
                "rerun_recording_id", default_value=recording_id
            ),
            DeclareLaunchArgument(
                "rerun_host",
                default_value="rerun+http://127.0.0.1:9876/proxy",
            ),
            DeclareLaunchArgument("start_zenoh_router", default_value="true"),
            DeclareLaunchArgument(
                "zenoh_router_startup_delay", default_value="1.0"
            ),
            base_launch,
        ]
    )
