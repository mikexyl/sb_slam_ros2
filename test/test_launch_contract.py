import importlib.util
from pathlib import Path
import re

import pytest


SRC = Path(__file__).resolve().parents[2]


def _text(relative):
    return (SRC / relative).read_text(encoding="utf-8")


def _module(relative):
    path = SRC / relative
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_vio_bridge_contract_and_qos():
    bridge = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/src/interfaces/"
        "multi_robot_loop_closure_bridge.cpp"
    )
    for endpoint in (
        '"pose_graph/updates"',
        '"pose_graph/get"',
        '"descriptors/global"',
        '"frames/verification"',
        '"frames/verification/get"',
    ):
        assert endpoint in bridge
    assert 'reliableTransientQos(100)' in bridge
    assert 'output->keyframe_id_' in bridge


def test_distributed_contract_snapshot_bootstrap_and_cbs_qos():
    distributed = _text("Kimera-Distributed/src/DistributedLoopClosureRos.cpp")
    for endpoint in (
        '"descriptors/requests"',
        '"descriptors/responses"',
        '"frames/requests"',
        '"frames/responses"',
        '"loops"',
        '"loops/ack"',
        '"pose_graph/updates"',
        '"pose_graph/get"',
    ):
        assert endpoint in distributed
    assert '/kimera_vio/pose_graph/get' in distributed
    assert 'requestVioPoseGraphSnapshot()' in distributed
    assert 'reliableTransientQos(1000)' in distributed

    cbs = _text("cbs_ros/src/cbs_ros_node.cpp")
    assert '/kimera_distributed/pose_graph/updates' in cbs
    assert '.reliable().transient_local()' in cbs


def test_graco_profile_enables_bridge_only_in_multi_robot_profile():
    profile = _text("sb_slam_ros2/launch/graco_robot.launch.py")
    distributed_pos = profile.index("distributed_launch")
    cbs_pos = profile.index("cbs_launch")
    vio_pos = profile.index("vio_launch")
    assert distributed_pos < cbs_pos < vio_pos
    assert '"use_lcd": "2"' in profile
    assert '"multi_robot_bridge.enabled": "true"' in profile
    assert '"mono_depth.enabled": dense_mapping_enabled_text' in profile
    assert '"start_zenoh_router": "false"' in profile

    generic = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/launch/kimera_vio_ros_mono.launch.py"
    )
    assert "'use_lcd',\n        default_value='0'" in generic
    assert "'multi_robot_bridge.enabled', default_value='false'" in generic

    mono_main = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/src/mono_vio_node.cpp"
    )
    assert "init_and_remove_ros_arguments" in mono_main
    assert "ParseCommandLineFlags" in mono_main

    graco_lcd = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/param/GrAcoMonoXfeat/LcdParams.yaml"
    )
    assert "vpr_model_type: jist" in graco_lcd
    assert '"models.jist"' in profile
    assert '"models.netvlad"' not in profile


def test_distributed_launch_rejects_incomplete_yaml(tmp_path):
    launch = _module(
        "Kimera-Distributed/launch/"
        "kimera_distributed_loop_closure_ros.launch.py"
    )
    config = tmp_path / "params.yaml"
    config.write_text("gt_files: []\n", encoding="utf-8")
    with pytest.raises(RuntimeError, match="non-scalar parameters"):
        launch._load_yaml(config)


def test_aerial_05_uses_one_timestamped_rerun_recording():
    multi_robot_launch = _module(
        "sb_slam_ros2/launch/graco_aerial_05_multi_robot.launch.py"
    )
    recording_id = multi_robot_launch._timestamped_recording_id()
    assert re.fullmatch(
        r"graco_aerial_05_\d{8}_\d{6}_[+-]\d{4}", recording_id
    )

    profile = _text("sb_slam_ros2/launch/graco_robot.launch.py")
    assert profile.count('"rerun_recording_id": rerun_recording_id') == 3
    assert profile.count('"rerun_application_id": rerun_application_id') == 3
    assert '"rerun.recording_id": rerun_recording_id' in profile
    assert '"rerun.application_id": rerun_application_id' in profile
    assert '"dense_mapping.point_stride", default_value="4"' in profile
    assert '"dense_mapping.max_points_per_submap",' in profile
    assert 'default_value="200000"' in profile
    assert '"max_points_per_submap": LaunchConfiguration(' in profile
    assert '"dense_mapping.max_runs_per_submap", default_value="5"' in profile
    assert '"max_runs_per_submap": LaunchConfiguration(' in profile
    assert '"mono_depth.da3_essential_factors_enabled": "false"' in profile
    assert (
        '"mono_depth.da3_baseline_ratio_factors_enabled": "false"'
        in profile
    )

    distributed = _text(
        "Kimera-Distributed/launch/"
        "kimera_distributed_loop_closure_ros.launch.py"
    )
    cbs = _text("cbs_ros/launch/cbs_ros_node.launch.py")
    vio = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/launch/"
        "kimera_vio_ros_mono.launch.py"
    )
    for launch_text in (distributed, cbs, vio):
        assert '"rerun_recording_id"' in launch_text or (
            "'rerun_recording_id'" in launch_text
        )
        assert '"rerun_application_id"' in launch_text or (
            "'rerun_application_id'" in launch_text
        )


def test_aerial_06_07_uses_separate_bags_without_cross_bag_clock():
    launch = _module(
        "sb_slam_ros2/launch/graco_aerial_06_07_multi_robot.launch.py"
    )
    recording_id = launch._timestamped_recording_id()
    assert re.fullmatch(
        r"graco_aerial_06_07_\d{8}_\d{6}_[+-]\d{4}", recording_id
    )

    launch_text = _text(
        "sb_slam_ros2/launch/graco_aerial_06_07_multi_robot.launch.py"
    )
    assert '"aerial_06_bag_path"' in launch_text
    assert '"aerial_07_bag_path"' in launch_text
    assert '"use_sim_time": "false"' in launch_text
    assert '"bag_publish_clock": "false"' in launch_text

    profile = _text("sb_slam_ros2/launch/graco_robot.launch.py")
    assert '"rosbag_publish_clock"' in profile
    assert '"rosbag_path": bag_path' in profile
    assert '"rosbag_publish_clock": bag_publish_clock' in profile
    assert '"use_sim_time": LaunchConfiguration("use_sim_time")' in profile
    assert "TimerAction(period=1.0, actions=[vio_launch])" in profile


def test_single_robot_launch_keeps_distributed_and_cbs_enabled():
    launch = _module(
        "sb_slam_ros2/launch/graco_single_robot_loop_closure.launch.py"
    )
    recording_id = launch._timestamped_recording_id()
    assert re.fullmatch(
        r"graco_single_robot_lcd_\d{8}_\d{6}_[+-]\d{4}", recording_id
    )

    launch_text = _text(
        "sb_slam_ros2/launch/graco_single_robot_loop_closure.launch.py"
    )
    assert '"robot_id": "0"' in launch_text
    assert '"num_robots": "1"' in launch_text
    assert '"play_bag": "true"' in launch_text
    assert '"use_sim_time": "true"' in launch_text
    assert '"bag_publish_clock": "true"' in launch_text
    assert '"dense_mapping.enabled": "true"' in launch_text
    assert '"models.da3": LaunchConfiguration("models.da3")' in launch_text
    assert '"graco_robot.launch.py"' in launch_text

    profile = _text("sb_slam_ros2/launch/graco_robot.launch.py")
    assert "distributed_launch" in profile
    assert "cbs_launch" in profile
    assert '"use_lcd": "2"' in profile
    assert '"multi_robot_bridge.enabled": "true"' in profile


def test_dense_mapping_is_single_robot_only_and_shares_rerun_recording():
    profile = _text("sb_slam_ros2/launch/graco_robot.launch.py")
    assert '"dense_mapping.enabled", default_value="false"' in profile
    assert 'FindPackageShare("dense_mapping")' in profile
    assert 'actions.append(dense_mapping_launch)' in profile
    assert '"dense_mapping.publisher_enabled": dense_mapping_enabled_text' in profile
    assert '"mono_depth.mode": "multi_view"' in profile
    assert '"mono_depth.da3_keyframe_selection_method": "distance"' in profile
    assert '"mono_depth.min_keyframe_distance_m": "10.0"' in profile
    assert '"mono_depth.min_confidence", default_value="1.2"' in profile
    assert '"mono_depth.min_confidence": LaunchConfiguration(' in profile
    assert '"mono_depth.scale_alignment_method": "landmarks"' in profile
    assert '"geometry_filter.enabled": LaunchConfiguration(' in profile
    assert '"geometry_filter.pose_source": LaunchConfiguration(' in profile
    assert (
        '"dense_mapping.geometry_filter_pose_source",' in profile
    )
    assert 'default_value="da3"' in profile
    assert profile.count('"rerun_recording_id": rerun_recording_id') == 3
    assert profile.count('"rerun_application_id": rerun_application_id') == 3
    assert '"rerun.recording_id": rerun_recording_id' in profile
    assert '"rerun.application_id": rerun_application_id' in profile

    for launch_name in (
        "graco_aerial_05_multi_robot.launch.py",
        "graco_aerial_06_07_multi_robot.launch.py",
    ):
        launch_text = _text(f"sb_slam_ros2/launch/{launch_name}")
        assert '"dense_mapping.enabled": "true"' not in launch_text


def test_vio_dense_mapping_publisher_topics_qos_and_gating():
    publisher = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/src/interfaces/"
        "dense_mapping_publisher.cpp"
    )
    assert '"mapping/da3_runs"' in publisher
    assert '"mapping/keyframes"' in publisher
    assert ".reliable().durability_volatile()" in publisher
    keyframe_publish = publisher.index(
        "keyframe_publisher_->publish(makeKeyframeState"
    )
    no_packet_gate = publisher.index("if (!output->da3_packet_)")
    valid_run_gate = publisher.index("if (!makeDa3Run(")
    da3_publish = publisher.index("da3_run_publisher_->publish(message)")
    assert keyframe_publish < no_packet_gate < valid_run_gate < da3_publish
    assert "output.cur_kf_id_ != 0u" in publisher
    assert "backend output is missing current keyframe measurements" in publisher
    assert "DA3 run metadata is incomplete" in publisher
    assert "packet.valid_mask" in publisher
    assert "packet.depth_support_mask" not in publisher
    assert "packet.confidence_filtering_enabled" in publisher

    inference = _text(
        "Kimera-VIO/src/frontend/MonoDepthInference.cpp"
    )
    assert "buildPacket(previous, depth_results[0], true)" in inference

    backend = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/src/interfaces/backend_interface.cpp"
    )
    assert "registerBackendOutputCallback" in backend
    assert "dense_mapping_publisher_->publish(output, odometry)" in backend

    vio_header = _text(
        "Kimera-VIO/include/kimera-vio/backend/VioBackend-definitions.h"
    )
    assert "keyframe_measurements_" in vio_header
    assert "da3_packet_" in vio_header
