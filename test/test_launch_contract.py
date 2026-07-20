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
    assert '"submap.metric_scale_method": LaunchConfiguration(' in profile
    assert '"submap.anchor_method": LaunchConfiguration(' in profile
    assert '"dense_mapping.submap_metric_scale_method",' in profile
    assert '"dense_mapping.submap_anchor_method",' in profile
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
    assert '"submap.metric_scale_method": LaunchConfiguration(' in profile
    assert '"submap.anchor_method": LaunchConfiguration(' in profile
    assert profile.count('default_value="odometry"') >= 2
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

    mapping_publisher = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/src/interfaces/"
        "dense_mapping_publisher.cpp"
    )
    assert '"mapping/local_window_poses"' in mapping_publisher
    assert "output.state_.keys()" in mapping_publisher
    assert "odometry_T_smoother" in mapping_publisher
    assert "pose_graph_tools_msgs::msg::PoseGraph" in mapping_publisher

    vio_header = _text(
        "Kimera-VIO/include/kimera-vio/backend/VioBackend-definitions.h"
    )
    assert "keyframe_measurements_" in vio_header
    assert "da3_packet_" in vio_header


def test_odometry_conditioned_da3_is_an_isolated_vio_only_experiment():
    experiment = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/launch/"
        "graco_aerial_05_odometry_conditioned_da3.launch.py"
    )
    assert "DA3-LARGE-1.1_pose_v2_350x504_fp16.engine" in experiment
    assert "LeftCameraParams.yaml" in experiment
    assert "'mono_depth.enabled': 'false'" in experiment
    assert "'dense_mapping.publisher_enabled': 'true'" in experiment
    assert "'use_lcd': '0'" in experiment
    assert "'multi_robot_bridge.enabled': 'false'" in experiment
    assert "'mono_depth.da3_essential_factors_enabled': 'false'" in experiment
    assert (
        "'mono_depth.da3_baseline_ratio_factors_enabled': 'false'"
        in experiment
    )
    assert "'selection.minimum_distance_m': '10.0'" in experiment
    assert "'minimum_confidence': '1.2'" in experiment
    assert "'record_experiment_inputs', default_value='true'" in experiment
    assert "'/a5/kimera_vio/mapping/local_window_poses'" in experiment
    assert experiment.count("rerun_recording_id") >= 3
    assert experiment.count("rerun_application_id") >= 3

    dense_launch = _text(
        "dense_mapping/launch/odometry_conditioned_da3.launch.py"
    )
    assert "odometry_conditioned_da3_node" in dense_launch
    assert "camera_calibration_path" in dense_launch
    assert "topics.camera_info" not in dense_launch
    assert "'submap.metric_scale_method': 'none'" in dense_launch
    assert "'submap.anchor_method': 'odometry'" in dense_launch
    assert "'submap.overlap_scale_method': 'none'" in dense_launch
    assert "'geometry_filter.enabled': 'false'" in dense_launch
    assert "'geometry_filter.apply_to_mapping': 'false'" in dense_launch
    assert "'geometry_filter.pose_source': 'da3'" in dense_launch
    assert "geometry_config.pose_source = GeometryPoseSource::kOdometry" in (
        _text("dense_mapping/src/odometry_conditioned_da3_node.cpp")
    )
    assert "poseToMessage(da3_context_T_current)" in (
        _text("dense_mapping/src/odometry_conditioned_da3_node.cpp")
    )
    assert "'max_runs_per_submap', default_value='5'" in dense_launch
    assert "topics.local_window_poses" in dense_launch
    assert "'ros2'," in dense_launch
    assert "'bag'," in dense_launch
    assert "'record'," in dense_launch
    assert "input_bag_record.output_directory" in dense_launch
    mapper_launch = _text("dense_mapping/launch/dense_mapping.launch.py")
    assert "on_exit=EmitEvent(event=Shutdown(" in mapper_launch

    visualizer = _text("dense_mapping/src/rerun_visualizer.cpp")
    assert "set_time_timestamp_nanos_since_epoch" in visualizer
    assert "std::chrono::system_clock" not in visualizer

    vio_visualizer = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/include/kimera_vio_ros/"
        "interfaces/RerunVisualizer.h"
    )
    assert "setTimeNSec(static_cast<size_t>(input.timestamp_))" in vio_visualizer
    assert "std::chrono::system_clock" not in vio_visualizer

    offline_launch = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/launch/"
        "graco_aerial_05_odometry_conditioned_da3_offline.launch.py"
    )
    assert "'playback_rate', default_value='1.0'" in offline_launch
    assert "'start_zenoh_router', default_value='true'" in offline_launch
    assert "'zenoh_router_startup_delay_s', default_value='1.0'" in (
        offline_launch
    )
    assert "'rmw_zenoh_cpp', 'rmw_zenohd'" in offline_launch
    assert "reason='ROS 2 Zenoh router exited'" in offline_launch
    assert "'request_global_ba'" in offline_launch
    assert "'std_srvs/srv/Trigger'" in offline_launch
    assert "'playback_delay_s', default_value='3.0'" in offline_launch
    assert "'playback_duration_s', default_value='118.0'" in offline_launch
    assert "'inference_drain_delay_s', default_value='60.0'" in offline_launch
    assert "'timeout'," in offline_launch
    assert "'--signal=INT'," in offline_launch
    assert "LaunchConfiguration('playback_delay_s')" in offline_launch
    assert "LaunchConfiguration('playback_duration_s')" in offline_launch
    assert "LaunchConfiguration('playback_rate')" in offline_launch
    assert "') / float('," in offline_launch
    assert "') + 1.0)'," in offline_launch
    assert "'raw_image_qos.reliability': 'reliable'" in offline_launch
    assert "'raw_image_qos.depth': '200'" in offline_launch
    assert "'output_qos.depth', default_value='1000'" in offline_launch
    assert "'output_qos.depth': LaunchConfiguration(" in offline_launch
    assert "'output_qos.depth', default_value='1000'" in dense_launch
    assert "'output_qos.depth': LaunchConfiguration(" in dense_launch
    assert "'image_cache.duration_s'" in offline_launch
    assert "--wait-for-all-acked" in offline_launch
    assert "--start-paused" in offline_launch
    assert "'/rosbag2_player/resume'" in offline_launch
    assert "'rosbag2_interfaces/srv/Resume'" in offline_launch
    assert "OnProcessExit" in offline_launch
    assert "inference_drain_delay_s" in offline_launch
    assert "input_bag_record.enabled': 'false'" in offline_launch
    assert "'submap_sparse_ba.global.enabled', default_value='true'" in (
        offline_launch
    )
    assert "'submap_sparse_ba.global.enabled': LaunchConfiguration(" in (
        offline_launch
    )
    assert (
        "'submap_sparse_ba.depth_refiner.enabled', default_value='true'"
        in offline_launch
    )
    assert (
        "'submap_sparse_ba.depth_refiner.enabled': LaunchConfiguration("
        in offline_launch
    )
    assert (
        "'submap_sparse_ba.depth_refiner.grid_rows', default_value='4'"
        in offline_launch
    )
    assert (
        "'submap_sparse_ba.depth_refiner.grid_cols', default_value='4'"
        in offline_launch
    )
    assert (
        "'submap_sparse_ba.depth_refiner.grid_rows': LaunchConfiguration("
        in offline_launch
    )
    assert (
        "'submap_sparse_ba.depth_refiner.grid_cols': LaunchConfiguration("
        in offline_launch
    )
    offline_concatenated = re.sub(r"'\s*'", "", offline_launch)
    dense_concatenated = re.sub(r"'\s*'", "", dense_launch)
    assert (
        "'submap_sparse_ba.depth_refiner."
        "sparse_landmark_constraints.enabled', default_value='true'"
        in offline_concatenated
    )
    assert (
        "'submap_sparse_ba.depth_refiner."
        "sparse_landmark_constraints.enabled', default_value='true'"
        in dense_concatenated
    )
    assert "'depth_refiner.sparse_landmark_constraints.enabled'" in (
        dense_launch
    )
    assert re.search(
        r"'submap_sparse_ba\.depth_refiner\.landmark_support_filter\."
        r"enabled',\s*default_value='true'",
        offline_launch,
    )
    assert re.search(
        r"'submap_sparse_ba\.depth_refiner\.landmark_support_filter\."
        r"enabled',\s*default_value='false'",
        dense_launch,
    )
    for suffix, default in (
        ('radius_px', '128'),
        ('minimum_landmarks', '3'),
    ):
        argument = (
            "'submap_sparse_ba.depth_refiner.landmark_support_filter."
            f"{suffix}'"
        )
        assert argument in offline_concatenated
        assert argument in dense_concatenated
        assert f"default_value='{default}'" in offline_launch
        assert f"default_value='{default}'" in dense_launch
    assert "'depth_refiner.landmark_support_filter.enabled'" in dense_launch
    assert "'depth_refiner.landmark_support_filter.radius_px'" in dense_launch
    assert (
        "'depth_refiner.landmark_support_filter.minimum_landmarks'"
        in dense_launch
    )
    assert re.search(
        r"'submap_sparse_ba\.depth_refiner\.two_view_consistency\."
        r"enabled',\s*default_value='false'",
        offline_launch,
    )
    assert re.search(
        r"'submap_sparse_ba\.depth_refiner\.two_view_consistency\."
        r"enabled',\s*default_value='true'",
        dense_launch,
    )
    for suffix, default in (
        ('sample_stride', '16'),
        ('maximum_constraints', '2000'),
        ('measurement_sigma', '0.20'),
        ('max_relative_depth_error', '0.25'),
    ):
        assert (
            "'submap_sparse_ba.depth_refiner.two_view_consistency."
            f"{suffix}'" in offline_concatenated
        )
        assert f"default_value='{default}'" in offline_launch
        assert (
            "'submap_sparse_ba.depth_refiner.two_view_consistency."
            f"{suffix}'" in dense_concatenated
        )
    assert "'depth_refiner.two_view_consistency.enabled'" in dense_launch
    assert (
        "'depth_refiner.two_view_consistency.maximum_constraints'"
        in dense_launch
    )
    assert "'submap_sparse_ba.pose_initialization_source'," in offline_launch
    assert "default_value='first_estimate'" in offline_launch
    assert "'submap_sparse_ba.pose_initialization_source': (" in (
        offline_launch
    )

    assert "'topics.global_refined_landmarks'" in dense_launch
    assert "'services.request_global_ba'" in dense_launch
    assert "'global.enabled': LaunchConfiguration(" in dense_launch
    assert "'pose_initialization_source': LaunchConfiguration(" in dense_launch
    assert "'topics.depth_refined_da3_runs'" in dense_launch
    assert "'topics.depth_refined_keyframes'" in dense_launch
    assert "'node.name': 'depth_refined_mapper'" in dense_launch
    assert "'node.name': 'odometry_anchored_mapper'" in dense_launch
    assert "'node.name': 'da3_chain_mapper'" in dense_launch
    assert "'node.name': 'no_pose_da3_mapper'" in dense_launch
    # Every pose-conditioned mapper consumes the one pose-scale-adjusted
    # stream; only the no-pose comparison has a native-scale stream.
    assert "'topics.native_da3_runs'" not in dense_launch
    assert "'native_da3_runs'" not in dense_launch
    da3_chain_section = dense_launch.split('da3_chain_mapper =', 1)[1]
    da3_chain_section = da3_chain_section.split('no_pose_da3_mapper =', 1)[0]
    assert "'topics.da3_runs'" in da3_chain_section
    assert "'da3_runs'" in da3_chain_section
    assert "'unconditioned_da3_runs'" not in da3_chain_section
    assert "'topics.unconditioned_da3_runs'" in dense_launch
    assert dense_launch.count("'unconditioned_da3_runs'") >= 2
    assert dense_launch.count(
        'GroupAction(actions=[IncludeLaunchDescription('
    ) == 4
    assert (
        "'comparison.da3_chain.enabled', default_value='false'"
        in dense_launch
    )
    assert "LaunchConfiguration('comparison.da3_chain.enabled')" in (
        dense_launch
    )
    assert (
        "'comparison.no_pose_da3.enabled', default_value='false'"
        in dense_launch
    )
    assert "LaunchConfiguration('comparison.no_pose_da3.enabled')" in (
        dense_launch
    )
    assert "'depth_refined/da3_runs'" in dense_launch
    assert "'depth_refined/keyframes'" in dense_launch
    assert "'max_runs_per_submap': '5'" in dense_launch
    assert dense_launch.count("'input_qos.depth': '1000'") == 4
    assert dense_launch.count('on_exit=EmitEvent(event=Shutdown(') == 2
    assert "'sparse_state_outputs.enabled': 'false'" in dense_launch
    assert dense_launch.count("'submap.metric_scale_method': 'none'") >= 2
    assert dense_launch.count("'submap.anchor_method': 'odometry'") >= 2
    assert dense_launch.count("'submap.view_pose_method': 'odometry'") == 1
    assert dense_launch.count("'submap.view_pose_method': 'da3'") == 3
    assert dense_launch.count("'submap.overlap_scale_method': 'none'") >= 2
    assert dense_launch.count("'geometry_filter.enabled': 'false'") >= 2
    assert (
        dense_launch.count("'geometry_filter.apply_to_mapping': 'false'")
        >= 2
    )
    for entity in (
        "'alignments',",
        "'odometry_anchored_da3',",
        "'da3_chain',",
        "'no_pose_da3',",
        "'grid_ba',",
    ):
        assert entity in dense_launch
    assert "'submap.anchor_method': 'da3'" in dense_launch
    assert "da3_chain/map/points" in dense_concatenated
    assert "no_pose_da3/map/points" in dense_concatenated
    assert (
        "'comparison.da3_chain.enabled', default_value='true'"
        in offline_launch
    )
    assert "'comparison.da3_chain.enabled': LaunchConfiguration(" in (
        offline_launch
    )
    assert (
        "'comparison.no_pose_da3.enabled', default_value='false'"
        in offline_launch
    )
    assert "'comparison.no_pose_da3.enabled': LaunchConfiguration(" in (
        offline_launch
    )
    assert 'DA3-LARGE-1.1_multiview_v2_350x504_fp16.engine' in (
        offline_launch
    )
    assert "full_union" not in dense_launch

    visualizer = _text("dense_mapping/src/rerun_visualizer.cpp")
    assert '"sparse_ba/global"' in visualizer
    assert 'prefix + "/odometry_trajectory"' in visualizer
    assert 'prefix + "/optimized_trajectory"' in visualizer
