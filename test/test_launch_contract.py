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
    assert "pose_graph_qos.transient_local()" in cbs
    assert "pose_graph_qos.durability_volatile()" in cbs


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


def test_runtime_is_tensorrt_only_and_mixvpr_defaults_to_512d():
    detector_header = _text(
        "Kimera-VIO/include/kimera-vio/loopclosure/"
        "VLADLoopClosureDetector.h"
    )
    detector_source = _text(
        "Kimera-VIO/src/loopclosure/VLADLoopClosureDetector.cpp"
    )
    params_source = _text(
        "Kimera-VIO/src/loopclosure/LoopClosureDetectorParams.cpp"
    )
    ros_interface = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/src/interfaces/base_interface.cpp"
    )
    pipeline_header = _text("Kimera-VIO/include/kimera-vio/pipeline/Pipeline.h")
    pipeline_source = _text("Kimera-VIO/src/pipeline/Pipeline.cpp")
    stereo_matcher = _text("Kimera-VIO/src/frontend/StereoMatcher.cpp")

    runtime_text = "\n".join(
        (
            detector_header,
            detector_source,
            params_source,
            ros_interface,
            pipeline_header,
            pipeline_source,
            stereo_matcher,
        )
    )
    for removed_path in (
        "jist_onnx.h",
        "mixvpr_onnx.h",
        "patchnetvlad_onnx.h",
        "JistONNX",
        "MixVPRONNX",
        "PatchNetVLADONNX",
        "VPRONNXWrapper",
        "kPatchNetVLAD",
        "Ort::",
        "OnnxStereoDepth",
        "stereo_depth_onnx.h",
    ):
        assert removed_path not in runtime_text
    assert "VPR models require a native TensorRT .engine file" in detector_source
    assert "TensorRT JIST and MixVPR are the only supported VPR models" in params_source
    assert "requires a TensorRT .engine file" in ros_interface

    runtime_config_files = [
        *list((SRC / "Kimera-VIO" / "params").rglob("*.yaml")),
        *list(
            (SRC / "Kimera-VIO-ROS2" / "kimera_vio_ros" / "param").rglob(
                "*.yaml"
            )
        ),
        *list((SRC / "sb_slam_ros2" / "launch").glob("*.py")),
    ]
    for path in runtime_config_files:
        text = path.read_text(encoding="utf-8")
        assert ".onnx" not in text, path

    ground_launch = _text(
        "sb_slam_ros2/launch/"
        "graco_ground_01_02_03_04_05_06_multi_robot.launch.py"
    )
    assert "mixvpr_resnet50_512d_fp16_sm120_trt10.13.engine" in ground_launch


def test_distributed_launch_rejects_incomplete_yaml(tmp_path):
    launch = _module(
        "Kimera-Distributed/launch/"
        "kimera_distributed_loop_closure_ros.launch.py"
    )
    config = tmp_path / "params.yaml"
    config.write_text("gt_files: []\n", encoding="utf-8")
    with pytest.raises(RuntimeError, match="non-scalar parameters"):
        launch._load_yaml(config)


def test_graco_and_cbs_default_to_scale_sigma_point_one():
    launch_files = (
        "cbs_ros/launch/cbs_ros_node.launch.py",
        "cbs_ros/launch/example.launch.py",
        "sb_slam_ros2/launch/graco_robot.launch.py",
        "sb_slam_ros2/launch/graco_ground_robot.launch.py",
        "sb_slam_ros2/launch/graco_ground_01_02_03_multi_robot.launch.py",
        "sb_slam_ros2/launch/graco_ground_02_03_04_05_multi_robot.launch.py",
        (
            "sb_slam_ros2/launch/"
            "graco_ground_01_02_03_04_05_06_multi_robot.launch.py"
        ),
        "sb_slam_ros2/launch/graco_aerial_05_06_07_08_multi_robot.launch.py",
    )
    for launch_file in launch_files:
        assert (
            'DeclareLaunchArgument("sim3_scale_sigma", default_value="0.1")'
            in _text(launch_file)
        )

    cbs_node = _text("cbs_ros/src/cbs_ros_node.cpp")
    assert "double sim3_scale_sigma_{0.1};" in cbs_node

    ground_profile = _text(
        "sb_slam_ros2/launch/"
        "graco_ground_01_02_03_04_05_06_jist_ds_no_aug_no_lg.launch.py"
    )
    assert '"sim3_scale_sigma": "0.1"' in ground_profile


def test_cbs_support_nodes_follow_stage_strategy():
    example_launch = _text("cbs_ros/launch/example.launch.py")
    offline_launch = _text("cbs_ros/launch/offline.launch.py")

    for removed_argument in (
        "run_graph_publisher",
        "run_belief_stage_controller",
    ):
        assert removed_argument not in example_launch
        assert removed_argument not in offline_launch

    assert 'executable="graph_publisher_node"' in example_launch
    assert 'if belief_stage_switch_strategy == "ros_message":' in (
        example_launch
    )
    assert 'executable="belief_stage_controller_node"' in example_launch
    assert (
        '"belief_stage_switch_strategy": belief_stage_switch_strategy'
        in offline_launch
    )
    assert (
        'DeclareLaunchArgument(\n'
        '                "belief_stage_switch_strategy", default_value="random"'
        in offline_launch
    )
    assert (
        '"synchronize_optimization_rounds": '
        "synchronize_optimization_rounds"
        in offline_launch
    )


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


def test_aerial_5678_named_jist_profiles_preserve_experiment_setup():
    reference = _text(
        "sb_slam_ros2/launch/"
        "graco_aerial_05_06_07_08_jist_aug_ds.launch.py"
    )
    for expected in (
        '"bag_rate", default_value="1.0"',
        'default_value="/data/graco/aerial-05-40m"',
        'default_value="/data/graco/aerial-06-20m_stereo_ros2"',
        'default_value="/data/graco/aerial-07-25m_stereo_ros2"',
        'default_value="/data/graco/aerial-08-25m_ros2"',
        '"vio_mode": "stereo"',
        '"vio_dataset_name": LaunchConfiguration("vio_dataset_name")',
        (
            '"distributed_dataset_name": LaunchConfiguration(\n'
            '                "distributed_dataset_name"\n'
            "            )"
        ),
        '"vpr_model_type": LaunchConfiguration("vpr_model_type")',
        '"use_external_odom": "false"',
        "JIST_r18_512_seqgem_frames_fp32.engine",
        "mixvpr_resnet50_512d_fp16_sm120_trt10.13.engine",
        '"loop_closure.max_submap_size": "10"',
        '"loop_closure.max_submap_distance": "5"',
        '"pgo_formulation": "sim3"',
        '"sim3_scale_sigma": "0.05"',
        '"visualization_mode", default_value="minimal"',
    ):
        assert expected in reference

    distributed = _text(
        "Kimera-Distributed/params/"
        "visual_loopclosure_GrAcoJistDynamic.yaml"
    )
    assert "min_sim_vlad: 0.7" in distributed

    fixed_distributed = _text(
        "Kimera-Distributed/params/"
        "visual_loopclosure_GrAcoJistFixed5.yaml"
    )
    assert "dist_local: 90" in fixed_distributed
    assert "min_sim_vlad: 0.7" in fixed_distributed

    dynamic = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/param/"
        "GrAcoStereoXfeatJistAugDs/LcdParams.yaml"
    )
    assert "vpr_seq_interval: 1" in dynamic
    assert "max_covisibility_score: 0.1" in dynamic
    assert "min_sim_score: 0.85" in dynamic
    assert "max_consecutive_frame_covisibility_score: 1.0" in dynamic
    assert "use_covis_projection: 1" in dynamic

    fixed = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/param/"
        "GrAcoStereoXfeatJistAugFixed5/LcdParams.yaml"
    )
    assert "vpr_seq_interval: 1" in fixed
    assert "vpr_short_sequence_policy: wait" in fixed
    assert "max_covisibility_score: 1.0" in fixed
    assert "min_sim_score: -1.0" in fixed
    assert "max_consecutive_frame_covisibility_score: 1.0" in fixed
    assert "use_covis_projection: 1" in fixed

    fixed_noaug = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/param/"
        "GrAcoStereoXfeatJistNoAugFixed5/LcdParams.yaml"
    )
    assert "vpr_seq_interval: 1" in fixed_noaug
    assert "vpr_short_sequence_policy: wait" in fixed_noaug
    assert "max_covisibility_score: 1.0" in fixed_noaug
    assert "min_sim_score: -1.0" in fixed_noaug
    assert "max_consecutive_frame_covisibility_score: 1.0" in fixed_noaug
    assert "use_covis_projection: 0" in fixed_noaug
    ablation = _text(
        "sb_slam_ros2/launch/"
        "graco_aerial_05_06_07_08_jist_aug_fixed5.launch.py"
    )
    assert "GrAcoStereoXfeatJistAugFixed5" in ablation
    assert "GrAcoJistFixed5" in ablation
    assert "graco_aerial_05_06_07_08_jist_aug_ds.launch.py" in ablation

    noaug_ablation = _text(
        "sb_slam_ros2/launch/"
        "graco_aerial_05_06_07_08_jist_no_aug_fixed5.launch.py"
    )
    assert "GrAcoStereoXfeatJistNoAugFixed5" in noaug_ablation
    assert "GrAcoJistFixed5" in noaug_ablation
    assert '"bag_rate": "1.0"' in noaug_ablation
    assert "JIST_r18_512_seqgem_simplified_fp32.engine" in noaug_ablation
    assert "rerun+http://192.168.0.206:9876/proxy" in noaug_ablation
    assert '"start_zenoh_router": "false"' in noaug_ablation

    mixvpr_params = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/param/"
        "GrAcoStereoXfeatMixVprAugNoDynamicI2/LcdParams.yaml"
    )
    assert "vpr_model_type: mixvpr" in mixvpr_params
    assert "vpr_seq_interval: 2" in mixvpr_params
    assert "max_covisibility_score: 1.0" in mixvpr_params
    assert "min_sim_score: -1.0" in mixvpr_params
    assert "max_consecutive_frame_covisibility_score: 1.0" in mixvpr_params
    assert "use_covis_projection: 1" in mixvpr_params

    mixvpr_distributed = _text(
        "Kimera-Distributed/params/"
        "visual_loopclosure_GrAcoMixVprNoDynamic.yaml"
    )
    assert "dist_local: 90" in mixvpr_distributed
    assert "min_sim_vlad: 0.6" in mixvpr_distributed

    mixvpr_launch = _text(
        "sb_slam_ros2/launch/"
        "graco_aerial_05_06_07_08_mixvpr_no_dynamic.launch.py"
    )
    assert "GrAcoStereoXfeatMixVprAugNoDynamicI2" in mixvpr_launch
    assert "GrAcoMixVprNoDynamic" in mixvpr_launch
    assert '"vpr_model_type": "mixvpr"' in mixvpr_launch
    assert "/data3/graco/aerial-05-40m_full_ros2" in mixvpr_launch
    assert "/data3/graco/aerial-08-25m_full_ros2" in mixvpr_launch
    assert "rerun+http://192.168.0.206:9876/proxy" in mixvpr_launch
    assert '"start_zenoh_router": "false"' in mixvpr_launch
    assert "graco_aerial_05_06_07_08_jist_aug_ds.launch.py" in mixvpr_launch

    mixvpr_i5_params = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/param/"
        "GrAcoStereoXfeatMixVprAugNoDynamicI5/LcdParams.yaml"
    )
    assert "vpr_model_type: mixvpr" in mixvpr_i5_params
    assert "vpr_seq_interval: 5" in mixvpr_i5_params
    assert "max_covisibility_score: 1.0" in mixvpr_i5_params
    assert "min_sim_score: -1.0" in mixvpr_i5_params
    assert "max_consecutive_frame_covisibility_score: 1.0" in mixvpr_i5_params
    assert "use_covis_projection: 1" in mixvpr_i5_params

    mixvpr_i5_launch = _text(
        "sb_slam_ros2/launch/"
        "graco_aerial_05_06_07_08_mixvpr_no_dynamic_i5.launch.py"
    )
    assert "GrAcoStereoXfeatMixVprAugNoDynamicI5" in mixvpr_i5_launch
    assert "GrAcoMixVprNoDynamic" in mixvpr_i5_launch
    assert '"vpr_model_type": "mixvpr"' in mixvpr_i5_launch
    assert '"bag_rate": "1.0"' in mixvpr_i5_launch
    assert "interval5" in mixvpr_i5_launch
    assert '"start_zenoh_router": "false"' in mixvpr_i5_launch

    mixvpr_noaug_i5_params = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/param/"
        "GrAcoStereoXfeatMixVprNoAugNoDynamicI5/LcdParams.yaml"
    )
    assert "vpr_model_type: mixvpr" in mixvpr_noaug_i5_params
    assert "vpr_seq_interval: 5" in mixvpr_noaug_i5_params
    assert "max_covisibility_score: 1.0" in mixvpr_noaug_i5_params
    assert "min_sim_score: -1.0" in mixvpr_noaug_i5_params
    assert "max_consecutive_frame_covisibility_score: 1.0" in (
        mixvpr_noaug_i5_params
    )
    assert "use_covis_projection: 0" in mixvpr_noaug_i5_params

    mixvpr_noaug_i5_launch = _text(
        "sb_slam_ros2/launch/"
        "graco_aerial_05_06_07_08_mixvpr_no_aug_no_dynamic_i5.launch.py"
    )
    assert "GrAcoStereoXfeatMixVprNoAugNoDynamicI5" in (
        mixvpr_noaug_i5_launch
    )
    assert "GrAcoMixVprNoDynamic" in mixvpr_noaug_i5_launch
    assert '"vpr_model_type": "mixvpr"' in mixvpr_noaug_i5_launch
    assert '"bag_rate": "1.0"' in mixvpr_noaug_i5_launch
    assert "noaug-no-dynamic-interval5" in mixvpr_noaug_i5_launch
    assert '"start_zenoh_router": "false"' in mixvpr_noaug_i5_launch

    jist_noaug_params = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/param/"
        "GrAcoStereoXfeatJistDsNoAug/LcdParams.yaml"
    )
    assert "vpr_model_type: jist" in jist_noaug_params
    assert "vpr_seq_interval: 1" in jist_noaug_params
    assert "max_covisibility_score: 0.1" in jist_noaug_params
    assert "min_sim_score: 0.85" in jist_noaug_params
    assert "use_covis_projection: 0" in jist_noaug_params

    mixvpr_ds_noaug_params = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/param/"
        "GrAcoStereoXfeatMixVprDsNoAugI2/LcdParams.yaml"
    )
    assert "vpr_model_type: mixvpr" in mixvpr_ds_noaug_params
    assert "vpr_seq_interval: 2" in mixvpr_ds_noaug_params
    assert "max_covisibility_score: 0.1" in mixvpr_ds_noaug_params
    assert "min_sim_score: 0.85" in mixvpr_ds_noaug_params
    assert "use_covis_projection: 0" in mixvpr_ds_noaug_params

    mixvpr_ds_distributed = _text(
        "Kimera-Distributed/params/"
        "visual_loopclosure_GrAcoMixVprDynamic.yaml"
    )
    assert "dist_local: 30" in mixvpr_ds_distributed
    assert "min_sim_vlad: 0.6" in mixvpr_ds_distributed

    for launch_file, profile, distributed_profile, model in (
        (
            "graco_aerial_05_06_07_08_mixvpr_ds_no_aug.launch.py",
            "GrAcoStereoXfeatMixVprDsNoAugI2",
            "GrAcoMixVprDynamic",
            "mixvpr",
        ),
        (
            "graco_aerial_05_06_07_08_jist_ds_no_aug.launch.py",
            "GrAcoStereoXfeatJistDsNoAug",
            "GrAcoJistDynamic",
            "jist",
        ),
    ):
        noaug_launch = _text(f"sb_slam_ros2/launch/{launch_file}")
        assert profile in noaug_launch
        assert distributed_profile in noaug_launch
        assert f'"vpr_model_type": "{model}"' in noaug_launch
        assert "rerun+http://192.168.0.206:9876/proxy" in noaug_launch
        assert '"start_zenoh_router": "false"' in noaug_launch

    jist_noaug_launch = _text(
        "sb_slam_ros2/launch/"
        "graco_aerial_05_06_07_08_jist_ds_no_aug.launch.py"
    )
    assert "JIST_r18_512_seqgem_simplified_fp32.engine" in jist_noaug_launch
    assert "JIST_r18_512_seqgem_simplified_fp16.engine" in jist_noaug_launch
    assert '"models.jist": _default_jist_model_path()' in jist_noaug_launch

    multi_robot = _text(
        "sb_slam_ros2/launch/"
        "graco_aerial_05_06_07_08_multi_robot.launch.py"
    )
    assert '"experiment_setup.yaml"' in multi_robot


def test_mixvpr_interval5_uses_exact_single_frame_stride():
    params = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/param/"
        "GrAcoStereoXfeatMixVprAugNoDynamicI5/LcdParams.yaml"
    )
    assert "vpr_model_type: mixvpr" in params
    assert "vpr_seq_interval: 5" in params
    assert "max_covisibility_score: 1.0" in params
    assert "min_sim_score: -1.0" in params
    assert "use_covis_projection: 1" in params

    launch = _text(
        "sb_slam_ros2/launch/"
        "graco_aerial_05_06_07_08_mixvpr_no_dynamic_i5.launch.py"
    )
    assert "GrAcoStereoXfeatMixVprAugNoDynamicI5" in launch
    assert '"bag_rate": "1.0"' in launch
    assert "interval5" in launch

    detector = _text(
        "Kimera-VIO/src/loopclosure/VLADLoopClosureDetector.cpp"
    )
    assert "vpr_db_->get_seq_length() == 1" in detector
    assert "target_frame_id % vpr_seq_interval != 0u" in detector
    assert "single_frame_sequence" in detector


def test_mixvpr_interval5_noaug_toggles_only_projection():
    aug_params = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/param/"
        "GrAcoStereoXfeatMixVprAugNoDynamicI5/LcdParams.yaml"
    )
    noaug_params = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/param/"
        "GrAcoStereoXfeatMixVprNoAugNoDynamicI5/LcdParams.yaml"
    )

    def settings(text):
        return {
            line
            for line in text.splitlines()
            if line and not line.lstrip().startswith("#")
        }

    assert settings(aug_params) - settings(noaug_params) == {
        "use_covis_projection: 1"
    }
    assert settings(noaug_params) - settings(aug_params) == {
        "use_covis_projection: 0"
    }


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


def test_campus_minimal_visualization_adds_vio_tracking_and_trajectory():
    campus = _text("sb_slam_ros2/launch/campus_six_robot.launch.py")
    assert '"visualization_mode",\n                default_value="minimal"' in campus
    assert 'FindPackageShare("kimera_vio_ros")' in campus
    assert '"param",\n                        "D455"' in campus
    assert '"src", "Kimera-VIO", "params", "D455"' not in campus
    assert "vio_rerun_visualization_profile = visualization_mode" in campus
    assert '"rerun_visualization_profile": (' in campus
    assert '"rerun_tracking_image_jpeg_quality": LaunchConfiguration(' in (
        campus
    )
    assert '"dense_mapping.publisher_enabled": "false"' in campus
    assert '"mono_depth.enabled": "false"' in campus
    assert '"use_external_odom": LaunchConfiguration(' in campus
    assert '"use_external_odom",\n                default_value="true"' in campus
    assert '"topic.external_odom": (' in campus
    assert "JIST_r18_512_seqgem_simplified_fp32.engine" in campus
    assert (
        '"belief_republish_hellinger_threshold", default_value="0.01"'
        in campus
    )
    assert '"pgo_formulation",\n                default_value="sim3"' in campus
    for scale_parameter in (
        "sim3_scale_sigma",
        "sim3_odom_scale_sigma",
        "sim3_loop_scale_sigma",
        "sim3_inter_loop_scale_sigma",
    ):
        assert f'"{scale_parameter}": LaunchConfiguration(' in campus
        assert (
            f'"{scale_parameter}",\n                default_value="1e-6"'
            in campus
        )
    for prior_parameter in (
        "sim3_pose_scale_prior_sigma",
        "sim3_anchor_scale_prior_sigma",
    ):
        assert f'"{prior_parameter}": LaunchConfiguration(' in campus
        assert f'"{prior_parameter}",\n                default_value="-1"' in campus
    assert '"sim3_inter_loop_has_scale_measurement"' in campus
    assert 'default_value="false"' in campus
    assert "sim3_anchor_belief_scale_sigma" not in campus
    assert (
        '"pgo_formulation": LaunchConfiguration("pgo_formulation")' in campus
    )
    assert '"pgo_formulation": "pose3"' not in campus

    for launch_name in (
        "kimera_vio_ros.launch.py",
        "kimera_vio_ros_mono.launch.py",
    ):
        launch = _text(f"Kimera-VIO-ROS2/kimera_vio_ros/launch/{launch_name}")
        assert "'rerun_visualization_profile'" in launch
        assert "'rerun_tracking_image_jpeg_quality'" in launch

    stereo_vio = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/launch/kimera_vio_ros.launch.py"
    )
    assert "'--use_external_odometry='" in stereo_vio
    assert "'use_external_odom': LaunchConfiguration(" in stereo_vio
    assert "('external_odom', LaunchConfiguration('topic.external_odom'))" in (
        stereo_vio
    )

    interface = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/include/"
        "kimera_vio_ros/interfaces/RerunVisualizer.h"
    )
    assert "VisualizationProfile::kMinimal" in interface
    assert "kTrackingImageOnly" not in interface
    assert "kTrackingImageAndTrajectory" not in interface
    assert "rerun::EncodedImage::from_bytes" in interface


def test_aerial_05_06_07_08_is_partitionable_mono_experiment():
    launch = _module(
        "sb_slam_ros2/launch/"
        "graco_aerial_05_06_07_08_multi_robot.launch.py"
    )
    recording_id = launch._timestamped_recording_id()
    assert re.fullmatch(
        r"graco_aerial_05_06_07_08_\d{8}_\d{6}_[+-]\d{4}",
        recording_id,
    )
    assert launch._EXPECTED_ROBOT_NAMES == ("a5", "a6", "a7", "a8")
    assert launch._parse_active_robot_ids("0, 2,3") == (0, 2, 3)
    for invalid in ("", "0,", "0,0", "4", "a5"):
        with pytest.raises(RuntimeError):
            launch._parse_active_robot_ids(invalid)

    launch_text = _text(
        "sb_slam_ros2/launch/"
        "graco_aerial_05_06_07_08_multi_robot.launch.py"
    )
    assert '"num_robots": "4"' in launch_text
    assert '"active_robot_ids"' in launch_text
    assert '"bag_rate", default_value="1.0"' in launch_text
    assert '"bag_start_delay", default_value="20.0"' in launch_text
    assert '"zenoh_router_startup_delay"' in launch_text
    assert (
        'period=LaunchConfiguration("zenoh_router_startup_delay")'
        in launch_text
    )
    assert '"dense_mapping.enabled": "false"' in launch_text
    assert '"keyframe_state.publisher_enabled": "false"' in launch_text
    assert '"visualization_mode", default_value="minimal"' in launch_text
    assert '"play_bag": "false"' in launch_text
    assert '"use_sim_time": "false"' in launch_text
    assert "_SOURCE_IMAGE_TOPIC = \"/camera_left/image_raw\"" in launch_text
    assert "_SOURCE_IMU_TOPIC = \"/gnss/imu\"" in launch_text

    robot_launch = _text("sb_slam_ros2/launch/graco_robot.launch.py")
    assert '"kimera_vio_ros_mono.launch.py"' in robot_launch
    assert '"pgo_formulation": "sim3"' in robot_launch
    assert '"rerun_enabled": detailed_rerun_enabled' in robot_launch
    assert '"use_rerun_visualizer": "true"' in robot_launch
    assert '"rerun_visualization_profile": vio_rerun_profile' in robot_launch
    assert "vio_rerun_profile = visualization_mode" in robot_launch
    assert '"belief_republish_hellinger_threshold": LaunchConfiguration(' in (
        robot_launch
    )


def test_aerial_05_06_07_08_stereo_matches_ros1_experiment_profile():
    launch_text = _text(
        "sb_slam_ros2/launch/"
        "graco_aerial_05_06_07_08_stereo_multi_robot.launch.py"
    )
    for expected in (
        '"vio_mode": "stereo"',
        '"vio_dataset_name": "GrAcoStereoXfeat"',
        '"distributed_dataset_name": "GrAcoStereo"',
        '"vpr_model_type": "mixvpr"',
        '"bag_rate": "1.0"',
        '"loop_closure.alpha": "0.5"',
        '"loop_closure.bow_batch_size": "50"',
        '"loop_closure.vlc_batch_size": "10"',
        '"loop_closure.loop_batch_size": "50"',
        '"loop_closure.loop_sync_sleep_time": "10"',
        '"loop_closure.comm_sleep_time": "5"',
        '"loop_closure.detection_batch_size": "50"',
        '"loop_closure.max_submap_size": "10"',
        '"loop_closure.max_submap_distance": "5"',
        '"loop_closure.adaptive_scoring_tau_max": "0.01"',
        '"loop_closure.adaptive_scoring_tau_min": "0.01"',
        '"loop_closure.adaptive_scoring_lambda": "1.0"',
        '"visualization_mode", default_value="minimal"',
    ):
        assert expected in launch_text

    base = _text(
        "sb_slam_ros2/launch/"
        "graco_aerial_05_06_07_08_multi_robot.launch.py"
    )
    assert '_SOURCE_RIGHT_IMAGE_TOPIC = "/camera_right/image_raw"' in base
    assert '"right_image_topic": (' in base
    assert "required_topics.add(_SOURCE_RIGHT_IMAGE_TOPIC)" in base
    assert '"dense_mapping.enabled": "false"' in base
    assert '"keyframe_state.publisher_enabled": "false"' in base

    robot = _text("sb_slam_ros2/launch/graco_robot.launch.py")
    assert '"kimera_vio_ros.launch.py"' in robot
    assert '"topic.left.image": image_topic' in robot
    assert '"topic.right.image": right_image_topic' in robot
    assert '"models.mixvpr": models["models.mixvpr"]' in robot
    assert (
        'distributed_log_output_path = Path(log_output_path) / "distributed"'
        in robot
    )
    assert '"log_output_path": str(distributed_log_output_path)' in robot
    assert '"log_output_path": log_output_path' in robot

    lcd = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/param/"
        "GrAcoStereoXfeat/LcdParams.yaml"
    )
    assert "vpr_model_type: mixvpr" in lcd
    assert "vpr_seq_interval: 5" in lcd
    assert "vpr_max_sequence_distance_m: 10" in lcd

    distributed = _text(
        "Kimera-Distributed/params/visual_loopclosure_GrAcoStereo.yaml"
    )
    assert "min_sim_vlad: 0.6" in distributed
    assert "scoring_mode" not in distributed
    assert "use_score_combination" not in distributed


def test_aerial_05_06_07_08_jist_dynamic_uses_sequence_local_distance():
    launch_text = _text(
        "sb_slam_ros2/launch/"
        "graco_aerial_05_06_07_08_jist_aug_ds.launch.py"
    )
    assert '"distributed_dataset_name": "GrAcoJistDynamic"' in launch_text

    distributed = _text(
        "Kimera-Distributed/params/"
        "visual_loopclosure_GrAcoJistDynamic.yaml"
    )
    assert "dist_local: 30" in distributed
    assert "min_sim_vlad: 0.7" in distributed


def test_aerial_05_06_07_08_cross_host_wrappers_partition_robots():
    workstation = _text(
        "sb_slam_ros2/scripts/run_graco_aerial_5678_workstation.bash"
    )
    assert "active_robot_ids:=1,2,3" in workstation
    assert "SPLIT_RMW_IMPLEMENTATION:-rmw_fastrtps_cpp" in workstation
    assert "fastdds_workstation_to_jetson.xml" in workstation
    assert "ROS_LOCALHOST_ONLY=0" in workstation
    assert "start_zenoh_router:=false" in workstation
    assert "nvidia-cuda-mps-control" in workstation
    assert "ZENOH_SESSION_CONFIG_URI" in workstation

    jetson = _text(
        "sb_slam_ros2/scripts/jetson/"
        "run_graco_aerial_5678_split_profile.bash"
    )
    assert "active_robot_ids:=0" in jetson
    assert "SPLIT_RMW_IMPLEMENTATION:-rmw_fastrtps_cpp" in jetson
    assert "fastdds_jetson_to_workstation.xml" in jetson
    assert "ROS_LOCALHOST_ONLY=0" in jetson
    assert "start_zenoh_router:=false" in jetson
    assert "ZENOH_SESSION_CONFIG_URI" in jetson
    assert "ZENOH_ROUTER_CONFIG_URI" in jetson
    assert "JIST_r18_512_seqgem_simplified_fp16.engine" in jetson
    assert "sample_ros_processes.py" in jetson
    assert "tegrastats --interval 1000" in jetson

    workstation_zenoh = _text(
        "sb_slam_ros2/config/rmw_zenoh_workstation_router.json5"
    )
    assert '"tcp/127.0.0.1:7447"' in workstation_zenoh
    assert "enabled: false" in workstation_zenoh

    jetson_session = _text(
        "sb_slam_ros2/config/rmw_zenoh_jetson_local_session.json5"
    )
    assert '"tcp/127.0.0.1:7447"' in jetson_session

    jetson_router = _text(
        "sb_slam_ros2/config/"
        "rmw_zenoh_jetson_router_to_workstation.json5"
    )
    assert 'mode: "router"' in jetson_router
    assert '"tcp/192.168.0.220:7447"' in jetson_router

    workstation_fastdds = _text(
        "sb_slam_ros2/config/fastdds_workstation_to_jetson.xml"
    )
    assert "<address>192.168.0.220</address>" in workstation_fastdds
    assert "<address>192.168.0.217</address>" in workstation_fastdds

    jetson_fastdds = _text(
        "sb_slam_ros2/config/fastdds_jetson_to_workstation.xml"
    )
    assert "<address>192.168.0.217</address>" in jetson_fastdds
    assert "<address>192.168.0.220</address>" in jetson_fastdds


def test_cbs_writes_component_timing_for_jetson_benchmarking():
    node = _text("cbs_ros/src/cbs_ros_node.cpp")
    assert '"timing_robot_"' in node
    for field in (
        "update_ms",
        "stats_ms",
        "belief_request_ms",
        "visualization_ms",
        "active_callback_ms",
    ):
        assert field in node
    assert "elapsedMilliseconds(update_start, SteadyClock::now())" in node


def test_jetson_benchmark_isolates_vio_and_disables_dense_mapping():
    launch = _text(
        "sb_slam_ros2/launch/graco_ground_01_jetson_benchmark.launch.py"
    )
    assert '"dataset_name": "GrAcoGndStereoXfeat"' in launch
    assert '"use_lcd": "2"' in launch
    assert '"multi_robot_bridge.enabled": "false"' in launch
    assert '"log_output": "true"' in launch
    assert '"dense_mapping.publisher_enabled": "false"' in launch
    assert '"mono_depth.enabled": "false"' in launch
    assert '"use_rerun_visualizer": "false"' in launch
    assert "FindPackageShare(\"dense_mapping\")" not in launch
    assert "FindPackageShare(\"kimera_distributed\")" not in launch
    assert "FindPackageShare(\"cbs_ros\")" not in launch
    assert 'default_value="20.0"' in launch
    assert "GrAco ground-01 playback completed" in launch


def test_aerial_05_07_records_cbs_and_global_ba_inputs():
    launch = _module(
        "sb_slam_ros2/launch/graco_aerial_05_07_multi_robot.launch.py"
    )
    recording_id = launch._timestamped_recording_id()
    assert re.fullmatch(
        r"graco_aerial_05_07_\d{8}_\d{6}_[+-]\d{4}", recording_id
    )
    assert launch._RECORDED_TOPICS == (
        "/a5/kimera_vio/pose_graph/updates",
        "/a5/kimera_vio/mapping/local_window_poses",
        "/a5/kimera_vio/mapping/keyframes",
        "/a5/kimera_distributed/pose_graph/updates",
        "/a5/kimera_distributed/keyframe_loop_closures",
        "/a7/kimera_vio/pose_graph/updates",
        "/a7/kimera_vio/mapping/local_window_poses",
        "/a7/kimera_vio/mapping/keyframes",
        "/a7/kimera_distributed/pose_graph/updates",
        "/a7/kimera_distributed/keyframe_loop_closures",
    )

    launch_text = _text(
        "sb_slam_ros2/launch/graco_aerial_05_07_multi_robot.launch.py"
    )
    assert '"use_sim_time": "false"' in launch_text
    assert '"bag_publish_clock": "false"' in launch_text
    assert '"record_output", default_value="true"' in launch_text
    assert (
        'on_exit=EmitEvent(\n'
        '            event=Shutdown(reason="A5/A7 output recorder exited")'
        in launch_text
    )
    assert '"loop_closure.alpha", default_value="0.7"' in launch_text
    assert (
        '"loop_closure.adaptive_scoring_tau_max",' in launch_text
    )

    profile = _text("sb_slam_ros2/launch/graco_robot.launch.py")
    assert (
        '"adaptive_scoring_tau_max": LaunchConfiguration(' in profile
    )
    assert (
        '"loop_closure.adaptive_scoring_tau_max",\n'
        '                default_value="0.7"' in profile
    )

    names = _text("Kimera-Distributed/params/robot_names_graco_57.yaml")
    assert names == "robot0_name: a5\nrobot1_name: a7\n"


def test_aerial_05_07_offline_runs_two_local_ba_nodes_with_cbs():
    launch = _module(
        "sb_slam_ros2/launch/"
        "graco_aerial_05_07_global_sparse_ba_offline.launch.py"
    )
    recording_id = launch._timestamped_recording_id()
    assert re.fullmatch(
        r"graco-aerial-05-07-two-ba-offline-"
        r"\d{8}_\d{6}_[+-]\d{4}",
        recording_id,
    )
    assert launch._REPLAY_TOPICS == (
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
    text = _text(
        "sb_slam_ros2/launch/"
        "graco_aerial_05_07_global_sparse_ba_offline.launch.py"
    )
    assert 'ba_nodes = {"a5": _ba_node("a5", 0), "a7": _ba_node("a7", 1)}' in text
    assert text.count('executable="global_sparse_ba_node"') == 1
    assert '"loop_closure.required": False' in text
    assert '"shutdown_after_optimization": True' in text
    assert 'f"experiments/{robot}/global_sparse_ba"' in text
    assert 'package="cbs_ros"' in text
    assert 'reason=f"{robot} CBS node exited"' in text
    assert '"cbs_log_dir",\n            default_value=""' in text
    cbs = _text("cbs_ros/src/cbs_ros_node.cpp")
    assert "if (!log_dir_str.empty())" in cbs
    assert "CBS filesystem logging disabled because log_dir is empty" in cbs
    assert 'expected_return_codes=(0,)' in text
    assert "A5 and A7 global sparse BA completed" in text
    assert "timed out waiting for both global BA nodes" in text
    assert "local_loop_closures" in text
    assert "rerun.timestamp_offset_ns" not in text
    assert "rerun_timestamp_offset_ns" not in text
    assert "rerun_timestamp_offset_ns" not in cbs
    assert "viz_->setTime();" in cbs
    assert "latest_sensor_timestamp_ns_" not in cbs
    assert '"additional_pose_graph_topics": [' in text
    assert '"pose_graph_transient_local": False' in text
    assert "pose_graph_subscriptions_" in cbs


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
    assert '"dense_mapping.enabled": "false"' in launch_text
    assert '"keyframe_state.publisher_enabled": "true"' in launch_text
    assert '"models.da3": LaunchConfiguration("models.da3")' in launch_text
    assert '"graco_robot.launch.py"' in launch_text

    profile = _text("sb_slam_ros2/launch/graco_robot.launch.py")
    assert "distributed_launch" in profile
    assert "cbs_launch" in profile
    assert '"use_lcd": "2"' in profile
    assert '"multi_robot_bridge.enabled": "true"' in profile


def test_graco_ground_01_02_03_matches_ros1_and_disables_dense_mapping():
    launch = _module(
        "sb_slam_ros2/launch/"
        "graco_ground_01_02_03_multi_robot.launch.py"
    )
    recording_id = launch._timestamped_recording_id()
    assert re.fullmatch(
        r"graco_ground_01_02_03_\d{8}_\d{6}_[+-]\d{4}", recording_id
    )
    assert launch._EXPECTED_ROBOT_NAMES == ("g1", "g2", "g3")
    assert launch._BAG_ARGUMENTS == (
        "ground_01_bag_path",
        "ground_02_bag_path",
        "ground_03_bag_path",
    )
    assert launch._SOURCE_LEFT_IMAGE_TOPIC == "/camera_left/image_raw"
    assert launch._SOURCE_RIGHT_IMAGE_TOPIC == "/camera_right/image_raw"
    assert launch._SOURCE_IMU_TOPIC == "/gnss/imu"

    experiment = _text(
        "sb_slam_ros2/launch/"
        "graco_ground_01_02_03_multi_robot.launch.py"
    )
    assert 'default_value="0.8"' in experiment
    assert '"graco_ground_robot.launch.py"' in experiment
    assert '"/data/graco/ground-01"' in experiment
    assert '"/data/graco/ground-02"' in experiment
    assert '"/data/graco/ground-03_ros2"' in experiment
    assert '"xfeat_320x224_fp16.engine"' in experiment
    assert '"lg_320x224_dyn_min1_fp16.engine"' in experiment
    assert "models.xfeat_interp_bilinear" not in experiment
    assert '"--topics"' in experiment
    assert '"--remap"' in experiment
    assert '"--clock"' not in experiment

    profile = _text(
        "sb_slam_ros2/launch/graco_ground_robot.launch.py"
    )
    assert '"dataset_name": "GrAcoGnd"' in profile
    assert '"dataset_name": "GrAcoGndStereoXfeat"' in profile
    assert '"pgo_formulation": "sim3"' in profile
    assert '"belief_republish_hellinger_threshold"' in profile
    assert '"dense_mapping.publisher_enabled": "false"' in profile
    assert '"mono_depth.enabled": "false"' in profile
    assert 'FindPackageShare("dense_mapping")' not in profile

    cbs_launch = _text("cbs_ros/launch/cbs_ros_node.launch.py")
    assert "if additional_pose_graph_topics:" in cbs_launch
    assert (
        'params["additional_pose_graph_topics"] = additional_pose_graph_topics'
        in cbs_launch
    )

    stereo_vio = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/launch/kimera_vio_ros.launch.py"
    )
    assert "'dense_mapping.publisher_enabled'" in stereo_vio
    assert "'dense_mapping.publisher_enabled': LaunchConfiguration(" in (
        stereo_vio
    )


def test_dense_mapping_is_single_robot_only_and_shares_rerun_recording():
    profile = _text("sb_slam_ros2/launch/graco_robot.launch.py")
    assert '"dense_mapping.enabled", default_value="false"' in profile
    assert 'FindPackageShare("dense_mapping")' in profile
    assert 'actions.append(dense_mapping_launch)' in profile
    assert '"keyframe_state.publisher_enabled", default_value="true"' in profile
    assert '"dense_mapping.publisher_enabled": LaunchConfiguration(' in profile
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
    assert "DeclareLaunchArgument('engine_path')" in experiment
    assert "DA3-LARGE-1.1_pose_v2_350x504_fp16.engine" not in experiment
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
    assert "'max_runs_per_submap', default_value='1'" in experiment
    assert "'max_runs_per_submap': LaunchConfiguration(" in experiment
    assert (
        "'geometry_filter.minimum_disparity_px', default_value='100.0'"
        in experiment
    )
    assert (
        "'geometry_filter.visualization_max_disparity_px',"
        "\n            default_value='100.0'"
        in experiment
    )
    assert "'geometry_filter.minimum_disparity_px': LaunchConfiguration(" in (
        experiment
    )
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
    assert "'submap.anchor_method': anchor" in dense_launch
    assert (
        "'odometry_anchored_mapper', 'da3_runs', '', 'odometry', 'da3'"
        in dense_launch
    )
    assert "'submap.overlap_scale_method': 'none'" in dense_launch
    assert "'geometry_filter.enabled': 'false'" in dense_launch
    assert "'geometry_filter.apply_to_mapping': 'false'" in dense_launch
    assert "'geometry_filter.pose_source': 'da3'" in dense_launch
    assert (
        "'geometry_filter.minimum_disparity_px', default_value='10.0'"
        in dense_launch
    )
    assert "'geometry_filter.minimum_disparity_px': LaunchConfiguration(" in (
        dense_launch
    )
    assert "geometry_config.pose_source = GeometryPoseSource::kOdometry" in (
        _text("dense_mapping/src/nodes/odometry_conditioned_da3_node.cpp")
    )
    assert "poseToMessage(da3_context_T_current)" in (
        _text("dense_mapping/src/nodes/odometry_conditioned_da3_node.cpp")
    )
    assert "'max_runs_per_submap', default_value='1'" in dense_launch
    assert "topics.local_window_poses" in dense_launch
    assert "'ros2'," in dense_launch
    assert "'bag'," in dense_launch
    assert "'record'," in dense_launch
    assert "input_bag_record.output_directory" in dense_launch
    mapper_launch = _text("dense_mapping/launch/dense_mapping.launch.py")
    assert "on_exit=EmitEvent(event=Shutdown(" in mapper_launch

    visualizer = _text("dense_mapping/src/visualization/rerun_visualizer.cpp")
    assert "set_time_timestamp_nanos_since_epoch" in visualizer
    assert "std::chrono::system_clock::now()" in visualizer

    vio_visualizer = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/include/kimera_vio_ros/"
        "interfaces/RerunVisualizer.h"
    )
    assert "this->setTime();" in vio_visualizer
    assert "setTimeNSec(static_cast<size_t>(input.timestamp_))" not in vio_visualizer

    aria_visualizer = _text("aria_visualization/src/visualizer_rerun.cpp")
    assert "std::chrono::system_clock::now()" in aria_visualizer
    assert "set_time_timestamp_nanos_since_epoch" in aria_visualizer

    mesh_splat = _text(
        "Kimera-VIO-ROS2/mesh_splat/mesh_splat/mesh_splat_node.py"
    )
    assert "time.time_ns()" in mesh_splat

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
    assert "'max_runs_per_submap', default_value='1'" in offline_launch
    assert "'max_runs_per_submap': LaunchConfiguration(" in offline_launch
    assert (
        "'geometry_filter.minimum_disparity_px', default_value='100.0'"
        in offline_launch
    )
    assert (
        "'geometry_filter.visualization_max_disparity_px',"
        "\n            default_value='100.0'"
        in offline_launch
    )
    assert "'geometry_filter.minimum_disparity_px': LaunchConfiguration(" in (
        offline_launch
    )
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
    assert "expected_return_codes=(0, 124)" in offline_launch
    assert "process_exit_handler(" in offline_launch
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
    for mapper_name in (
        'depth_refined_mapper',
        'odometry_anchored_mapper',
        'da3_chain_mapper',
    ):
        assert mapper_name in dense_launch
    # Every mapper consumes the one pose-scale-adjusted stream or its explicit
    # grid-refined replay.
    assert "'topics.native_da3_runs'" not in dense_launch
    assert "'native_da3_runs'" not in dense_launch
    da3_chain_section = dense_launch.split('da3_chain_mapper =', 1)[1]
    da3_chain_section = da3_chain_section.split(
        'refined_mapper_arguments =', 1
    )[0]
    assert "'da3_chain_mapper', 'da3_runs', 'da3_chain', 'da3', 'da3'" in (
        da3_chain_section
    )
    assert "'unconditioned_da3_runs'" not in da3_chain_section
    assert 'unconditioned_da3_runs' not in dense_launch
    assert 'no_pose' not in dense_launch
    assert dense_launch.count('dense_mapper_include(') == 3
    assert (
        "'comparison.da3_chain.enabled', default_value='false'"
        in dense_launch
    )
    assert "LaunchConfiguration('comparison.da3_chain.enabled')" in (
        dense_launch
    )
    assert "'depth_refined/da3_runs'" in dense_launch
    assert "depth_refined/keyframes" in dense_launch
    assert "'max_runs_per_submap', default_value='1'" in dense_launch
    assert "'max_runs_per_submap': LaunchConfiguration(" in dense_launch
    assert "'input_qos.depth': '1000'" in dense_launch
    assert dense_launch.count('on_exit=EmitEvent(event=Shutdown(') == 3
    assert (
        "'sparse_state_outputs.enabled': 'true' if not output_suffix "
        "else 'false'" in dense_launch
    )
    assert "'submap.metric_scale_method': 'none'" in dense_launch
    assert "'submap.anchor_method': anchor" in dense_launch
    assert "'submap.view_pose_method': view" in dense_launch
    assert "'submap.overlap_scale_method': 'none'" in dense_launch
    assert "'geometry_filter.enabled': 'false'" in dense_launch
    assert "'geometry_filter.apply_to_mapping': 'false'" in dense_launch
    for entity in (
        "'alignments',",
        "'odometry_anchored_da3',",
        "'da3_chain',",
        "'grid_ba',",
    ):
        assert entity in dense_launch
    assert "'da3_chain_mapper', 'da3_runs', 'da3_chain', 'da3', 'da3'" in (
        dense_launch
    )
    assert (
        "'comparison.da3_chain.enabled', default_value='true'"
        in offline_launch
    )
    assert "'comparison.da3_chain.enabled': LaunchConfiguration(" in (
        offline_launch
    )
    assert 'comparison.no_pose_da3' not in offline_launch
    assert "DeclareLaunchArgument('engine_path')" in offline_launch
    assert 'DA3-LARGE-1.1_multiview_v2_350x504_fp16.engine' not in (
        offline_launch
    )
    assert "full_union" not in dense_launch

    visualizer = _text("dense_mapping/src/visualization/rerun_visualizer.cpp")
    assert '"sparse_ba/global"' in visualizer
    assert 'prefix + "/odometry_trajectory"' in visualizer
    assert 'prefix + "/optimized_trajectory"' in visualizer


def test_aerial_5678_vpr_ablation_refinement_contract():
    mixvpr_launch = _text(
        "sb_slam_ros2/launch/"
        "graco_aerial_05_06_07_08_mixvpr_512d_sharedseq5.launch.py"
    )
    endpoint_launch = _text(
        "sb_slam_ros2/launch/"
        "graco_aerial_05_06_07_08_jist_ds_no_aug_no_lg.launch.py"
    )
    refined_launch = _text(
        "sb_slam_ros2/launch/"
        "graco_aerial_05_06_07_08_jist_ds_no_aug_no_lg_framerefine.launch.py"
    )

    assert '"jist_frame_refinement": "false"' in mixvpr_launch
    assert '"jist_frame_refinement": "false"' in endpoint_launch
    assert '"jist_frame_refinement": "true"' in refined_launch
    for launch_text in (endpoint_launch, refined_launch):
        assert "JIST_r18_512_seqgem_frames_fp32.engine" in launch_text
        assert '"loop_closure.min_sim_vlad": "0.7"' in launch_text
        assert '"bag_rate": "1.0"' in launch_text

    sequence_keys = (
        "vpr_seq_interval",
        "vpr_min_sequence_frames",
        "vpr_max_sequence_distance_m",
        "vpr_short_sequence_policy",
        "max_covisibility_score",
        "min_sim_score",
        "min_keyframe_diversity_score",
        "max_consecutive_frame_covisibility_score",
        "publish_only_sequence",
        "use_covis_projection",
    )

    def sequence_contract(profile):
        values = {}
        for line in profile.splitlines():
            stripped = line.strip()
            for key in sequence_keys:
                if stripped.startswith(f"{key}:"):
                    values[key] = stripped.split(":", 1)[1].strip()
        return values

    mixvpr_profile = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/param/"
        "GrAcoStereoXfeatMixVprDsNoAugSharedSeq5/LcdParams.yaml"
    )
    jist_profile = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/param/"
        "GrAcoStereoXfeatJistDsNoAugNoLg/LcdParams.yaml"
    )
    assert sequence_contract(mixvpr_profile) == sequence_contract(jist_profile)

    robot_launch = _text("sb_slam_ros2/launch/graco_robot.launch.py")
    distributed_launch = _text(
        "Kimera-Distributed/launch/"
        "kimera_distributed_loop_closure_ros.launch.py"
    )
    vio_launch = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/launch/kimera_vio_ros.launch.py"
    )
    assert robot_launch.count("jist_frame_refinement") >= 4
    assert distributed_launch.count("jist_frame_refinement") >= 2
    assert vio_launch.count("jist_frame_refinement") >= 2


def test_ground_sequence_diversity_ablation_contract():
    base_interface = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/src/interfaces/base_interface.cpp"
    )
    vio_launch = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/launch/kimera_vio_ros.launch.py"
    )
    robot_launch = _text("sb_slam_ros2/launch/graco_robot.launch.py")
    multi_launch = _text(
        "sb_slam_ros2/launch/"
        "graco_ground_01_02_03_04_05_06_multi_robot.launch.py"
    )
    jist_launch = _text(
        "sb_slam_ros2/launch/"
        "graco_ground_01_02_03_04_05_06_jist_ds_no_aug_no_lg.launch.py"
    )
    ablation_launch = _text(
        "sb_slam_ros2/launch/"
        "graco_ground_01_02_03_04_05_06_jist_ds_no_aug_no_lg_"
        "ffs_seqdiv_off_framerefine.launch.py"
    )
    boundary005_launch = _text(
        "sb_slam_ros2/launch/"
        "graco_ground_01_02_03_04_05_06_jist_ds_no_aug_no_lg_ffs_"
        "boundary005_seqdiv_off_framerefine.launch.py"
    )
    boundary005_params = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/param/"
        "GrAcoGndStereoXfeatJistDsNoAugNoLgBoundary005/LcdParams.yaml"
    )
    mono_boundary005_launch = _text(
        "sb_slam_ros2/launch/"
        "graco_ground_01_02_03_04_05_06_jist_ds_no_aug_no_lg_mono_"
        "boundary005_seqdiv_off_framerefine.launch.py"
    )
    mono_boundary005_lcd = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/param/"
        "GrAcoGndMonoXfeatJistDsNoAugNoLgBoundary005/LcdParams.yaml"
    )
    mono_boundary005_frontend = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/param/"
        "GrAcoGndMonoXfeatJistDsNoAugNoLgBoundary005/FrontendParams.yaml"
    )
    mono_boundary005_pipeline = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/param/"
        "GrAcoGndMonoXfeatJistDsNoAugNoLgBoundary005/PipelineParams.yaml"
    )
    mono_lg_boundary005_launch = _text(
        "sb_slam_ros2/launch/"
        "graco_ground_01_02_03_04_05_06_jist_ds_no_aug_lg_mono_"
        "boundary005_seqdiv_off_framerefine.launch.py"
    )
    mono_lg_boundary005_lcd = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/param/"
        "GrAcoGndMonoXfeatJistDsNoAugLgBoundary005/LcdParams.yaml"
    )
    mono_lg_boundary005_backend = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/param/"
        "GrAcoGndMonoXfeatJistDsNoAugLgBoundary005/BackendParams.yaml"
    )
    mono_lg_boundary005_frontend = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/param/"
        "GrAcoGndMonoXfeatJistDsNoAugLgBoundary005/FrontendParams.yaml"
    )
    mono_lg_boundary005_pipeline = _text(
        "Kimera-VIO-ROS2/kimera_vio_ros/param/"
        "GrAcoGndMonoXfeatJistDsNoAugLgBoundary005/PipelineParams.yaml"
    )

    assert '"loop_closure.min_sim_score", -1.0' in base_interface
    for launch_text in (vio_launch, robot_launch, multi_launch, jist_launch):
        assert launch_text.count("loop_closure.min_sim_score") >= 2
    assert '"loop_closure.min_sim_score": "0.0"' in ablation_launch
    assert '"jist_frame_refinement": "true"' in ablation_launch
    assert '"stereo_depth.method": "FastFoundationStereo"' in ablation_launch
    assert "seqdiv-off" in ablation_launch
    assert '"jist_frame_refinement": "true"' in boundary005_launch
    assert '"loop_closure.min_sim_score": "0.0"' in boundary005_launch
    assert "GrAcoGndStereoXfeatJistDsNoAugNoLgBoundary005" in boundary005_launch
    assert "max_covisibility_score: 0.05" in boundary005_params
    assert '"vio_mode": LaunchConfiguration("vio_mode")' in jist_launch
    assert '"vio_mode": "mono"' in mono_boundary005_launch
    assert '"jist_frame_refinement": "true"' in mono_boundary005_launch
    assert '"stereo_depth.method": ""' in mono_boundary005_launch
    assert "GrAcoGndMonoXfeatJistDsNoAugNoLgBoundary005" in mono_boundary005_launch
    assert "max_covisibility_score: 0.05" in mono_boundary005_lcd
    assert "desc_tracking_mode: 0" in mono_boundary005_frontend
    assert "publish_only_sequence: true" in mono_boundary005_frontend
    assert "frontend_type: 0" in mono_boundary005_pipeline
    assert '"vio_mode": "mono"' in mono_lg_boundary005_launch
    assert '"jist_frame_refinement": "true"' in mono_lg_boundary005_launch
    assert '"stereo_depth.method": ""' in mono_lg_boundary005_launch
    assert "GrAcoGndMonoXfeatJistDsNoAugLgBoundary005" in mono_lg_boundary005_launch
    assert "max_covisibility_score: 0.05" in mono_lg_boundary005_lcd
    assert "nr_states: 100" in mono_lg_boundary005_backend
    assert "desc_tracking_mode: 2" in mono_lg_boundary005_frontend
    assert "publish_only_sequence: true" in mono_lg_boundary005_frontend
    assert "frontend_type: 0" in mono_lg_boundary005_pipeline
