# Working experiment setups

This file records completed, reusable experiment configurations. A setup being
listed here means that its full bags completed without a node crash and that
its outputs were preserved; it does not by itself certify every loop as an
inlier or make the setup a final benchmark configuration.

## GrAco aerial A5/A6/A7/A8 — JIST, no augmentation, LightGlue, fixed five

Tested on `192.168.0.148` on 2026-07-30.

### Configuration

| Setting | Value |
| --- | --- |
| ROS / middleware | ROS 2 Jazzy / Zenoh |
| Bags | Full A5, A6, A7, and A8 ROS 2 bags |
| Playback | `1.0x` |
| VIO | Stereo, external odometry disabled |
| VPR | JIST TensorRT FP32 |
| JIST similarity threshold | `0.7` |
| Sequence construction | Fixed five admitted keyframes |
| Sequence interval | `1` |
| Anchor boundary threshold | `max_covisibility_score: 1.0` |
| Keyframe-to-keyframe covisibility gate | Disabled with `max_consecutive_frame_covisibility_score: 1.0` |
| Per-sequence diversity filter | Disabled with `min_sim_score: 0.0` |
| Per-keyframe diversity filter | Disabled with `min_keyframe_diversity_score: 0.0` |
| Frontend rematching | LightGlue enabled: `kf_queue_size: 10`, `desc_tracking_mode: 2` |
| Landmark projection / augmentation | Disabled: `use_covis_projection: 0` |
| Local loop exclusion | `dist_local: 90` |
| CBS | Sim3, `sim3_scale_sigma: 0.05` |
| Visualization | Existing Rerun proxy, minimal profile |

With the current sequence implementation, the anchor threshold of `1.0`
marks the boundary immediately after the second admitted frame. The JIST
`wait` short-sequence policy then keeps collecting until the five-frame model
input is full and finalizes the sequence. The consecutive-frame threshold of
`1.0` rejects no frame, so this is the fixed-five behavior rather than a
dynamic covisibility sequence.

Relevant files:

- Launch: [`graco_aerial_05_06_07_08_jist_no_aug_fixed5.launch.py`](../launch/graco_aerial_05_06_07_08_jist_no_aug_fixed5.launch.py)
- VIO/LCD profile: [`GrAcoStereoXfeatJistNoAugFixed5`](../../Kimera-VIO-ROS2/kimera_vio_ros/param/GrAcoStereoXfeatJistNoAugFixed5)
- Distributed LCD profile: [`visual_loopclosure_GrAcoJistFixed5.yaml`](../../Kimera-Distributed/params/visual_loopclosure_GrAcoJistFixed5.yaml)

Run from the workspace root after ensuring CUDA MPS and the Zenoh router are
already active:

```bash
pixi run --frozen -e server-148-jazzy ros2 launch sb_slam_ros2 \
  graco_aerial_05_06_07_08_jist_no_aug_fixed5.launch.py
```

### Completed run

Run ID:

```text
jist-noaug-lgrematch-fixed5-jist0.7-diversity-off-distlocal90-sim3-20260730_133254_+0300
```

Results:

- Remote: `/home/mikexyl/workspaces/sb_slam_ros2/src/code-logs/a5678/jist-noaug-lgrematch-fixed5-jist0.7-diversity-off-distlocal90-sim3-20260730_133254_+0300`
- Local: `/home/mikexyl/workspaces/kimera_noetic_ws/src/code-logs/a5678/jist-noaug-lgrematch-fixed5-jist0.7-diversity-off-distlocal90-sim3-20260730_133254_+0300`
- Result size: approximately 31 MB.
- All four bags completed and all VIO, LCD, and CBS nodes shut down cleanly.
- CBS TUM and G2O snapshots exist for all four robots.
- The four `loop_closures.csv` files contain 217 unique endpoint pairs: 59
  intra-robot and 158 inter-robot. This is a logging count, not an outlier
  classification.

The otherwise equivalent keyframe-to-keyframe covisibility `0.9` run diverged
and is intentionally not listed as a working setup.
