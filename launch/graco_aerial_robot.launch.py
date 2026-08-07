"""Launch one GrAco aerial robot stack without experiment orchestration.

This is the reusable aerial robot primitive.  It launches exactly one
namespaced VIO, distributed loop-closure, and CBS stack through
``graco_robot.launch.py``.  Bag playback, the Zenoh router, run manifests, and
whole-experiment shutdown policy belong to the single- and multi-robot parent
launches.

The parent must provide the robot identity, topology, topics, model/profile
arguments, and output locations.  Keeping playback out of this file makes the
same robot stack safe to instantiate once or many times.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("sb_slam_ros2"),
                    "launch",
                    "graco_robot.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_id": LaunchConfiguration("robot_id"),
            "robot_name": LaunchConfiguration("robot_name"),
            "num_robots": LaunchConfiguration("num_robots"),
            "robot_names_file": LaunchConfiguration("robot_names_file"),
            "image_topic": LaunchConfiguration("image_topic"),
            "right_image_topic": LaunchConfiguration("right_image_topic"),
            "imu_topic": LaunchConfiguration("imu_topic"),
            # Aerial experiment parents own bag playback and global services.
            "play_bag": "false",
            "bag_publish_clock": "false",
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "start_zenoh_router": "false",
            "dense_mapping.enabled": LaunchConfiguration(
                "dense_mapping.enabled"
            ),
            "keyframe_state.publisher_enabled": LaunchConfiguration(
                "keyframe_state.publisher_enabled"
            ),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_id", default_value="0"),
            DeclareLaunchArgument("robot_name", default_value="a5"),
            DeclareLaunchArgument("num_robots", default_value="4"),
            DeclareLaunchArgument(
                "robot_names_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("kimera_distributed"),
                        "params",
                        "robot_names_graco.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value=[
                    "/",
                    LaunchConfiguration("robot_name"),
                    "/camera_left/image_raw",
                ],
            ),
            DeclareLaunchArgument(
                "right_image_topic",
                default_value=[
                    "/",
                    LaunchConfiguration("robot_name"),
                    "/camera_right/image_raw",
                ],
            ),
            DeclareLaunchArgument(
                "imu_topic",
                default_value=[
                    "/",
                    LaunchConfiguration("robot_name"),
                    "/gnss/imu",
                ],
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "dense_mapping.enabled", default_value="false"
            ),
            DeclareLaunchArgument(
                "keyframe_state.publisher_enabled", default_value="false"
            ),
            robot_stack,
        ]
    )
