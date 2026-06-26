from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    node_name = LaunchConfiguration("node_name")
    params_file = LaunchConfiguration("params_file")
    input_odom_topic = LaunchConfiguration("input_odom_topic")
    output_pose_topic = LaunchConfiguration("output_pose_topic")
    output_pose_alias_topic = LaunchConfiguration("output_pose_alias_topic")
    output_point_topic = LaunchConfiguration("output_point_topic")
    output_odom_topic = LaunchConfiguration("output_odom_topic")
    zero_on_first_pose = LaunchConfiguration("zero_on_first_pose")
    zero_delay_sec = LaunchConfiguration("zero_delay_sec")
    zero_frame_id = LaunchConfiguration("zero_frame_id")
    reference_pose_enabled = LaunchConfiguration("reference_pose_enabled")
    reference_frame_id = LaunchConfiguration("reference_frame_id")
    reference_pose_xyz = LaunchConfiguration("reference_pose_xyz")
    reference_pose_rpy = LaunchConfiguration("reference_pose_rpy")
    output_stamp_mode = LaunchConfiguration("output_stamp_mode")
    yaw_fusion_enabled = LaunchConfiguration("yaw_fusion_enabled")
    fast_odom_topic = LaunchConfiguration("fast_odom_topic")
    yaw_fusion_publish_on_fast_odom = LaunchConfiguration("yaw_fusion_publish_on_fast_odom")
    yaw_fusion_max_fast_age_sec = LaunchConfiguration("yaw_fusion_max_fast_age_sec")
    yaw_fusion_map_correction_alpha = LaunchConfiguration("yaw_fusion_map_correction_alpha")
    yaw_fusion_status_topic = LaunchConfiguration("yaw_fusion_status_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("lidar_localization"),
                        "config",
                        "lidar_pose_interface.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("node_name", default_value="lidar_pose_from_odom"),
            DeclareLaunchArgument("input_odom_topic", default_value="/Odometry"),
            DeclareLaunchArgument("output_pose_topic", default_value="/lidar/pose"),
            DeclareLaunchArgument("output_pose_alias_topic", default_value="/robot/pose"),
            DeclareLaunchArgument("output_point_topic", default_value="/lidar/point"),
            DeclareLaunchArgument("output_odom_topic", default_value="/lidar/odom"),
            DeclareLaunchArgument("zero_on_first_pose", default_value="true"),
            DeclareLaunchArgument("zero_delay_sec", default_value="5.0"),
            DeclareLaunchArgument("zero_frame_id", default_value="robot_origin"),
            DeclareLaunchArgument("reference_pose_enabled", default_value="true"),
            DeclareLaunchArgument("reference_frame_id", default_value="robot_origin"),
            DeclareLaunchArgument(
                "reference_pose_xyz", default_value="[0.237488, -0.272804, -0.483]"
            ),
            DeclareLaunchArgument(
                "reference_pose_rpy",
                default_value="[0.0, 0.0, -1.5707963267948966]",
            ),
            DeclareLaunchArgument("output_stamp_mode", default_value="now"),
            DeclareLaunchArgument("yaw_fusion_enabled", default_value="false"),
            DeclareLaunchArgument("fast_odom_topic", default_value="/Odometry"),
            DeclareLaunchArgument("yaw_fusion_publish_on_fast_odom", default_value="true"),
            DeclareLaunchArgument("yaw_fusion_max_fast_age_sec", default_value="0.50"),
            DeclareLaunchArgument("yaw_fusion_map_correction_alpha", default_value="0.05"),
            DeclareLaunchArgument(
                "yaw_fusion_status_topic", default_value="/robot/yaw_fusion/status"
            ),
            Node(
                package="lidar_localization",
                executable="pose_from_odom",
                name=node_name,
                output="screen",
                parameters=[
                    params_file,
                    {
                        "input_odom_topic": input_odom_topic,
                        "output_pose_topic": output_pose_topic,
                        "output_pose_alias_topic": output_pose_alias_topic,
                        "output_point_topic": output_point_topic,
                        "output_odom_topic": output_odom_topic,
                        "zero_on_first_pose": ParameterValue(
                            zero_on_first_pose, value_type=bool
                        ),
                        "zero_delay_sec": ParameterValue(
                            zero_delay_sec, value_type=float
                        ),
                        "zero_frame_id": zero_frame_id,
                        "reference_pose_enabled": ParameterValue(
                            reference_pose_enabled, value_type=bool
                        ),
                        "reference_frame_id": reference_frame_id,
                        "reference_pose_xyz": ParameterValue(
                            reference_pose_xyz, value_type=List[float]
                        ),
                        "reference_pose_rpy": ParameterValue(
                            reference_pose_rpy, value_type=List[float]
                        ),
                        "output_stamp_mode": output_stamp_mode,
                        "yaw_fusion_enabled": ParameterValue(
                            yaw_fusion_enabled, value_type=bool
                        ),
                        "fast_odom_topic": fast_odom_topic,
                        "yaw_fusion_publish_on_fast_odom": ParameterValue(
                            yaw_fusion_publish_on_fast_odom, value_type=bool
                        ),
                        "yaw_fusion_max_fast_age_sec": ParameterValue(
                            yaw_fusion_max_fast_age_sec, value_type=float
                        ),
                        "yaw_fusion_map_correction_alpha": ParameterValue(
                            yaw_fusion_map_correction_alpha, value_type=float
                        ),
                        "yaw_fusion_status_topic": yaw_fusion_status_topic,
                    },
                ],
            ),
        ]
    )
