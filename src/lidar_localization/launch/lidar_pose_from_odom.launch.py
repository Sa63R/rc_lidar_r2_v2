from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    input_odom_topic = LaunchConfiguration("input_odom_topic")
    zero_on_first_pose = LaunchConfiguration("zero_on_first_pose")
    zero_delay_sec = LaunchConfiguration("zero_delay_sec")
    zero_frame_id = LaunchConfiguration("zero_frame_id")
    reference_pose_enabled = LaunchConfiguration("reference_pose_enabled")
    reference_frame_id = LaunchConfiguration("reference_frame_id")
    output_stamp_mode = LaunchConfiguration("output_stamp_mode")

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
            DeclareLaunchArgument("input_odom_topic", default_value="/Odometry"),
            DeclareLaunchArgument("zero_on_first_pose", default_value="true"),
            DeclareLaunchArgument("zero_delay_sec", default_value="5.0"),
            DeclareLaunchArgument("zero_frame_id", default_value="robot_origin"),
            DeclareLaunchArgument("reference_pose_enabled", default_value="true"),
            DeclareLaunchArgument("reference_frame_id", default_value="robot_origin"),
            DeclareLaunchArgument("output_stamp_mode", default_value="now"),
            Node(
                package="lidar_localization",
                executable="pose_from_odom",
                name="lidar_pose_from_odom",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "input_odom_topic": input_odom_topic,
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
                        "output_stamp_mode": output_stamp_mode,
                    },
                ],
            ),
        ]
    )
