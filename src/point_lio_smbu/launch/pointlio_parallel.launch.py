from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    point_lio_params_file = LaunchConfiguration("point_lio_params_file")
    map_localizer_params_file = LaunchConfiguration("map_localizer_params_file")
    map_path = LaunchConfiguration("map_path")

    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
        ("aft_mapped_to_init", "Odometry"),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="pointlio",
                description="Namespace for the Point-LIO comparison chain.",
            ),
            DeclareLaunchArgument(
                "point_lio_params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("point_lio"),
                        "config",
                        "mid360_parallel.yaml",
                    ]
                ),
                description="Point-LIO MID360 parallel comparison config.",
            ),
            DeclareLaunchArgument(
                "map_localizer_params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("lidar_map_localization"),
                        "config",
                        "map_localizer_pointlio.yaml",
                    ]
                ),
                description="Point-LIO map localizer comparison config.",
            ),
            DeclareLaunchArgument(
                "map_path",
                default_value=PathJoinSubstitution(
                    [
                        EnvironmentVariable("RC_LIDAR_WORKSPACE", default_value="."),
                        "maps",
                        "mid360_map_20260625_005422.pcd",
                    ]
                ),
                description="Shared PCD map used by the Point-LIO map localizer.",
            ),
            Node(
                package="point_lio",
                executable="pointlio_mapping",
                namespace=namespace,
                parameters=[point_lio_params_file],
                remappings=remappings,
                output="screen",
            ),
            Node(
                package="lidar_map_localization",
                executable="map_localizer",
                namespace=namespace,
                name="map_localizer",
                parameters=[
                    map_localizer_params_file,
                    {"map_path": map_path},
                ],
                output="screen",
            ),
            Node(
                package="lidar_localization",
                executable="continuous_robot_pose",
                namespace=namespace,
                name="continuous_robot_pose",
                parameters=[
                    {
                        "fast_odom_topic": "/pointlio/Odometry",
                        "map_odom_topic": "/pointlio/map_lidar/odom",
                        "quality_topic": "/pointlio/map_lidar/quality",
                        "output_pose_topic": "/pointlio/robot/pose_continuous",
                        "output_odom_topic": "/pointlio/robot/odom_continuous",
                        "status_topic": "/pointlio/robot/pose_continuous/status",
                    }
                ],
                output="screen",
            ),
        ]
    )
