from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    map_path = LaunchConfiguration("map_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("lidar_map_localization"),
                        "config",
                        "map_localizer.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "map_path",
                default_value=PathJoinSubstitution(
                    [
                        EnvironmentVariable(
                            "RC_LIDAR_WORKSPACE",
                            default_value="/home/orangepi/RC_LIDAR_NOW!!!",
                        ),
                        "maps",
                        "mid360_map.pcd",
                    ]
                ),
            ),
            Node(
                package="lidar_map_localization",
                executable="map_localizer",
                name="map_localizer",
                output="screen",
                parameters=[
                    params_file,
                    {"map_path": map_path},
                ],
            ),
        ]
    )
