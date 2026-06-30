from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("rc_nav2_bringup"))
    default_params = package_share / "config" / "fake_nav_params.yaml"
    default_map = package_share / "maps" / "mid360_064655_nav2_low.yaml"

    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_x = LaunchConfiguration("start_x")
    start_y = LaunchConfiguration("start_y")
    start_yaw = LaunchConfiguration("start_yaw")

    lifecycle_nodes = [
        "map_server",
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=str(default_map)),
            DeclareLaunchArgument("params_file", default_value=str(default_params)),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_x", default_value="10.0"),
            DeclareLaunchArgument("start_y", default_value="0.0"),
            DeclareLaunchArgument("start_yaw", default_value="0.0"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_map_to_odom",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
                output="screen",
            ),
            Node(
                package="rc_nav2_bringup",
                executable="fake_diff_drive.py",
                name="fake_diff_drive",
                output="screen",
                parameters=[
                    {
                        "start_x": start_x,
                        "start_y": start_y,
                        "start_yaw": start_yaw,
                        "cmd_vel_topic": "/cmd_vel",
                        "odom_topic": "/odom",
                        "odom_frame": "odom",
                        "base_frame": "base_link",
                        "publish_rate": 50.0,
                        "max_linear_speed": 2.5,
                        "max_lateral_speed": 2.5,
                        "max_angular_speed": 3.14,
                        "body_marker_topic": "/fake_robot_body",
                        "body_length": 0.70,
                        "body_width": 0.80,
                        "body_height": 0.08,
                    }
                ],
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "yaml_filename": map_file,
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "autostart": True,
                        "node_names": lifecycle_nodes,
                    }
                ],
            ),
        ]
    )
