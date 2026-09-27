from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    map_path = LaunchConfiguration("map_path")
    auto_initialize_identity = LaunchConfiguration("auto_initialize_identity")
    initial_guess_enabled = LaunchConfiguration("initial_guess_enabled")
    initial_guess_x = LaunchConfiguration("initial_guess_x")
    initial_guess_y = LaunchConfiguration("initial_guess_y")
    initial_guess_z = LaunchConfiguration("initial_guess_z")
    initial_guess_roll = LaunchConfiguration("initial_guess_roll")
    initial_guess_pitch = LaunchConfiguration("initial_guess_pitch")
    initial_guess_yaw = LaunchConfiguration("initial_guess_yaw")
    quality_overlap_enabled = LaunchConfiguration("quality_overlap_enabled")
    quality_overlap_distance = LaunchConfiguration("quality_overlap_distance")
    quality_overlap_period_sec = LaunchConfiguration("quality_overlap_period_sec")
    quality_overlap_max_points = LaunchConfiguration("quality_overlap_max_points")
    timestamp_mismatch_warn_sec = LaunchConfiguration("timestamp_mismatch_warn_sec")
    timestamp_mismatch_log_period_sec = LaunchConfiguration(
        "timestamp_mismatch_log_period_sec"
    )
    input_queue_depth = LaunchConfiguration("input_queue_depth")
    executor_threads = LaunchConfiguration("executor_threads")
    odom_buffer_duration_sec = LaunchConfiguration("odom_buffer_duration_sec")
    max_scan_odom_delta_sec = LaunchConfiguration("max_scan_odom_delta_sec")
    max_scan_age_sec = LaunchConfiguration("max_scan_age_sec")
    localize_period_sec = LaunchConfiguration("localize_period_sec")
    map_leaf_size = LaunchConfiguration("map_leaf_size")
    scan_leaf_size = LaunchConfiguration("scan_leaf_size")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
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
                            default_value=".",
                        ),
                        "maps",
                        "mid360_map.pcd",
                    ]
                ),
            ),
            DeclareLaunchArgument("auto_initialize_identity", default_value="false"),
            DeclareLaunchArgument("initial_guess_enabled", default_value="false"),
            DeclareLaunchArgument("initial_guess_x", default_value="0.0"),
            DeclareLaunchArgument("initial_guess_y", default_value="0.0"),
            DeclareLaunchArgument("initial_guess_z", default_value="0.0"),
            DeclareLaunchArgument("initial_guess_roll", default_value="0.0"),
            DeclareLaunchArgument("initial_guess_pitch", default_value="0.0"),
            DeclareLaunchArgument("initial_guess_yaw", default_value="0.0"),
            DeclareLaunchArgument("quality_overlap_enabled", default_value="false"),
            DeclareLaunchArgument("quality_overlap_distance", default_value="0.20"),
            DeclareLaunchArgument("quality_overlap_period_sec", default_value="1.0"),
            DeclareLaunchArgument("quality_overlap_max_points", default_value="1000"),
            DeclareLaunchArgument("timestamp_mismatch_warn_sec", default_value="0.03"),
            DeclareLaunchArgument(
                "timestamp_mismatch_log_period_sec", default_value="2.0"
            ),
            DeclareLaunchArgument("input_queue_depth", default_value="1"),
            DeclareLaunchArgument("executor_threads", default_value="3"),
            DeclareLaunchArgument("odom_buffer_duration_sec", default_value="2.0"),
            DeclareLaunchArgument("max_scan_odom_delta_sec", default_value="0.05"),
            DeclareLaunchArgument("max_scan_age_sec", default_value="0.5"),
            DeclareLaunchArgument("localize_period_sec", default_value="0.1"),
            DeclareLaunchArgument("map_leaf_size", default_value="0.10"),
            DeclareLaunchArgument("scan_leaf_size", default_value="0.10"),
            Node(
                package="lidar_map_localization",
                executable="map_localizer",
                name="map_localizer",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "map_path": map_path,
                        "auto_initialize_identity": ParameterValue(
                            auto_initialize_identity, value_type=bool
                        ),
                        "initial_guess_enabled": ParameterValue(
                            initial_guess_enabled, value_type=bool
                        ),
                        "initial_guess_x": ParameterValue(
                            initial_guess_x, value_type=float
                        ),
                        "initial_guess_y": ParameterValue(
                            initial_guess_y, value_type=float
                        ),
                        "initial_guess_z": ParameterValue(
                            initial_guess_z, value_type=float
                        ),
                        "initial_guess_roll": ParameterValue(
                            initial_guess_roll, value_type=float
                        ),
                        "initial_guess_pitch": ParameterValue(
                            initial_guess_pitch, value_type=float
                        ),
                        "initial_guess_yaw": ParameterValue(
                            initial_guess_yaw, value_type=float
                        ),
                        "quality_overlap_enabled": ParameterValue(
                            quality_overlap_enabled, value_type=bool
                        ),
                        "quality_overlap_distance": ParameterValue(
                            quality_overlap_distance, value_type=float
                        ),
                        "quality_overlap_period_sec": ParameterValue(
                            quality_overlap_period_sec, value_type=float
                        ),
                        "quality_overlap_max_points": ParameterValue(
                            quality_overlap_max_points, value_type=int
                        ),
                        "timestamp_mismatch_warn_sec": ParameterValue(
                            timestamp_mismatch_warn_sec, value_type=float
                        ),
                        "timestamp_mismatch_log_period_sec": ParameterValue(
                            timestamp_mismatch_log_period_sec, value_type=float
                        ),
                        "input_queue_depth": ParameterValue(
                            input_queue_depth, value_type=int
                        ),
                        "executor_threads": ParameterValue(
                            executor_threads, value_type=int
                        ),
                        "odom_buffer_duration_sec": ParameterValue(
                            odom_buffer_duration_sec, value_type=float
                        ),
                        "max_scan_odom_delta_sec": ParameterValue(
                            max_scan_odom_delta_sec, value_type=float
                        ),
                        "max_scan_age_sec": ParameterValue(
                            max_scan_age_sec, value_type=float
                        ),
                        "localize_period_sec": ParameterValue(
                            localize_period_sec, value_type=float
                        ),
                        "map_leaf_size": ParameterValue(
                            map_leaf_size, value_type=float
                        ),
                        "scan_leaf_size": ParameterValue(
                            scan_leaf_size, value_type=float
                        ),
                    },
                ],
            ),
        ]
    )
