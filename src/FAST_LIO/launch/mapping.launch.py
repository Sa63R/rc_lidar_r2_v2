import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


def _optional_float_override(context, launch_arg, parameter_name):
    value = LaunchConfiguration(launch_arg).perform(context).strip()
    if not value:
        return None
    try:
        return parameter_name, float(value)
    except ValueError as exc:
        raise RuntimeError(f"{launch_arg} must be a float, got {value!r}") from exc


def _launch_fast_lio_node(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    map_file_path = LaunchConfiguration('map_file_path')

    parameters = [
        PathJoinSubstitution([config_path, config_file]),
        {
            'use_sim_time': use_sim_time,
            'map_file_path': map_file_path,
        },
    ]

    overrides = {}
    for launch_arg, parameter_name in (
        ('filter_size_surf', 'filter_size_surf'),
        ('filter_size_map', 'filter_size_map'),
        ('det_range', 'mapping.det_range'),
    ):
        override = _optional_float_override(context, launch_arg, parameter_name)
        if override is not None:
            key, value = override
            overrides[key] = value

    if overrides:
        parameters.append(overrides)

    return [
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            parameters=parameters,
            output='screen'
        )
    ]


def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    decalre_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )
    declare_map_file_path_cmd = DeclareLaunchArgument(
        'map_file_path',
        default_value=PathJoinSubstitution([
            EnvironmentVariable(
                'RC_LIDAR_WORKSPACE',
                default_value='.',
            ),
            'maps',
            'mid360_map.pcd',
        ]),
        description='PCD path used by the /map_save service'
    )
    declare_filter_size_surf_cmd = DeclareLaunchArgument(
        'filter_size_surf',
        default_value='',
        description='Optional override for FAST-LIO filter_size_surf'
    )
    declare_filter_size_map_cmd = DeclareLaunchArgument(
        'filter_size_map',
        default_value='',
        description='Optional override for FAST-LIO filter_size_map'
    )
    declare_det_range_cmd = DeclareLaunchArgument(
        'det_range',
        default_value='',
        description='Optional override for FAST-LIO mapping.det_range'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(decalre_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)
    ld.add_action(declare_map_file_path_cmd)
    ld.add_action(declare_filter_size_surf_cmd)
    ld.add_action(declare_filter_size_map_cmd)
    ld.add_action(declare_det_range_cmd)

    ld.add_action(OpaqueFunction(function=_launch_fast_lio_node))
    ld.add_action(rviz_node)

    return ld
