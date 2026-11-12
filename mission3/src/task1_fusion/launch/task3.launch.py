from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from task1_fusion.task3_config import load_config, fused_overrides, prepare


def corner_node(context):
    config = LaunchConfiguration('corner_config').perform(context)
    mode = LaunchConfiguration('input_mode').perform(context)
    overrides = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    corner_cfg = load_config(config)['task3_corner']['ros__parameters']
    prepare(corner_cfg)
    if mode == 'fused':
        fusion_path = LaunchConfiguration('fusion_config').perform(context)
        with open(fusion_path, encoding='utf-8') as file:
            cfg = yaml.safe_load(file)['task1_fusion']['ros__parameters']
        overrides.update(fused_overrides(corner_cfg, cfg))
    elif mode != 'lidar':
        raise ValueError('input_mode must be lidar or fused')
    parent = corner_cfg.get('output_frame') or overrides.get(
        'input_parent_frame', corner_cfg.get('input_parent_frame', 'map'))
    return [Node(package='task1_fusion', executable='corner', output='screen',
                 parameters=[config, overrides]),
            Node(package='task1_fusion', executable='vision', output='screen',
                 condition=IfCondition(LaunchConfiguration('start_vision')),
                 parameters=[config, {
                     'input_topic': corner_cfg.get('output_prefix', '/task3/corner').rstrip('/')+'/odom',
                     'input_parent_frame': parent,
                     'input_child_frame': corner_cfg.get('output_child_frame', 'task3_corner'),
                     'use_sim_time': LaunchConfiguration('use_sim_time'),
                 }])]


def generate_launch_description():
    share = Path(get_package_share_directory('task1_fusion'))
    return LaunchDescription([
        DeclareLaunchArgument('corner_config', default_value=str(share/'config/task3_corner.yaml')),
        DeclareLaunchArgument('input_mode', default_value='lidar', choices=['lidar', 'fused']),
        DeclareLaunchArgument('fusion_config', default_value=str(share/'config/fusion.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_vision', default_value='true'),
        OpaqueFunction(function=corner_node),
    ])
