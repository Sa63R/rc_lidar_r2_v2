from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    share = Path(get_package_share_directory('task1_fusion'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share/'config/fusion.yaml')),
        DeclareLaunchArgument('start_can', default_value='false'),
        DeclareLaunchArgument('protocol', default_value=str(share/'config/can_body_twist.example.yaml')),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('record', default_value='true'),
        DeclareLaunchArgument('csv', default_value='results/real_run.csv'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_task3', default_value='false'),
        DeclareLaunchArgument('task3_config', default_value=str(share/'config/task3_corner.yaml')),
        Node(package='task1_fusion', executable='fusion', output='screen',
             parameters=[LaunchConfiguration('config'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
        Node(package='task1_fusion', executable='can_bridge', output='screen',
             condition=IfCondition(LaunchConfiguration('start_can')),
             parameters=[{'protocol_file': LaunchConfiguration('protocol'),
                          'can_interface': LaunchConfiguration('can_interface')}]),
        Node(package='task1_fusion', executable='logger', output='screen',
             condition=IfCondition(LaunchConfiguration('record')),
             parameters=[{'output_file': LaunchConfiguration('csv'),
                          'use_sim_time': LaunchConfiguration('use_sim_time')}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share/'launch/task3.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_task3')),
            launch_arguments={
                'input_mode': 'fused', 'corner_config': LaunchConfiguration('task3_config'),
                'fusion_config': LaunchConfiguration('config'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items()),
    ])
