"""Driver-free FAST-LIO playback; use a bag recorded with point times and IMU."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    mode = value('mode')
    if mode not in ('mapping', 'localization'):
        raise RuntimeError('mode must be mapping or localization')
    config = Path(value('config')).expanduser().resolve()
    map_path = Path(value('map_path')).expanduser().resolve()
    if not config.is_file():
        raise RuntimeError(f'FAST-LIO config missing: {config}')
    if mode == 'localization' and not map_path.is_file():
        raise RuntimeError(f'Build/save the matching map first: {map_path}')
    common = {'use_sim_time': True}
    nodes = [Node(
        package='fast_lio', executable='fastlio_mapping', output='screen',
        parameters=[str(config), common, {
            'map_file_path': str(map_path),
            'runtime_pos_log_enable': False,
            'pcd_save.pcd_save_en': mode == 'mapping',
            'publish.path_en': True,
        }])]
    # Public datasets describe the sensor/IMU origin, not the team's robot centre.
    # Zero extrinsics are a deliberate dataset-only configuration.
    pose_params = dict(common, **{
        'output_pose_alias_topic': '', 'zero_on_first_pose': False,
        'reference_pose_enabled': False, 'output_stamp_mode': 'input',
        'lidar_to_base_xyz': [0.0, 0.0, 0.0],
        'lidar_to_base_rpy': [0.0, 0.0, 0.0],
        'publish_tf': False, 'publish_static_tf': False,
    })
    nodes.append(Node(
        package='lidar_localization', executable='pose_from_odom',
        name='offline_fast_pose', output='screen', parameters=[pose_params, {
            'input_odom_topic': '/Odometry',
            'output_pose_topic': '/robot/pose_odom_only',
            'output_point_topic': '/training/fast_point',
            'output_odom_topic': '/training/fast_odom',
        }]))
    if mode == 'localization':
        nodes.extend([
            Node(package='lidar_map_localization', executable='map_localizer',
                 output='screen', parameters=[
                     str(Path(get_package_share_directory('lidar_map_localization')) /
                         'config/map_localizer.yaml'), common, {
                         'map_path': str(map_path),
                         # The map and scan start at the same dataset origin.
                         'auto_initialize_identity': True,
                         'localize_period_sec': 0.2,
                         'map_leaf_size': 0.25, 'scan_leaf_size': 0.25,
                         'registration_method': value('registration_method'),
                         'fitness_threshold': 0.20,
                     }]),
            Node(package='lidar_localization', executable='pose_from_odom',
                 name='offline_map_pose', output='screen', parameters=[pose_params, {
                     'input_odom_topic': '/map_lidar/odom',
                     'output_pose_topic': '/robot/pose',
                     'output_point_topic': '/training/map_point',
                     'output_odom_topic': '/training/map_odom',
                 }]),
            Node(package='lidar_localization', executable='continuous_robot_pose',
                 output='screen', parameters=[common, {
                     'output_frame_id': 'map', 'child_frame_id': 'body',
                     'output_stamp_mode': 'input',
                     'reference_pose_enabled': False,
                     'lidar_to_base_xyz': [0.0, 0.0, 0.0],
                     'lidar_to_base_rpy': [0.0, 0.0, 0.0],
                     'stamp_match_mode': 'stamp',
                 }]),
        ])
    return nodes


def generate_launch_description():
    config = str(Path(get_package_share_directory('fast_lio')) / 'config/mid360.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping'),
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('map_path', default_value='maps/offline_demo.pcd'),
        DeclareLaunchArgument('registration_method', default_value='icp'),
        OpaqueFunction(function=setup),
    ])
