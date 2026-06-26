import ast
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rc_serial_driver'), 'config', 'rc_serial_driver.yaml')

    def env_bool(name, default):
        return os.environ.get(name, default).lower() in ('1', 'true', 'yes', 'on')

    def env_float_list(name, default):
        value = os.environ.get(name)
        if value is None:
            return default
        try:
            parsed = ast.literal_eval(value)
        except (SyntaxError, ValueError):
            parsed = value.split(',')
        if len(parsed) != len(default):
            raise RuntimeError(f'{name} must contain {len(default)} float values')
        return [float(item) for item in parsed]

    can_interface = os.environ.get('CAN_INTERFACE', 'can0')
    can_bitrate = int(os.environ.get('CAN_BITRATE', '1000000'))
    can_pose_debug_topic = os.environ.get('RC_CAN_POSE_DEBUG_TOPIC', '/rc_serial_driver/can_pose_debug')
    pose_source = os.environ.get('RC_CAN_POSE_SOURCE', 'tf')
    pose_topic = os.environ.get('RC_CAN_POSE_TOPIC', '/lidar/pose')
    raw_odom_pose_topic = os.environ.get('RC_CAN_RAW_ODOM_POSE_TOPIC', '/Odometry')
    raw_odom_pose_timeout_sec = float(os.environ.get('RC_CAN_RAW_ODOM_POSE_TIMEOUT_SEC', '0.5'))
    raw_odom_pose_strict = env_bool('RC_CAN_RAW_ODOM_POSE_STRICT', '1')
    raw_odom_pose_yaw_offset_rad = float(os.environ.get('RC_CAN_RAW_ODOM_POSE_YAW_OFFSET_RAD', '0.0'))
    raw_odom_apply_robot_extrinsic = env_bool('RC_CAN_RAW_ODOM_APPLY_ROBOT_EXTRINSIC', '1')
    raw_odom_lidar_to_base_xyz = env_float_list(
        'RC_CAN_RAW_ODOM_LIDAR_TO_BASE_XYZ', [0.237488, -0.272804, -0.483])
    raw_odom_lidar_to_base_rpy = env_float_list(
        'RC_CAN_RAW_ODOM_LIDAR_TO_BASE_RPY', [0.0, 0.0, -1.5707963267948966])
    raw_odom_reference_xyz = env_float_list(
        'RC_CAN_RAW_ODOM_REFERENCE_XYZ', [0.237488, -0.272804, -0.483])
    raw_odom_reference_rpy = env_float_list(
        'RC_CAN_RAW_ODOM_REFERENCE_RPY', [0.0, 0.0, -1.5707963267948966])
    yaw_pose_topic_enabled = env_bool('RC_CAN_USE_YAW_POSE_TOPIC', '0')
    yaw_pose_topic = os.environ.get('RC_CAN_YAW_POSE_TOPIC', '/robot/pose')
    yaw_pose_topic_timeout_sec = float(os.environ.get('RC_CAN_YAW_POSE_TOPIC_TIMEOUT_SEC', '0.5'))
    yaw_pose_topic_strict = env_bool('RC_CAN_YAW_POSE_TOPIC_STRICT', '0')
    yaw_ignore_quality_gate = env_bool('RC_CAN_YAW_IGNORE_READY_GATE', '1')
    raw_odom_yaw_enabled = env_bool('RC_CAN_USE_RAW_ODOM_YAW', '0')
    raw_odom_yaw_topic = os.environ.get('RC_CAN_RAW_ODOM_YAW_TOPIC', '/Odometry')
    raw_odom_yaw_timeout_sec = float(os.environ.get('RC_CAN_RAW_ODOM_YAW_TIMEOUT_SEC', '0.5'))
    raw_odom_yaw_strict = env_bool('RC_CAN_RAW_ODOM_YAW_STRICT', '0')
    raw_odom_yaw_offset_rad = float(os.environ.get('RC_CAN_RAW_ODOM_YAW_OFFSET_RAD', '0.0'))
    raw_odom_yaw_ignore_quality_gate = env_bool('RC_CAN_RAW_ODOM_YAW_IGNORE_READY_GATE', '1')
    quality_gate_enabled = env_bool('RC_CAN_REQUIRE_READY', '0')
    quality_topic = os.environ.get('RC_CAN_READY_QUALITY_TOPIC', '/map_lidar/quality')
    quality_timeout_sec = float(os.environ.get('RC_CAN_READY_TIMEOUT_SEC', '1.0'))
    quality_max_fitness = float(os.environ.get('RC_CAN_READY_MAX_FITNESS', '0.05'))
    quality_min_accepts = int(os.environ.get('RC_CAN_READY_MIN_ACCEPTS', '5'))
    quality_require_accepted = env_bool('RC_CAN_READY_REQUIRE_ACCEPTED', '1')
    quality_require_converged = env_bool('RC_CAN_READY_REQUIRE_CONVERGED', '1')
    restart_script = os.environ.get(
        'RC_RESTART_SCRIPT',
        os.path.join(
            os.environ.get('RC_LIDAR_WORKSPACE', os.getcwd()),
            'autostart',
            'restart_lidar_stack.sh',
        ),
    )

    rc_serial_driver_node = Node(
        package='rc_serial_driver',
        executable='rc_serial_driver_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[
            config,
            {
                'can_interface': can_interface,
                'can_bitrate': can_bitrate,
                'can_pose_debug_topic': can_pose_debug_topic,
                'pose_source': pose_source,
                'pose_topic': pose_topic,
                'raw_odom_pose_topic': raw_odom_pose_topic,
                'raw_odom_pose_timeout_sec': raw_odom_pose_timeout_sec,
                'raw_odom_pose_strict': raw_odom_pose_strict,
                'raw_odom_pose_yaw_offset_rad': raw_odom_pose_yaw_offset_rad,
                'raw_odom_apply_robot_extrinsic': raw_odom_apply_robot_extrinsic,
                'raw_odom_lidar_to_base_xyz': raw_odom_lidar_to_base_xyz,
                'raw_odom_lidar_to_base_rpy': raw_odom_lidar_to_base_rpy,
                'raw_odom_reference_xyz': raw_odom_reference_xyz,
                'raw_odom_reference_rpy': raw_odom_reference_rpy,
                'yaw_pose_topic_enabled': yaw_pose_topic_enabled,
                'yaw_pose_topic': yaw_pose_topic,
                'yaw_pose_topic_timeout_sec': yaw_pose_topic_timeout_sec,
                'yaw_pose_topic_strict': yaw_pose_topic_strict,
                'yaw_ignore_quality_gate': yaw_ignore_quality_gate,
                'raw_odom_yaw_enabled': raw_odom_yaw_enabled,
                'raw_odom_yaw_topic': raw_odom_yaw_topic,
                'raw_odom_yaw_timeout_sec': raw_odom_yaw_timeout_sec,
                'raw_odom_yaw_strict': raw_odom_yaw_strict,
                'raw_odom_yaw_offset_rad': raw_odom_yaw_offset_rad,
                'raw_odom_yaw_ignore_quality_gate': raw_odom_yaw_ignore_quality_gate,
                'quality_gate_enabled': quality_gate_enabled,
                'quality_topic': quality_topic,
                'quality_timeout_sec': quality_timeout_sec,
                'quality_max_fitness': quality_max_fitness,
                'quality_min_consecutive_accepts': quality_min_accepts,
                'quality_require_accepted': quality_require_accepted,
                'quality_require_converged': quality_require_converged,
                'restart_script': restart_script,
            },
        ],
    )

    return LaunchDescription([rc_serial_driver_node])
