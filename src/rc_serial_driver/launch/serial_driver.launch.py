import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rc_serial_driver'), 'config', 'rc_serial_driver.yaml')

    def env_bool(name, default):
        return os.environ.get(name, default).lower() in ('1', 'true', 'yes', 'on')

    can_interface = os.environ.get('CAN_INTERFACE', 'can0')
    can_bitrate = int(os.environ.get('CAN_BITRATE', '1000000'))
    pose_source = os.environ.get('RC_CAN_POSE_SOURCE', 'tf')
    pose_topic = os.environ.get('RC_CAN_POSE_TOPIC', '/lidar/pose')
    yaw_pose_topic_enabled = env_bool('RC_CAN_USE_YAW_POSE_TOPIC', '0')
    yaw_pose_topic = os.environ.get('RC_CAN_YAW_POSE_TOPIC', '/robot/pose')
    yaw_pose_topic_timeout_sec = float(os.environ.get('RC_CAN_YAW_POSE_TOPIC_TIMEOUT_SEC', '0.5'))
    yaw_pose_topic_strict = env_bool('RC_CAN_YAW_POSE_TOPIC_STRICT', '0')
    quality_gate_enabled = env_bool('RC_CAN_REQUIRE_READY', '0')
    quality_topic = os.environ.get('RC_CAN_READY_QUALITY_TOPIC', '/map_lidar/quality')
    quality_timeout_sec = float(os.environ.get('RC_CAN_READY_TIMEOUT_SEC', '1.0'))
    quality_max_fitness = float(os.environ.get('RC_CAN_READY_MAX_FITNESS', '0.05'))
    quality_min_accepts = int(os.environ.get('RC_CAN_READY_MIN_ACCEPTS', '5'))
    quality_require_accepted = env_bool('RC_CAN_READY_REQUIRE_ACCEPTED', '1')
    quality_require_converged = env_bool('RC_CAN_READY_REQUIRE_CONVERGED', '1')

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
                'pose_source': pose_source,
                'pose_topic': pose_topic,
                'yaw_pose_topic_enabled': yaw_pose_topic_enabled,
                'yaw_pose_topic': yaw_pose_topic,
                'yaw_pose_topic_timeout_sec': yaw_pose_topic_timeout_sec,
                'yaw_pose_topic_strict': yaw_pose_topic_strict,
                'quality_gate_enabled': quality_gate_enabled,
                'quality_topic': quality_topic,
                'quality_timeout_sec': quality_timeout_sec,
                'quality_max_fitness': quality_max_fitness,
                'quality_min_consecutive_accepts': quality_min_accepts,
                'quality_require_accepted': quality_require_accepted,
                'quality_require_converged': quality_require_converged,
            },
        ],
    )

    return LaunchDescription([rc_serial_driver_node])
