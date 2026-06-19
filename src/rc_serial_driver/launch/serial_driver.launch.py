import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rc_serial_driver'), 'config', 'rc_serial_driver.yaml')
    can_interface = os.environ.get('CAN_INTERFACE', 'can0')
    can_bitrate = int(os.environ.get('CAN_BITRATE', '1000000'))

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
            },
        ],
    )

    return LaunchDescription([rc_serial_driver_node])
