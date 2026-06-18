from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x", "-0.315",
                "--y", "-0.240",
                "--z", "-0.800",
                "--roll", "0",
                "--pitch", "0",
                "--yaw", "90",
                "--frame-id", "lidar",
                "--child-frame-id", "init",
            ],
            output="screen",
        )
    ])
