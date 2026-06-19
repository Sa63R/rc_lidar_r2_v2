from launch import LaunchDescription
from launch_ros.actions import Node

YAW_180_DEG_RAD = "3.141592653589793"


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x", "-0.240",
                "--y", "0.287302",
                "--z", "-0.760",
                "--roll", "0",
                "--pitch", "0",
                "--yaw", YAW_180_DEG_RAD,
                "--frame-id", "world",
                "--child-frame-id", "init",
            ],
            output="screen",
        )
    ])
