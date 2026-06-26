from launch import LaunchDescription
from launch_ros.actions import Node

ROBOT_AXES_YAW_RAD = "0"


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x", "-0.260",
                "--y", "0.262",
                "--z", "-0.483",
                "--roll", "0",
                "--pitch", "0",
                "--yaw", ROBOT_AXES_YAW_RAD,
                "--frame-id", "map",
                "--child-frame-id", "robot_origin",
            ],
            output="screen",
        )
    ])
