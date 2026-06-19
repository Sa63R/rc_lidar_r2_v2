from launch import LaunchDescription
from launch_ros.actions import Node

ROBOT_AXES_YAW_RAD = "1.5707963267948966"


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
                "--yaw", ROBOT_AXES_YAW_RAD,
                "--frame-id", "body",
                "--child-frame-id", "base_link",
            ],
            output="screen",
        )
    ])
