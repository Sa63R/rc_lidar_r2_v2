from glob import glob
from setuptools import setup

package_name = "lidar_localization"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sangyuwan",
    maintainer_email="user@example.com",
    description="LiDAR pose interface nodes for MID-360 localization.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "fdcan_gateway = lidar_localization.fdcan_gateway:main",
            "canfd_pose_sender = lidar_localization.canfd_pose_sender:main",
            "pose_from_odom = lidar_localization.pose_from_odom:main",
            "continuous_robot_pose = lidar_localization.continuous_robot_pose:main",
            "pose_from_tf = lidar_localization.pose_from_tf:main",
            "pose_stability_filter = lidar_localization.pose_stability_filter:main",
            "print_lidar_pose = lidar_localization.print_lidar_pose:main",
            "print_xy_yaw = lidar_localization.print_xy_yaw:main",
        ],
    },
)
