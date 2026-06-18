import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'fastlio_world_init'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rc',
    maintainer_email='linboxi123@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'world_from_fastlio_init = fastlio_world_init.world_from_fastlio_init:main',
            'robot_tf_offsets = fastlio_world_init.robot_tf_offsets:main',
            'lidar_tf_offset = fastlio_world_init.lidar_tf_offset:main',
        ],
    },
)
