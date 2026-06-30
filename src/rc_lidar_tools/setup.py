from setuptools import find_packages, setup

package_name = 'rc_lidar_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rc',
    maintainer_email='linboxi123@163.com',
    description='Maintenance and calibration tools for the RC LiDAR workspace.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            (
                'rotation_extrinsic_calibrator = '
                'rc_lidar_tools.rotation_extrinsic_calibrator:main'
            ),
        ],
    },
)
