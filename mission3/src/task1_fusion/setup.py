from glob import glob
from setuptools import setup, find_packages

setup(name='task1_fusion', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/task1_fusion']),
                  ('share/task1_fusion', ['package.xml']),
                  ('share/task1_fusion/config', glob('config/*.yaml') + glob('config/*.xml')),
                  ('share/task1_fusion/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools', 'numpy', 'PyYAML'], zip_safe=True,
      maintainer='Task 1 workspace', maintainer_email='maintainer@example.com',
      description='Wheel and delayed LiDAR fusion', license='MIT',
      entry_points={'console_scripts': [
          'fusion = task1_fusion.fusion_node:main',
          'corner = task1_fusion.corner_node:main',
          'vision = task1_fusion.vision_node:main',
          'vision_ui = task1_fusion.vision_ui:main',
          'can_bridge = task1_fusion.can_bridge:main',
          'logger = task1_fusion.logger_node:main']})
