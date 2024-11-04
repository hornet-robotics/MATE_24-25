from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'camera_select'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Competitive Robotics Club at Sac State',
    maintainer_email='null',
    description='ROS 2 package for selecting a camera with keyboard input.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = camera_select.camera_select_node:main',
            'listener = camera_select.subscriber_node:main',
        ],
    },
)
