from setuptools import find_packages, setup

package_name = 'simple_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Competitive Robotics Club at Sac State',
    maintainer_email='null',
    description='simple demonstration of publisher and subscriber nodes using topic',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = simple_package.publisher_node:main',
            'listener = simple_package.subscriber_node:main',
        ],
    },
)
