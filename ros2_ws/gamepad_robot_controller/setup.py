from setuptools import setup
import os
from glob import glob

package_name = 'gamepad_robot_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Remove or comment out this line:
        # ('share/ament_index/resource_index/packages',
        #    ['resource/' + package_name]),
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        ('share/' + package_name, ['package.xml']),
        # Add launch directory if you have it
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='ROS 2 node for controlling DDSM115 motors with gamepad',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gamepad_controller = gamepad_robot_controller.gamepad_controller_node:main',
        ],
    },
)
