from setuptools import setup
import os
from glob import glob

package_name = 'robot_control_system'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ympipa',
    maintainer_email='your.email@example.com',
    description='Robot control system with manual and autonomous modes',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mode_controller = robot_control_system.mode_controller:main',
            'gamepad_wrapper = robot_control_system.gamepad_wrapper:main',
            'mode_switcher = robot_control_system.mode_switcher_key:main',
            'full_autonomous = robot_control_system.full_autonomous:main',
        ],
    },
)
