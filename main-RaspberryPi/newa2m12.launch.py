#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

def generate_launch_description():
    # Launch configurations for parameters
    channel_type = LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='256000')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    # QoS configuration for LIDAR to handle high-frequency data
    lidar_qos = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Adjust reliability for speed
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10  # Adjust depth based on data frequency
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('channel_type', default_value=channel_type, description='LIDAR channel type'),
        DeclareLaunchArgument('serial_port', default_value=serial_port, description='Serial port for LIDAR connection'),
        DeclareLaunchArgument('serial_baudrate', default_value=serial_baudrate, description='Baudrate for LIDAR serial connection'),
        DeclareLaunchArgument('frame_id', default_value=frame_id, description='Frame ID for LIDAR data'),
        DeclareLaunchArgument('inverted', default_value=inverted, description='Inverted scan data flag (True/False)'),
        DeclareLaunchArgument('angle_compensate', default_value=angle_compensate, description='Enable angle compensation for scan data'),
        DeclareLaunchArgument('scan_mode', default_value=scan_mode, description='LIDAR scan mode (Sensitivity/Turbo)'),

        # Logging info to display on screen for debugging
        LogInfo(
            condition=LaunchConfiguration('debug'),
            msg="Launching RPLIDAR with the following parameters:"
        ),

        # Node for RPLIDAR sensor
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate,
                'scan_mode': scan_mode
            }],
            remappings=[('/scan', '/lidar_scan')],  # Remap topic to /lidar_scan
            output='screen',
            qos_profile=lidar_qos  # Apply QoS settings for better data handling
        ),
    ])
