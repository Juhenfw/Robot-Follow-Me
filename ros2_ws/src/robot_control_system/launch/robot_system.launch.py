from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Parameters
    safety_distance_arg = DeclareLaunchArgument(
        'safety_distance',
        default_value='0.5',
        description='Distance (m) to start obstacle avoidance behaviors'
    )
    
    # Try to find packages
    try:
        rplidar_dir = get_package_share_directory('rplidar_ros')
        rplidar_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(rplidar_dir, 'launch', 'view_rplidar_a2m12_launch.py')
            ])
        )
    except:
        rplidar_launch = None
        print("Warning: RPLidar package not found")
    
    try:
        gamepad_dir = get_package_share_directory('gamepad_robot_controller')
        gamepad_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(gamepad_dir, 'launch', 'gamepad_controller.launch.py')
            ])
        )
    except:
        gamepad_launch = None
        print("Warning: Gamepad package not found")
    
    # Core nodes
    full_autonomous_node = Node(
        package='robot_control_system',
        executable='full_autonomous',
        name='full_autonomous',
        parameters=[{
            'default_mode': 'manual',
            'safety_distance': LaunchConfiguration('safety_distance'),
            'danger_distance': 0.3,
            'scan_angle_range': 180,
            'max_linear_speed': 0.2,
            'max_angular_speed': 0.5
        }],
        output='screen'
    )
    
    gamepad_wrapper_node = Node(
        package='robot_control_system',
        executable='gamepad_wrapper',
        name='gamepad_wrapper',
        output='screen'
    )
    
    # Using keyboard_controller instead of simple_keyboard
    keyboard_node = Node(
        package='robot_control_system',
        executable='keyboard_controller',
        name='keyboard_controller_node',
        output='screen'
    )
    
    # Build launch description
    ld = LaunchDescription([safety_distance_arg])
    
    # Add our nodes
    ld.add_action(full_autonomous_node)
    ld.add_action(gamepad_wrapper_node)
    ld.add_action(keyboard_node)
    
    # Add LiDAR and gamepad if available
    if rplidar_launch:
        ld.add_action(rplidar_launch)
    if gamepad_launch:
        ld.add_action(gamepad_launch)
    
    return ld