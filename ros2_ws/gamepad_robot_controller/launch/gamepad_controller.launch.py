from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable

def generate_launch_description():
    # Launch arguments
    r_wheel_port_arg = DeclareLaunchArgument(
        'r_wheel_port',
        default_value='/dev/ttyRS485-1',
        description='Right wheel motor port'
    )

    l_wheel_port_arg = DeclareLaunchArgument(
        'l_wheel_port',
        default_value='/dev/ttyRS485-2',
        description='Left wheel motor port'
    )

    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='30.0',
        description='Controller update rate in Hz'
    )

    # Ensure environment variables needed for pygame display are passed
    display_env = SetEnvironmentVariable(
        name='DISPLAY',
        value=EnvironmentVariable('DISPLAY', default_value=':0')
    )

    xauthority_env = SetEnvironmentVariable(
        name='XAUTHORITY',
        value=EnvironmentVariable('XAUTHORITY', default_value='~/.Xauthority')
    )

    # Set SDL_VIDEODRIVER to make pygame work better in launch files
    sdl_videodriver = SetEnvironmentVariable(
        name='SDL_VIDEODRIVER',
        value='x11'
    )

    # Create the node with output='screen' to see all console output
    gamepad_controller_node = Node(
        package='gamepad_robot_controller',
        executable='gamepad_controller',
        name='gamepad_controller',
        parameters=[{
            'r_wheel_port': LaunchConfiguration('r_wheel_port'),
            'l_wheel_port': LaunchConfiguration('l_wheel_port'),
            'update_rate': LaunchConfiguration('update_rate')
        }],
        output='screen',
        emulate_tty=True,  # Ensures proper terminal output
    )

    return LaunchDescription([
        r_wheel_port_arg,
        l_wheel_port_arg,
        update_rate_arg,
        display_env,
        xauthority_env,
        sdl_videodriver,
        gamepad_controller_node
    ])
