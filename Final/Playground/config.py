# placeholder/n

# Configuration for Robot Follower System

ROBOT_CONFIG = {
    'min_follow_distance': 30,  # cm
    'max_follow_distance': 300,  # cm
    'base_speed': 75,  # 0-100 scale
    'slow_speed': 30,
    'max_speed': 100,
    'turn_rate_factor': 0.5
}

UWB_CONFIG = {
    'anchor_positions': {
        'A1': [0, 0],
        'A2': [5, 0],
        'A3': [0, 5],
        'A4': [5, 5]
    },
    'update_rate': 10,  # Hz
    'port': '/dev/ttyUSB0',
    'baud_rate': 115200
}

LIDAR_CONFIG = {
    'scan_rate': 5,  # Hz
    'min_distance': 0.1,  # meters
    'max_distance': 5.0,  # meters
    'angle_increment': 1.0,  # degrees
    'port': '/dev/ttyUSB1',
    'baud_rate': 115200
}

SAFETY_CONFIG = {
    'min_obstacle_distance': 0.5,  # meters
    'max_target_error': 0.5,  # meters
    'min_confidence': 0.6  # 0-1 scale
}

LOG_CONFIG = {
    'log_level': 'INFO',
    'log_to_file': True,
    'log_dir': 'logs'
}
