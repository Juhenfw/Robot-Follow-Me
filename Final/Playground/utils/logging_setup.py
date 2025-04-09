# placeholder/n

# Logging setup

import logging
import os
import time
import json
import socket
import platform
import psutil
from datetime import datetime

def setup_logging(log_level="INFO", log_to_file=False, log_dir="logs"):
    """
    Setup logging configuration
    
    Args:
        log_level (str): Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        log_to_file (bool): Whether to log to file
        log_dir (str): Directory for log files
        
    Returns:
        logging.Logger: Configured logger
    """
    # Create log directory if needed
    if log_to_file and not os.path.exists(log_dir):
        os.makedirs(log_dir)
    
    # Get logger
    logger = logging.getLogger("robot_follower")
    
    # Clear existing handlers
    if logger.handlers:
        for handler in logger.handlers[:]:
            logger.removeHandler(handler)
    
    # Set log level
    level = getattr(logging, log_level.upper())
    logger.setLevel(level)
    
    # Create formatter
    formatter = logging.Formatter(
        '[%(asctime)s] [%(levelname)s] [%(module)s:%(lineno)d] %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    # Console handler
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)
    
    # File handler if requested
    if log_to_file:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_file = os.path.join(log_dir, f"robot_follower_{timestamp}.log")
        file_handler = logging.FileHandler(log_file)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
        
        # Log system info at startup
        logger.info(f"Log file created at: {log_file}")
        log_system_info(logger)
    
    return logger

def log_system_info(logger):
    """
    Log system information at startup
    
    Args:
        logger (logging.Logger): Logger instance
    """
    try:
        logger.info(f"System: {platform.system()} {platform.release()}")
        logger.info(f"Hostname: {socket.gethostname()}")
        logger.info(f"Python: {platform.python_version()}")
        logger.info(f"CPU: {psutil.cpu_percent()}% used")
        logger.info(f"Memory: {psutil.virtual_memory().percent}% used")
        logger.info(f"Disk: {psutil.disk_usage('/').percent}% used")
    except Exception as e:
        logger.warning(f"Error collecting system info: {e}")

def log_system_status(logger, sensors=None, controller=None):
    """
    Log system status periodically
    
    Args:
        logger (logging.Logger): Logger instance
        sensors (dict): Dictionary of sensor objects
        controller (RobotController): Robot controller instance
    """
    status = {
        'timestamp': datetime.now().isoformat(),
        'cpu': psutil.cpu_percent(),
        'memory': psutil.virtual_memory().percent
    }
    
    # Add sensor data if available
    if sensors:
        for name, sensor in sensors.items():
            if hasattr(sensor, 'is_running'):
                status[f'{name}_running'] = sensor.is_running
    
    # Add controller data if available
    if controller and hasattr(controller, 'get_current_speed'):
        left, right = controller.get_current_speed()
        status['motor_left'] = left
        status['motor_right'] = right
    
    # Log the status
    logger.debug(f"System status: {json.dumps(status)}")
    
    return status
