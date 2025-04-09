# placeholder/n

#!/usr/bin/env python3
# Main entry point for Robot Follower System

import os
import sys
import time
import argparse
import json
import signal
import threading
from config import *
from sensors.lidar_sensor import LidarSensor
from sensors.uwb_tracker import UWBTracker
from Playground import RobotController
from utils.logging_setup import setup_logging
from utils.visualization import VisualizationClient

# Global flag for graceful shutdown
shutdown_event = threading.Event()

def signal_handler(sig, frame):
    """Signal handler for graceful shutdown"""
    print("Shutdown requested... stopping all systems")
    shutdown_event.set()

def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='Robot Follower System')
    parser.add_argument('--sim', action='store_true', help='Run in simulation mode')
    parser.add_argument('--config', type=str, help='Path to config file')
    parser.add_argument('--log', type=str, default='INFO', help='Log level (DEBUG, INFO, WARNING, ERROR, CRITICAL)')
    parser.add_argument('--viz', action='store_true', help='Enable visualization')
    parser.add_argument('--viz-host', type=str, default='localhost', help='Visualization host')
    parser.add_argument('--viz-port', type=int, default=5555, help='Visualization port')
    
    return parser.parse_args()

def load_config(config_path=None):
    """Load configuration from file or use defaults"""
    config = {
        'ROBOT_CONFIG': ROBOT_CONFIG,
        'UWB_CONFIG': UWB_CONFIG,
        'LIDAR_CONFIG': LIDAR_CONFIG,
        'SAFETY_CONFIG': SAFETY_CONFIG,
        'LOG_CONFIG': LOG_CONFIG
    }
    
    if config_path and os.path.exists(config_path):
        try:
            with open(config_path, 'r') as f:
                file_config = json.load(f)
                
            # Merge configurations
            for key, value in file_config.items():
                if key in config:
                    config[key].update(value)
                else:
                    config[key] = value
                    
            print(f"Loaded configuration from {config_path}")
        except Exception as e:
            print(f"Error loading config from {config_path}: {e}")
    
    return config

def main():
    """Main function"""
    # Setup signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Parse arguments
    args = parse_arguments()
    
    # Load configuration
    config = load_config(args.config)
    
    # Setup logging
    log_config = config.get('LOG_CONFIG', {})
    logger = setup_logging(
        log_level=args.log if args.log else log_config.get('log_level', 'INFO'),
        log_to_file=log_config.get('log_to_file', True),
        log_dir=log_config.get('log_dir', 'logs')
    )
    
    logger.info("Starting Robot Follower System")
    logger.info(f"Simulation mode: {'ENABLED' if args.sim else 'DISABLED'}")
    
    try:
        # Initialize sensors
        logger.info("Initializing sensors...")
        
        lidar = LidarSensor(
            simulation_mode=args.sim,
            config=config.get('LIDAR_CONFIG', {})
        )
        
        uwb_tracker = UWBTracker(
            simulation_mode=args.sim,
            config=config.get('UWB_CONFIG', {})
        )
        
        # Initialize controller
        logger.info("Initializing robot controller...")
        controller = RobotController(
            lidar=lidar,
            uwb_tracker=uwb_tracker,
            config=config,
            simulation_mode=args.sim
        )
        
        # Initialize visualization if enabled
        viz_client = None
        if args.viz:
            logger.info("Initializing visualization client...")
            viz_client = VisualizationClient(
                host=args.viz_host,
                port=args.viz_port
            )
            viz_client.connect()
        
        # Start sensors and controller
        logger.info("Starting sensors...")
        lidar.start()
        uwb_tracker.start()
        
        logger.info("Starting robot controller...")
        controller.start()
        
        logger.info("System initialized and running")
        
        # Main loop
        update_interval = 0.1  # seconds
        last_update_time = time.time()
        
        while not shutdown_event.is_set():
            current_time = time.time()
            
            # Periodic updates
            if current_time - last_update_time >= update_interval:
                # Get latest data
                lidar_data = lidar.get_data()
                target_pos, target_vel, target_diag = uwb_tracker.get_data()
                robot_pos, robot_orientation = controller.get_current_position()
                left_speed, right_speed = controller.get_current_speed()
                safety_status = controller.safety.get_safety_status()
                
                # Send to visualization if enabled
                if viz_client and viz_client.connected:
                    viz_data = {
                        'timestamp': time.time(),
                        'robot': {
                            'position': robot_pos.tolist(),
                            'orientation': robot_orientation,
                            'left_speed': left_speed,
                            'right_speed': right_speed
                        },
                        'target': {
                            'position': target_pos.tolist(),
                            'velocity': target_vel.tolist(),
                            'error': target_diag.get('error_estimate', 0.0),
                            'confidence': target_diag.get('confidence', 0.0)
                        },
                        'safety': safety_status
                    }
                    viz_client.send_data(viz_data)
                
                last_update_time = current_time
            
            # Sleep a bit to prevent CPU overuse
            time.sleep(0.01)
        
    except Exception as e:
        logger.error(f"Error in main loop: {e}", exc_info=True)
    
    finally:
        # Shutdown everything
        logger.info("Shutting down...")
        
        if controller and controller.is_running:
            logger.info("Stopping controller...")
            controller.stop()
        
        if lidar and lidar.is_running:
            logger.info("Stopping LIDAR...")
            lidar.stop()
        
        if uwb_tracker and uwb_tracker.is_running:
            logger.info("Stopping UWB tracker...")
            uwb_tracker.stop()
        
        if viz_client:
            logger.info("Disconnecting visualization...")
            viz_client.disconnect()
        
        logger.info("Shutdown complete")

if __name__ == "__main__":
    main()
