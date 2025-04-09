#!/usr/bin/env python3
# create_project_structure.py

import os
import sys

def create_directory(path):
    """Create directory if it doesn't exist"""
    if not os.path.exists(path):
        os.makedirs(path)
        print(f"Created directory: {path}")

def create_file(path, content="# placeholder/n"):
    """Create file with minimal placeholder content"""
    with open(path, 'w') as f:
        f.write(content)
    print(f"Created file: {path}")

def create_project_structure(base_dir="myenv/share/FollowMe_YMPI/Playground"):
    """Create the robot follower project structure with empty files"""
    # Create base directory
    create_directory(base_dir)
    
    # Create config.py
    create_file(os.path.join(base_dir, "config.py"))
    
    # Create sensors directory and files
    sensors_dir = os.path.join(base_dir, "sensors")
    create_directory(sensors_dir)
    create_file(os.path.join(sensors_dir, "__init__.py"))
    create_file(os.path.join(sensors_dir, "sensor_base.py"))
    create_file(os.path.join(sensors_dir, "lidar_sensor.py"))
    create_file(os.path.join(sensors_dir, "uwb_tracker.py"))
    
    # Create controllers directory and files
    controllers_dir = os.path.join(base_dir, "controllers")
    create_directory(controllers_dir)
    create_file(os.path.join(controllers_dir, "__init__.py"))
    create_file(os.path.join(controllers_dir, "robot_controller.py"))
    create_file(os.path.join(controllers_dir, "pid_controller.py"))
    
    # Create utils directory and files
    utils_dir = os.path.join(base_dir, "utils")
    create_directory(utils_dir)
    create_file(os.path.join(utils_dir, "__init__.py"))
    create_file(os.path.join(utils_dir, "logging_setup.py"))
    create_file(os.path.join(utils_dir, "visualization.py"))
    create_file(os.path.join(utils_dir, "safety.py"))
    
    # Create main.py
    create_file(os.path.join(base_dir, "main.py"))
    
    # Make main.py executable
    os.chmod(os.path.join(base_dir, "main.py"), 0o755)
    
    print(f"/nProject structure created successfully in {base_dir}!")

if __name__ == "__main__":
    # Get base directory from command line if provided
    base_dir = sys.argv[1] if len(sys.argv) > 1 else "C:/Users/JuhenFW/VSCODE/myenv/share/FollowMe_YMPI/Playground"
    create_project_structure(base_dir)