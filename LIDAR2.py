import math
import numpy as np
import time
import serial
import pygame
import ddsm115 as motor
from rplidar import RPLidar
from threading import Thread
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# UWB Configuration
PORT_UWB = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BAUDRATE_UWB = 115200
TIMEOUT_UWB = 1

# LiDAR Configuration
PORT_LIDAR = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_f235ae4105e1d247940e6441b646a0b3-if00-port0"

# Wheel Configuration
RIGHT_WHEEL_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B0049TUZ-if00-port0"
LEFT_WHEEL_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B0045S9B-if00-port0"

# Global variables
running = False
angles = []
distances = []
colors = []
uwb_distance = None
target_angle = None
target_distance = None
autonomous_mode = True  # True for follow mode, False for manual control

# Initialize pygame for manual control
pygame.init()
screen = pygame.display.set_mode((800, 600))
pygame.display.set_caption("Follow Me Robot - Press TAB to switch modes")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 36)

# Initialize wheel motors
right_motor = motor.MotorControl(device=RIGHT_WHEEL_PORT)
right_motor.set_drive_mode(1, 2)
left_motor = motor.MotorControl(device=LEFT_WHEEL_PORT)
left_motor.set_drive_mode(1, 2)

# Initialize LiDAR
lidar = RPLidar(PORT_LIDAR, baudrate=256000)

# Motor speed parameters
NORMAL_SPEED = 30  # Normal speed
SLOW_SPEED = NORMAL_SPEED // 6  # Slow rotation speed
TURNING_SPEED = NORMAL_SPEED // 4  # Normal turning speed
DRIFT_BOOST = int(NORMAL_SPEED * 1.3)  # Drift speed
STOP = 0

# Robot following parameters
FOLLOW_DISTANCE = 50  # Target distance to maintain (cm)
DISTANCE_TOLERANCE = 50  # Tolerance range (cm)
ANGLE_TOLERANCE = 30  # Angle tolerance (degrees)

# LiDAR Real-time Plot Data
max_distance = 5000  # Max distance for LiDAR display
angles_plot = []
distances_plot = []
colors_plot = []

def correct_bias(raw_distance, bias):
    """Corrects bias in UWB sensor readings."""
    return raw_distance - bias

def calculate_rotation(target_angle, current_angle):
    """Calculates required rotation to face target."""
    angle_diff = target_angle - current_angle
    
    if angle_diff > 180:
        angle_diff -= 360
    elif angle_diff < -180:
        angle_diff += 360
        
    return angle_diff

def get_direction_name(angle):
    """Convert angle to human-readable direction."""
    directions = ["Front", "Front-Right", "Right", "Back-Right", 
                 "Back", "Back-Left", "Left", "Front-Left"]
    
    # Normalize angle to 0-360
    normalized_angle = angle % 360
    
    # Convert to direction index (8 directions)
    index = round(normalized_angle / 45) % 8
    
    return directions[index]

def move_robot(forward_speed, rotation_speed):
    """
    Controls robot movement using differential drive.
    
    Parameters:
    - forward_speed: Speed for forward/backward movement (-100 to 100)
    - rotation_speed: Speed for rotation (-100 to 100)
    """
    right_speed = forward_speed - rotation_speed
    left_speed = forward_speed + rotation_speed
    
    # Clamp speeds to valid range
    right_speed = max(min(right_speed, 100), -100)
    left_speed = max(min(left_speed, 100), -100)
    
    # Convert to wheel direction (DDSM115 specific)
    right_motor.send_rpm(1, -right_speed)  # Inverted based on motor orientation
    left_motor.send_rpm(1, left_speed)
    
    # Create movement description
    movement_desc = ""
    if forward_speed > 5:
        movement_desc += "Moving Forward"
    elif forward_speed < -5:
        movement_desc += "Moving Backward"
    else:
        movement_desc += "Stationary"
        
    if rotation_speed > 5:
        movement_desc += " + Rotating Left"
    elif rotation_speed < -5:
        movement_desc += " + Rotating Right"
    
    print(f"Movement: {movement_desc} (R={-right_speed:.1f}, L={left_speed:.1f})")

def update_lidar():
    """Updates LiDAR data continuously."""
    global running, angles_plot, distances_plot, colors_plot, target_angle, target_distance
    
    try:
        for scan in lidar.iter_scans():
            if not running:
                break
                
            angles_plot.clear()
            distances_plot.clear()
            colors_plot.clear()
            
            for _, angle, distance in scan:
                if distance <= max_distance:  # Limit to max_distance
                    angles_plot.append(np.deg2rad(angle))
                    distances_plot.append(distance)

                    # Color-coding based on distance
                    if distance < 500:
                        colors_plot.append('red')  # Close object
                    elif distance < 1500:
                        colors_plot.append('orange')  # Medium distance
                    else:
                        colors_plot.append('purple')  # Far object
            
            if distances_plot:
                # Find the closest point and use it as the target
                min_distance_idx = distances_plot.index(min(distances_plot))
                target_angle = angles_plot[min_distance_idx]
                target_distance = distances_plot[min_distance_idx]
                
                direction = get_direction_name(target_angle)
                print(f"LiDAR Target: {direction} at angle={target_angle:.2f}°, distance={target_distance/10:.2f} cm")
                
            time.sleep(0.05)
            
    except Exception as e:
        print(f"LiDAR Error: {e}")
    finally:
        if running:  # Only stop if we're still running (avoid double-stop)
            lidar.stop()
            print("LiDAR stopped.")

# Function to follow the target autonomously
def follow_person():
    """Autonomous control logic to follow a person."""
    global running, uwb_distance, target_angle, target_distance, autonomous_mode
    
    try:
        while running:
            # Skip if in manual mode
            if not autonomous_mode:
                time.sleep(0.1)
                continue
                
            if uwb_distance is None or target_angle is None:
                # No sensor data yet, wait
                time.sleep(0.1)
                continue
                
            # Default to stopped
            forward_command = 0
            rotation_command = 0
            
            # Determine rotation command (based on LiDAR angle)
            if target_angle is not None:
                angle_error = calculate_rotation(target_angle, 0)  # Assuming 0 is forward
                rotation_command = min(max(angle_error / 2, -NORMAL_SPEED), NORMAL_SPEED)
                
            # Determine forward/backward command (based on UWB distance)
            if uwb_distance is not None:
                distance_error = uwb_distance - FOLLOW_DISTANCE
                if abs(distance_error) > DISTANCE_TOLERANCE:
                    if distance_error > 0:
                        forward_command = min(distance_error / 3, NORMAL_SPEED)
                    else:
                        forward_command = max(distance_error / 3, -NORMAL_SPEED)
            
            move_robot(forward_command, rotation_command)
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("Follow operation stopped by user.")
    except Exception as e:
        print(f"Follow Error: {e}")

# Function for manual control using keyboard inputs
def handle_manual_control():
    """Process manual control inputs."""
    global running, autonomous_mode
    
    try:
        while running:
            # Skip if in autonomous mode
            if autonomous_mode:
                time.sleep(0.1)
                continue
            
            # Handle pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_TAB:
                        autonomous_mode = not autonomous_mode
                        mode_str = "AUTONOMOUS" if autonomous_mode else "MANUAL"
                        print(f"Switching to {mode_str} control mode")
            
            keys = pygame.key.get_pressed()
            forward_command = 0
            rotation_command = 0
            
            # Move forward/backward or rotate
            if keys[pygame.K_w]:
                forward_command = NORMAL_SPEED
            elif keys[pygame.K_s]:
                forward_command = -NORMAL_SPEED
            if keys[pygame.K_a]:
                rotation_command = -TURNING_SPEED
            elif keys[pygame.K_d]:
                rotation_command = TURNING_SPEED
            
            move_robot(forward_command, rotation_command)
            time.sleep(0.1)
            
    except Exception as e:
        print(f"Manual control error: {e}")

def main():
    """Main program entry point."""
    global running, autonomous_mode
    
    running = True
    autonomous_mode = True  # Start in autonomous mode by default
    
    # Start sensor threads
    uwb_thread = Thread(target=read_uwb_data, daemon=True)
    lidar_thread = Thread(target=update_lidar, daemon=True)
    follow_thread = Thread(target=follow_person, daemon=True)
    ui_thread = Thread(target=handle_manual_control, daemon=True)
    
    uwb_thread.start()
    lidar_thread.start()
    follow_thread.start()
    ui_thread.start()
    
    time.sleep(2)  # Give sensors time to start up
    
    try:
        # Handle manual control (if active)
        handle_manual_control()
    except KeyboardInterrupt:
        print("Program terminated by user.")
    finally:
        cleanup()

if __name__ == "__main__":
    main()
