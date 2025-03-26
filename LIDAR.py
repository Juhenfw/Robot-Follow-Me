import math
import numpy as np
import time
import serial
import ddsm115 as motor
from rplidar import RPLidar
from threading import Thread

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

# Initialize wheel motors
right_motor = motor.MotorControl(device=RIGHT_WHEEL_PORT)
right_motor.set_drive_mode(1, 2)
left_motor = motor.MotorControl(device=LEFT_WHEEL_PORT)
left_motor.set_drive_mode(1, 2)

# Initialize LiDAR
lidar = RPLidar(PORT_LIDAR, baudrate=256000)

# Motor speed parameters
NORMAL_SPEED = 75  # Normal speed
SLOW_SPEED = NORMAL_SPEED // 6  # Slow rotation speed
TURNING_SPEED = NORMAL_SPEED // 4  # Normal turning speed
DRIFT_BOOST = int(NORMAL_SPEED * 1.3)  # Drift speed
STOP = 0

# Robot following parameters
FOLLOW_DISTANCE = 150  # Target distance to maintain (cm)
DISTANCE_TOLERANCE = 30  # Tolerance range (cm)
ANGLE_TOLERANCE = 10  # Angle tolerance (degrees)

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
    
    print(f"Moving: R={-right_speed}, L={left_speed}")

def read_uwb_data():
    """Reads data from UWB sensor."""
    global uwb_distance, running
    
    try:
        ser = serial.Serial(PORT_UWB, BAUDRATE_UWB, timeout=TIMEOUT_UWB)
        print(f"Connected to UWB sensor at {PORT_UWB}")
        
        while running:
            if ser.in_waiting > 0:
                data = ser.readline().decode("utf-8").strip()
                
                if data.startswith("$KT0"):
                    parts = data.split(",")
                    if len(parts) >= 4:
                        raw_values = parts[1:4]
                        processed_values = []
                        
                        for value in raw_values:
                            if value.lower() == "null":
                                processed_values.append(0.0)
                            else:
                                processed_values.append(float(value))
                                
                        A0, A1, A2 = processed_values
                        
                        # Apply bias correction
                        cal_A0 = correct_bias(A0*100, 15)  # Convert to cm and correct bias
                        uwb_distance = cal_A0
                        print(f"UWB Distance: {cal_A0:.2f} cm")
                
            time.sleep(0.05)
            
    except serial.SerialException as e:
        print(f"UWB Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("UWB serial connection closed.")

def update_lidar():
    """Updates LiDAR data continuously."""
    global running, angles, distances, colors, target_angle, target_distance
    
    try:
        for scan in lidar.iter_scans():
            if not running:
                break
                
            scan_angles = []
            scan_distances = []
            
            for _, angle, distance in scan:
                if distance <= 2000:  # Limit to 2000 mm
                    scan_angles.append(angle)
                    scan_distances.append(distance)
            
            if scan_distances:
                # Find clusters of points that might represent a person
                # Simple approach: find the closest point within a reasonable range
                min_distance_idx = scan_distances.index(min(scan_distances))
                target_angle = scan_angles[min_distance_idx]
                target_distance = scan_distances[min_distance_idx]
                
                print(f"LiDAR Target: angle={target_angle:.2f}°, distance={target_distance/10:.2f}cm")
            
            time.sleep(0.05)
            
    except Exception as e:
        print(f"LiDAR Error: {e}")
    finally:
        if running:  # Only stop if we're still running (avoid double-stop)
            lidar.stop()
            print("LiDAR stopped.")

def follow_person():
    """Main control logic to follow a person."""
    global running, uwb_distance, target_angle, target_distance
    
    try:
        while running:
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
                
                if abs(angle_error) > ANGLE_TOLERANCE:
                    # Need to rotate to face person
                    rotation_speed = min(max(angle_error / 2, -NORMAL_SPEED), NORMAL_SPEED)
                    rotation_command = rotation_speed
                    print(f"Rotating to face target: {angle_error:.2f}°")
                
            # Determine forward/backward command (based on UWB distance)
            if uwb_distance is not None:
                distance_error = uwb_distance - FOLLOW_DISTANCE
                
                if abs(distance_error) > DISTANCE_TOLERANCE:
                    # Need to adjust distance
                    if distance_error > 0:
                        # Too far, move forward
                        forward_speed = min(distance_error / 3, NORMAL_SPEED)
                        forward_command = forward_speed
                        print(f"Moving forward: {distance_error:.2f}cm too far")
                    else:
                        # Too close, move backward
                        forward_speed = max(distance_error / 3, -NORMAL_SPEED)
                        forward_command = forward_speed
                        print(f"Moving backward: {-distance_error:.2f}cm too close")
            
            # Execute movement
            move_robot(forward_command, rotation_command)
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("Follow operation stopped by user.")
    except Exception as e:
        print(f"Follow Error: {e}")

def cleanup():
    """Cleans up resources before exiting."""
    global running
    
    running = False
    time.sleep(0.5)  # Allow threads to terminate
    
    # Stop motors
    right_motor.send_rpm(1, 0)
    left_motor.send_rpm(1, 0)
    right_motor.close()
    left_motor.close()
    
    # Stop LiDAR
    try:
        lidar.stop()
        lidar.disconnect()
    except:
        pass
    
    print("All systems stopped and disconnected.")

def main():
    """Main program entry point."""
    global running
    
    print("Starting Follow Me Robot System...")
    running = True
    
    # Start sensor threads
    uwb_thread = Thread(target=read_uwb_data, daemon=True)
    lidar_thread = Thread(target=update_lidar, daemon=True)
    
    uwb_thread.start()
    lidar_thread.start()
    
    time.sleep(2)  # Give sensors time to start up
    
    try:
        # Start following behavior
        follow_person()
    except KeyboardInterrupt:
        print("Program terminated by user.")
    finally:
        cleanup()

if __name__ == "__main__":
    main()
