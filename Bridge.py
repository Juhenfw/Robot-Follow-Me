# RECEIVER

import socket
import time
import math
import traceback
import numpy as np
from collections import deque
from motor_driver import MotorDriver  # Ensure this file exists

# UDP Configuration
UDP_IP = "0.0.0.0"  # Listen on all available interfaces
UDP_PORT = 5005
print(f"Initializing receiver on UDP port {UDP_PORT}...")

# Create and bind socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
print(f"Successfully listening on UDP port {UDP_PORT}")

# Set socket timeout to enable keyboard interrupt handling
sock.settimeout(1.0)

# Motor Configuration
try:
    motor_kanan = MotorDriver(16, 18)  # GPIO pins for right motor
    motor_kiri = MotorDriver(11, 13)   # GPIO pins for left motor
    print("Motors initialized successfully")
except Exception as e:
    print(f"Error initializing motors: {e}")
    traceback.print_exc()
    motor_kanan = None
    motor_kiri = None

# Filtering setup
buffer_size = 5  # Size of moving average window
distance_buffer = deque(maxlen=buffer_size)
x_buffer = deque(maxlen=buffer_size)
y_buffer = deque(maxlen=buffer_size)
angle_buffer = deque(maxlen=buffer_size)

# Target coordinates (where the robot should go)
target_x = 0
target_y = 0
print(f"Target position set to ({target_x}, {target_y})")

# Maximum allowed age for position data (in seconds)
MAX_DATA_AGE = 2.0

def hitung_jarak_sudut(x, y):
    """Calculate distance and angle to target"""
    # Distance calculation using Euclidean distance
    jarak = math.sqrt((x - target_x)**2 + (y - target_y)**2)
    
    # Angle calculation (in degrees)
    sudut = math.degrees(math.atan2(y - target_y, x - target_x))
    
    # Adjust angle to be in the range of -180 to 180 degrees
    if sudut > 180:
        sudut -= 360
    elif sudut < -180:
        sudut += 360
        
    return jarak, sudut

def filter_data(new_value, data_buffer):
    """Apply moving average filter to any data point"""
    data_buffer.append(new_value)
    return sum(data_buffer) / len(data_buffer)

def control_motors(distance, angle):
    """Control motors based on distance and angle to target"""
    if motor_kanan is None or motor_kiri is None:
        print("Motors not initialized. Cannot control movement.")
        return
        
    # Base motor speed (0-100)
    base_speed = 50
    
    # Distance factor: slow down as we get closer to target
    distance_factor = min(1.0, distance / 2.0)  # Full speed until 2m away
    
    # Angle factor: adjust differential based on angle
    # When angle is 0, motors are equal
    # When angle is positive (target to the right), left motor is faster
    # When angle is negative (target to the left), right motor is faster
    angle_factor = angle / 90.0  # Normalize angle effect
    
    # Calculate motor speeds with proportional control
    left_speed = base_speed * distance_factor * (1 - angle_factor)
    right_speed = base_speed * distance_factor * (1 + angle_factor)
    
    # Clamp motor speeds to valid range (0-100)
    left_speed = max(0, min(100, left_speed))
    right_speed = max(0, min(100, right_speed))
    
    # Stop motors if very close to target
    if distance < 0.1:  # 10cm
        left_speed = 0
        right_speed = 0
        print("Target reached! Stopping motors.")
    
    # Apply motor speeds
    motor_kiri.set_speed(left_speed)
    motor_kanan.set_speed(right_speed)
    
    print(f"Motor control: Left={left_speed:.1f}, Right={right_speed:.1f}")

# Main program
print("UWB Data Receiver Starting...")
last_data_time = 0

try:
    while True:
        try:
            # Try to receive data with timeout
            data, addr = sock.recvfrom(1024)
            print(f"Received data from {addr}")
            
            # Parse the received data
            try:
                data_str = data.decode('utf-8')
                parts = data_str.split(',')
                
                if len(parts) >= 4:
                    timestamp = float(parts[0])
                    x = float(parts[1])
                    y = float(parts[2])
                    z = float(parts[3])
                    
                    # Check data freshness
                    current_time = time.time()
                    data_age = current_time - timestamp
                    
                    if data_age <= MAX_DATA_AGE:
                        # Apply filtering
                        x_filtered = filter_data(x, x_buffer)
                        y_filtered = filter_data(y, y_buffer)
                        
                        # Calculate distance and angle to target
                        distance, angle = hitung_jarak_sudut(x_filtered, y_filtered)
                        
                        # Apply filtering to calculated values
                        distance_filtered = filter_data(distance, distance_buffer)
                        angle_filtered = filter_data(angle, angle_buffer)
                        
                        # Display processed data
                        print(f"Position: X={x_filtered:.2f}, Y={y_filtered:.2f}, Z={z:.2f}")
                        print(f"Target: Distance={distance_filtered:.2f}m, Angle={angle_filtered:.2f}°")
                        
                        # Control motors based on filtered distance and angle
                        control_motors(distance_filtered, angle_filtered)
                        last_data_time = current_time
                    else:
                        print(f"Received outdated data ({data_age:.1f}s old), ignoring")
                else:
                    print(f"Invalid data format: {data_str}")
            
            except ValueError as e:
                print(f"Error parsing data: {e}")
                
        except socket.timeout:
            # Check if we've lost connection
            current_time = time.time()
            if last_data_time > 0 and (current_time - last_data_time) > MAX_DATA_AGE:
                print("No fresh position data received. Stopping motors.")
                if motor_kanan is not None and motor_kiri is not None:
                    motor_kanan.set_speed(0)
                    motor_kiri.set_speed(0)
                    
        except Exception as e:
            print(f"Unexpected error: {e}")
            traceback.print_exc()
            time.sleep(1)
            
except KeyboardInterrupt:
    print("\nProgram stopped by user")
except Exception as e:
    print(f"Fatal error: {e}")
    traceback.print_exc()
finally:
    # Ensure motors are stopped
    if 'motor_kanan' in locals() and motor_kanan is not None:
        motor_kanan.set_speed(0)
    if 'motor_kiri' in locals() and motor_kiri is not None:
        motor_kiri.set_speed(0)
    # Close socket
    sock.close()
    print("Receiver script terminated")
