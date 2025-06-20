#!/usr/bin/env python3

import socket
import numpy as np
import math
import time
import threading
import sys
import os
import select
import netifaces  # Module to get IP address from the Wi-Fi network interface
from collections import deque

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from ddsm115 import MotorControl
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# Import gpiozero for controlling GPIO pins
from gpiozero import LED

# Konfigurasi Global
SCAN_THRESHOLD = 1500  # Jarak aman dalam mm
DANGER_THRESHOLD = 600  # Jarak bahaya dalam mm
MIN_VALID_DISTANCE = 200  # Jarak minimum valid (mm) untuk menghindari noise
CRITICAL_DANGER_THRESHOLD = 250  # Jarak kritis untuk override UWB (mm)

# Motor speed configuration
DEFAULT_SPEED = 75
ROTATION_FACTOR = 2
STOP_THRESHOLD = 80  # cm

# Konfigurasi performa real-time
CONTROL_FREQUENCY = 5000000  # Hz
LIDAR_SKIP_FRAMES = 1    # Process setiap 2 frame
UWB_TIMEOUT = 0.0001      # 1ms timeout
MAX_LOOP_TIME = 0.0015    # 15ms warning threshold

# Buffer sizes
LIDAR_BUFFER_SIZE = 1
UWB_BUFFER_SIZE = 1024

# Definisikan sudut lebih jelas
FRONT_REGION = [(330, 360), (0, 30)]  # Depan: 330° hingga 360° dan 0° hingga 30°
RIGHT_REGION = (31, 140)  # Kanan: 31° hingga 140°
LEFT_REGION  = (220, 329)  # Kiri: 220° hingga 329°
BACK_REGION = (150, 210)  # Belakang: 150° hingga 210°
TARGET_EXCLUSION_ANGLE = 10  # Rentang pengecualian untuk target (menghindari tabrakan dengan target)


# GPIO Pins Initialization
gpio_pin_17 = LED(17)  # GPIO 17 for program start
gpio_pin_27 = LED(27)  # GPIO 27 for error indication

# Dynamic IP function to get Raspberry Pi IP from Wi-Fi interface (default is wlan0)
def get_ip_from_wifi(interface='wlan0'):
    """Get the IP address of the Raspberry Pi from the Wi-Fi interface"""
    try:
        ip = netifaces.ifaddresses(interface)[netifaces.AF_INET][0]['addr']
        return ip
    except (KeyError, ValueError):
        print(f"Failed to get IP address for interface: {interface}")
        return None
    
def get_ip_from_subnet(ip, target_last_digit):
    """Modify the last digit of the IP address to match the target_last_digit"""
    ip_parts = ip.split(".")
    ip_parts[-1] = str(target_last_digit)  # Replace the last digit
    return ".".join(ip_parts)


# Dynamic Object Detection Class
class DynamicObjectDetector:
    """Detects dynamic vs static objects using temporal analysis"""
    
    def __init__(self):
        self.position_history = {}  # Store position history for each detected object
        self.history_window = 10    # Number of frames to keep in history
        self.movement_threshold = 150  # mm - minimum movement to consider dynamic
        self.static_frames_required = 8  # Frames object must be static to be considered static
        self.dynamic_timeout = 5.0  # Seconds to wait for dynamic object to move away
        
        # Track detected objects
        self.current_objects = {}
        self.dynamic_objects = set()
        self.static_objects = set()
        
        # Safety state
        self.dynamic_object_detected = False
        self.dynamic_object_last_seen = 0
        self.waiting_for_dynamic_object = False
        
    def _cluster_scan_points(self, scan_data):
        """Simple object detection using clustering of nearby points"""
        objects = {}
        current_cluster = []
        cluster_id = 0
        
        # Sort angles for sequential processing
        sorted_angles = sorted(scan_data.keys())
        
        for i, angle in enumerate(sorted_angles):
            distance = scan_data[angle]
            
            # Skip invalid readings
            if distance < MIN_VALID_DISTANCE or distance > 8000:
                continue
                
            # Convert to cartesian coordinates
            x = distance * math.cos(math.radians(angle))
            y = distance * math.sin(math.radians(angle))
            
            # Check if this point belongs to current cluster
            if current_cluster:
                last_x, last_y = current_cluster[-1]
                dist_to_last = math.sqrt((x - last_x)**2 + (y - last_y)**2)
                
                if dist_to_last < 300:  # Points within 30cm belong to same object
                    current_cluster.append((x, y))
                else:
                    # End current cluster and start new one
                    if len(current_cluster) >= 3:  # Minimum points for valid object
                        # Calculate cluster center
                        center_x = sum(p[0] for p in current_cluster) / len(current_cluster)
                        center_y = sum(p[1] for p in current_cluster) / len(current_cluster)
                        objects[cluster_id] = (center_x, center_y, len(current_cluster))
                        cluster_id += 1
                    
                    current_cluster = [(x, y)]
            else:
                current_cluster = [(x, y)]
        
        # Handle last cluster
        if len(current_cluster) >= 3:
            center_x = sum(p[0] for p in current_cluster) / len(current_cluster)
            center_y = sum(p[1] for p in current_cluster) / len(current_cluster)
            objects[cluster_id] = (center_x, center_y, len(current_cluster))
        
        return objects
    
    def update_object_positions(self, scan_data):
        """Update object positions and classify as dynamic or static"""
        current_time = time.time()
        
        # Detect objects in current scan
        objects = self._cluster_scan_points(scan_data)
        
        # Match objects with previous detections
        matched_objects = {}
        for obj_id, (x, y, point_count) in objects.items():
            best_match = None
            min_distance = float('inf')
            
            # Find closest previous object
            for prev_id, history in self.position_history.items():
                if history:
                    last_pos = history[-1]['position']
                    distance = math.sqrt((x - last_pos[0])**2 + (y - last_pos[1])**2)
                    if distance < min_distance and distance < 500:  # Max 50cm movement between frames
                        min_distance = distance
                        best_match = prev_id
            
            # Update or create object history
            if best_match is not None:
                matched_objects[best_match] = (x, y, point_count)
            else:
                # New object detected
                new_id = max(self.position_history.keys(), default=-1) + 1
                matched_objects[new_id] = (x, y, point_count)
                self.position_history[new_id] = []
        
        # Update position history
        for obj_id, (x, y, point_count) in matched_objects.items():
            if obj_id not in self.position_history:
                self.position_history[obj_id] = []
            
            self.position_history[obj_id].append({
                'position': (x, y),
                'timestamp': current_time,
                'point_count': point_count
            })
            
            # Keep only recent history
            if len(self.position_history[obj_id]) > self.history_window:
                self.position_history[obj_id].pop(0)
        
        # Clean up old objects
        objects_to_remove = []
        for obj_id, history in self.position_history.items():
            if not history or current_time - history[-1]['timestamp'] > 2.0:
                objects_to_remove.append(obj_id)
        
        for obj_id in objects_to_remove:
            del self.position_history[obj_id]
            self.dynamic_objects.discard(obj_id)
            self.static_objects.discard(obj_id)
        
        # Classify objects as dynamic or static
        self._classify_objects(current_time)
        
        return matched_objects
    
    def _classify_objects(self, current_time):
        """Classify objects as dynamic or static based on movement history"""
        self.dynamic_objects.clear()
        self.static_objects.clear()
        
        for obj_id, history in self.position_history.items():
            if len(history) < 3:  # Need minimum history for classification
                continue
            
            # Calculate total movement over time
            total_movement = 0
            static_count = 0
            
            for i in range(1, len(history)):
                prev_pos = history[i-1]['position']
                curr_pos = history[i]['position']
                movement = math.sqrt((curr_pos[0] - prev_pos[0])**2 + (curr_pos[1] - prev_pos[1])**2)
                total_movement += movement
                
                if movement < self.movement_threshold:
                    static_count += 1
            
            # Classification logic
            avg_movement = total_movement / (len(history) - 1)
            static_ratio = static_count / (len(history) - 1)
            
            if avg_movement > self.movement_threshold or static_ratio < 0.6:
                self.dynamic_objects.add(obj_id)
            elif static_ratio >= 0.8 and len(history) >= self.static_frames_required:
                self.static_objects.add(obj_id)
    
    def check_dynamic_objects_in_path(self, scan_data):
        """Check if there are dynamic objects in robot's path"""
        current_time = time.time()
        
        # Update object tracking
        objects = self.update_object_positions(scan_data)
        
        # Check for dynamic objects in critical zones
        dynamic_in_path = False
        closest_dynamic_distance = float('inf')
        
        for obj_id in self.dynamic_objects:
            if obj_id in self.position_history:
                history = self.position_history[obj_id]
                if history:
                    x, y = history[-1]['position']
                    distance = math.sqrt(x**2 + y**2)
                    angle = math.degrees(math.atan2(y, x)) % 360
                    
                    # Check if dynamic object is in front region and close
                    if self._is_in_front_region(angle) and distance < SCAN_THRESHOLD:
                        dynamic_in_path = True
                        closest_dynamic_distance = min(closest_dynamic_distance, distance)
                        self.dynamic_object_detected = True
                        self.dynamic_object_last_seen = current_time
                        print(f"DYNAMIC OBJECT DETECTED in path at {distance:.0f}mm, angle {angle:.0f}°")
        
        # Check if we should wait for dynamic object to move
        if self.dynamic_object_detected:
            if current_time - self.dynamic_object_last_seen > self.dynamic_timeout:
                print("Dynamic object timeout - resuming movement")
                self.dynamic_object_detected = False
                self.waiting_for_dynamic_object = False
            elif not dynamic_in_path:
                print("Dynamic object moved away - resuming movement")
                self.dynamic_object_detected = False
                self.waiting_for_dynamic_object = False
            else:
                self.waiting_for_dynamic_object = True
        
        return self.waiting_for_dynamic_object, closest_dynamic_distance
    
    def _is_in_front_region(self, angle):
        """Memeriksa apakah sudut berada di dalam area depan robot"""
        return (330 <= angle <= 360) or (0 <= angle <= 30)

    def _is_in_right_region(self, angle):
        """Memeriksa apakah sudut berada di dalam area kanan robot"""
        return 31 <= angle <= 140

    def _is_in_left_region(self, angle):
        """Memeriksa apakah sudut berada di dalam area kiri robot"""
        return 220 <= angle <= 329

    def _is_in_back_region(self, angle):
        """Memeriksa apakah sudut berada di dalam area belakang robot"""
        return 150 <= angle <= 210


    def _is_in_back_region(self, angle):
        """Memeriksa apakah sudut berada di dalam area belakang robot"""
        return 150 <= angle <= 210

    
    def get_static_objects_for_avoidance(self):
        """Get static objects that should be avoided"""
        static_obstacles = []
        for obj_id in self.static_objects:
            if obj_id in self.position_history:
                history = self.position_history[obj_id]
                if history:
                    x, y = history[-1]['position']
                    distance = math.sqrt(x**2 + y**2)
                    angle = math.degrees(math.atan2(y, x)) % 360
                    static_obstacles.append({
                        'id': obj_id,
                        'distance': distance,
                        'angle': angle,
                        'position': (x, y)
                    })
        return static_obstacles


class UWBTracker:
    """Handles UWB data processing and position estimation"""
    
    def __init__(self):
        # Default bias correction values
        self.bias = {
            'A0': 50.0,  # Bias value in cm
            'A1': 50.0,  # Bias value in cm
            'A2': 50.0   # Bias value in cm
        }
        
        # Default scale factor values
        self.scale_factor = {
            'A0': 1.0,   # Scale factor
            'A1': 1.03,  # Scale factor
            'A2': 1.02   # Scale factor
        }
        
        # Target direction estimation
        self.target_direction = None  # Estimated angle to target (degrees)
        self.target_distance = None   # Estimated distance to target (mm)
    
    def apply_bias_correction(self, distances):
        """Koreksi bias dan scaling pada pengukuran jarak"""
        corrected_distances = {
            'A0': max((distances['A0'] * 100 * self.scale_factor['A0']) - self.bias['A0'], 0),
            'A1': max((distances['A1'] * 100 * self.scale_factor['A1']) - self.bias['A1'], 0),
            'A2': max((distances['A2'] * 100 * self.scale_factor['A2']) - self.bias['A2'], 0)
        }
        return corrected_distances
    
    def estimate_target_direction(self, distances):
        """Estimate the direction and distance to the UWB target based on distances"""
        A0, A1, A2 = distances['A0'], distances['A1'], distances['A2']
        
        # Convert to mm for consistency with LIDAR
        target_distance = A0 * 10  # cm to mm
        
        diff = A2 - A1
        if abs(diff) < 20:  # Target is roughly straight ahead
            target_direction = 0  # Front is considered 0 degrees
        elif diff < 0:  # A2 < A1, target is more to the right
            angle_offset = min(45, abs(diff) * 1.5)  # Scale factor to tune sensitivity
            target_direction = 90 + angle_offset  # Right of front
        else:  # A1 < A2, target is more to the left
            angle_offset = min(45, abs(diff) * 1.5)  # Scale factor to tune sensitivity
            target_direction = 270 - angle_offset  # Left of front
        
        # Adjusting direction based on defined regions
        if target_direction >= 360:
            target_direction -= 360
        elif target_direction < 0:
            target_direction += 360
        
        self.target_direction = target_direction
        self.target_distance = target_distance
        
        return target_direction, target_distance

class LidarProcessor:
    """Processes LIDAR data from ROS2 LaserScan messages with dynamic object detection"""

    def __init__(self):
        self.scan_data = {}  # Dictionary to store scan data (angle -> distance)
        self.lock = threading.Lock()

        # Obstacle status
        self.front_obstacle = False
        self.left_obstacle = False
        self.right_obstacle = False
        self.back_obstacle = False  # For the back region
        self.danger_zone = False
        self.critical_danger = False

        # Minimum distance for each region
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.back_distance = float('inf')  # For the back region

        self.target_direction = None  # The direction of the target
        self.target_distance = None   # The distance to the target

        # Timestamp for the last scan
        self.last_scan_time = 0

        # Save raw scan for visualization
        self.last_scan_msg = None

        # Target information (from UWB)
        self.target_direction = None
        self.target_distance = None

        # Dynamic object detector
        self.dynamic_detector = DynamicObjectDetector()

        # Improved filtering
        self.moving_avg_window = 2
        self.distance_history = {}

    def _is_in_front_region(self, angle):
        """Check if angle is in front region (330° to 360° or 0° to 30°)"""
        return (330 <= angle <= 360) or (0 <= angle <= 30)

    def _is_in_back_region(self, angle):
        """Check if angle is in back region (150° to 210°)"""
        return 150 <= angle <= 210
    
    def set_target_info(self, direction, distance):
        """Set the target's direction and distance for LIDAR processing"""
        self.target_direction = direction
        self.target_distance = distance
        print(f"Target info set: direction={direction}, distance={distance}")

    def process_scan(self, scan_msg):
        """Process ROS2 LaserScan message with dynamic object detection"""
        with self.lock:
            self.last_scan_msg = scan_msg
            self.last_scan_time = time.time()
            self.scan_data.clear()

            ranges = scan_msg.ranges
            angle_increment = scan_msg.angle_increment
            angle_min = scan_msg.angle_min

            step_size = 10  # Adjust step size as needed

            for i in range(0, len(ranges), step_size):
                distance = ranges[i]

                if distance < 0.01 or distance > 10.0 or math.isinf(distance):
                    continue

                angle_rad = angle_min + (i * angle_increment)
                angle_deg = int(math.degrees(angle_rad) % 360)
                distance_mm = int(distance * 1000)

                self.scan_data[angle_deg] = distance_mm

            # Apply the moving average filter to smooth the scan data
            self.scan_data = self._filter_lidar_data(self.scan_data)

            # Analyze obstacles with dynamic object detection
            self._analyze_obstacles_with_dynamic_detection()

    def _filter_lidar_data(self, scan_data):
        """Filter LIDAR data using a moving average to smooth out readings"""
        filtered_data = {}

        for angle, distance in scan_data.items():
            if angle not in self.distance_history:
                self.distance_history[angle] = deque()

            # Append the new distance to the history
            self.distance_history[angle].append(distance)

            # Keep only the last `self.moving_avg_window` number of distances
            if len(self.distance_history[angle]) > self.moving_avg_window:
                self.distance_history[angle].popleft()

            # Calculate the moving average
            filtered_data[angle] = sum(self.distance_history[angle]) / len(self.distance_history[angle])

        return filtered_data

    def _analyze_obstacles_with_dynamic_detection(self):
        """Analyze obstacles using dynamic object detection"""
        if not self.scan_data:
            return

        # Reset obstacle status
        self.front_obstacle = self.left_obstacle = self.right_obstacle = False
        self.back_obstacle = False  # Reset the back obstacle status
        self.danger_zone = self.critical_danger = False
        self.front_distance = self.left_distance = self.right_distance = float('inf')
        self.back_distance = float('inf')  # Reset back distance

        # Check for dynamic objects in path
        waiting_for_dynamic, dynamic_distance = self.dynamic_detector.check_dynamic_objects_in_path(self.scan_data)

        # If waiting for dynamic object, set flags and stop movement
        if waiting_for_dynamic:
            self.front_obstacle = True
            self.danger_zone = True
            if dynamic_distance < CRITICAL_DANGER_THRESHOLD:
                self.critical_danger = True
            print(f"WAITING FOR DYNAMIC OBJECT TO MOVE (distance: {dynamic_distance:.0f}mm)")
            return

        # Check for static objects for avoidance
        static_obstacles = self.dynamic_detector.get_static_objects_for_avoidance()

        # Check obstacles in the front, left, right, and back regions
        for angle, distance in self.scan_data.items():
            # Skip target exclusion if it's the target object
            if (self.target_direction is not None and
                    abs(angle - self.target_direction) < TARGET_EXCLUSION_ANGLE):
                continue  # Skip this object, as it is the target, not an obstacle

            # Check if the object is in the front region
            if self._is_in_front_region(angle):
                if distance < self.front_distance:
                    self.front_distance = distance
                if distance < SCAN_THRESHOLD:
                    self.front_obstacle = True
                if distance < DANGER_THRESHOLD:
                    self.danger_zone = True
                if distance < CRITICAL_DANGER_THRESHOLD:
                    self.critical_danger = True

            # Check if the object is in the back region
            elif self._is_in_back_region(angle):
                if distance < self.back_distance:
                    self.back_distance = distance  # Update back distance
                if distance < SCAN_THRESHOLD:
                    self.back_obstacle = True  # Mark back as blocked

            # Check the right region (31° to 140°)
            elif 31 <= angle <= 140:  # Right region (31° to 140°)
                if distance < self.right_distance:
                    self.right_distance = distance
                if distance < SCAN_THRESHOLD:
                    self.right_obstacle = True

            # Check the left region (220° to 329°)
            elif 220 <= angle <= 329:  # Left region (220° to 329°)
                if distance < self.left_distance:
                    self.left_distance = distance
                if distance < SCAN_THRESHOLD:
                    self.left_obstacle = True

    def get_obstacle_status(self):
        """Get current obstacle status including dynamic object information"""
        with self.lock:
            scan_age = time.time() - self.last_scan_time
            data_valid = scan_age < 0.6

            # Check for dynamic objects
            waiting_for_dynamic = self.dynamic_detector.waiting_for_dynamic_object

            status = {
                'front': {
                    'obstacle': self.front_obstacle if data_valid else False,
                    'distance': self.front_distance if data_valid else float('inf')
                },
                'left': {
                    'obstacle': self.left_obstacle if data_valid else False,
                    'distance': self.left_distance if data_valid else float('inf')
                },
                'right': {
                    'obstacle': self.right_obstacle if data_valid else False,
                    'distance': self.right_distance if data_valid else float('inf')
                },
                'back': {
                    'obstacle': self.back_obstacle if data_valid else False,
                    'distance': self.back_distance if data_valid else float('inf')
                },
                'danger_zone': self.danger_zone if data_valid else False,
                'critical_danger': self.critical_danger if data_valid else False,
                'data_valid': data_valid,
                'scan_points': len(self.scan_data) if data_valid else 0,
                'scan_age': scan_age,
                'waiting_for_dynamic': waiting_for_dynamic,
                'dynamic_objects': len(self.dynamic_detector.dynamic_objects),
                'static_objects': len(self.dynamic_detector.static_objects)
            }

            return status
    
    def get_safe_direction(self):
        """Tentukan arah aman untuk bergerak berdasarkan rintangan"""
        status = self.get_obstacle_status()

        if not status['data_valid']:
            print("Warning: LIDAR data not valid or too old")
            return None

        # Jika menunggu objek dinamis, berhenti dulu
        if status['waiting_for_dynamic']:
            return "STOP_DYNAMIC"

        if status['critical_danger']:
            return "STOP"
        
        # Jika tidak ada rintangan di depan, kiri, kanan, dan belakang, lanjutkan maju
        if not status['front']['obstacle'] and not status['left']['obstacle'] and not status['right']['obstacle'] and not status['back']['obstacle']:
            return "FORWARD"
        
        if status['front']['obstacle']:
            if not status['left']['obstacle'] and not status['right']['obstacle']:
                if status['left']['distance'] > status['right']['distance']:
                    return "LEFT"
                else:
                    return "RIGHT"
            elif not status['left']['obstacle']:
                return "LEFT"
            elif not status['right']['obstacle']:
                return "RIGHT"
            else:
                distances = [
                    ('FORWARD', status['front']['distance']),
                    ('LEFT', status['left']['distance']),
                    ('RIGHT', status['right']['distance']),
                    ('BACK', status['back']['distance'])
                ]
                best_direction = max(distances, key=lambda x: x[1])[0]
                return best_direction

        return "FORWARD"


# Robot Controller with Enhanced Safety and Dynamic Object Handling
class RobotController:
    """Controls robot movement with enhanced safety for industrial use"""
    
    def __init__(self, r_wheel_port, l_wheel_port):
        # Motor controllers
        self.right_motor = MotorControl(device=r_wheel_port)
        self.left_motor = MotorControl(device=l_wheel_port)
        self.right_motor.set_drive_mode(1, 2)
        self.left_motor.set_drive_mode(1, 2)
        
        # Speed configuration
        self.speed = DEFAULT_SPEED
        self.rotation_factor = ROTATION_FACTOR
        self.stop_threshold = STOP_THRESHOLD
        
        # Status flags
        self.obstacle_avoidance_active = False
        self.last_command_time = time.time()
        self.current_direction = "STOP"
        self.emergency_stop = False
        self.waiting_for_dynamic_object = False
        
        # Command batching
        self.last_command = (0, 0)
        self.command_threshold = 5
        self.last_command_time = 0
        self.min_command_interval = 0.02
    
    def move(self, right_speed, left_speed):
        """Move dengan command batching"""
        current_time = time.time()
        
        # Skip jika command sama atau terlalu sering
        if (abs(right_speed - self.last_command[0]) < self.command_threshold and
            abs(left_speed - self.last_command[1]) < self.command_threshold and
            current_time - self.last_command_time < self.min_command_interval):
            return
        
        # Send command
        self.right_motor.send_rpm(1, int(right_speed))
        self.left_motor.send_rpm(1, int(left_speed))
        
        self.last_command = (right_speed, left_speed)
        self.last_command_time = current_time
    
    def stop(self):
        """Stop the robot"""
        self.move(0, 0)
        self.current_direction = "STOP"
    
    def check_critical_obstacles(self, lidar):
        """Check if there are any critical obstacles that require immediate action"""
        status = lidar.get_obstacle_status()
        
        # Check for dynamic objects first
        if status['waiting_for_dynamic']:
            print("WAITING FOR DYNAMIC OBJECT - SAFETY STOP")
            self.stop()
            self.waiting_for_dynamic_object = True
            return True
        
        # Only act if LIDAR data is valid and we have a critical danger
        if status['data_valid'] and status['critical_danger']:
            print("CRITICAL DANGER DETECTED - EMERGENCY STOP")
            self.stop()
            self.emergency_stop = True
            return True
        
        self.emergency_stop = False
        self.waiting_for_dynamic_object = False
        return False
    
    def handle_obstacle_avoidance(self, lidar, target_in_view=False):
        """Process LIDAR data for obstacle avoidance with dynamic object handling"""
        status = lidar.get_obstacle_status()
        
        # Skip full processing if data isn't valid
        if not status['data_valid']:
            if status['scan_age'] > 0.01:
                print(f"LIDAR data too old: {status['scan_age']:.2f} seconds")
            return False
        
        # If we're close to the target and the target is in view, ignore obstacles
        if target_in_view and lidar.target_distance and lidar.target_distance < STOP_THRESHOLD * 10:
            if not status['critical_danger'] and not status['waiting_for_dynamic']:
                print("Target in close proximity, ignoring non-critical static obstacles")
                return False

        # Handle dynamic objects first (highest priority)
        if status['waiting_for_dynamic']:
            print("DYNAMIC OBJECT IN PATH - WAITING FOR CLEARANCE")
            self.stop()
            self.obstacle_avoidance_active = True
            return True

        # Handle critical danger cases immediately
        if status['critical_danger']:
            print("CRITICAL DANGER DETECTED - STOPPING")
            self.stop()
            self.obstacle_avoidance_active = True
            return True

        # For non-critical cases, only intervene if needed
        if (status['danger_zone'] or 
            (status['front']['obstacle'] and status['front']['distance'] < DANGER_THRESHOLD * 1.5)):

            safe_direction = lidar.get_safe_direction()
            print(f"Safe direction: {safe_direction}")

            if safe_direction == "STOP" or safe_direction == "STOP_DYNAMIC":
                print("DANGER ZONE DETECTED - STOPPING")
                self.stop()
                self.obstacle_avoidance_active = True
                return True

            elif safe_direction == "LEFT":
                print("Obstacle avoidance: Turning LEFT (avoiding static obstacle)")
                self.move(-self.speed, self.speed / (self.rotation_factor * 2))
                self.current_direction = "LEFT"
                self.obstacle_avoidance_active = True
                return True

            elif safe_direction == "RIGHT":
                print("Obstacle avoidance: Turning RIGHT (avoiding static obstacle)")
                self.move(-self.speed / (self.rotation_factor * 2), self.speed)
                self.current_direction = "RIGHT"
                self.obstacle_avoidance_active = True
                return True
        
        # Additional safety checks for side obstacles
        if status['left']['distance'] < DANGER_THRESHOLD:
            print("Left side is blocked, rotating right")
            self.move(-self.speed / (self.rotation_factor * 2), self.speed)
            self.current_direction = "RIGHT"
            self.obstacle_avoidance_active = True
            return True
        
        elif status['right']['distance'] < DANGER_THRESHOLD:
            print("Right side is blocked, rotating left")
            self.move(-self.speed, self.speed / (self.rotation_factor * 2))
            self.current_direction = "LEFT"
            self.obstacle_avoidance_active = True
            return True
        
        # If we get here, no immediate obstacle avoidance is required
        self.obstacle_avoidance_active = False
        self.waiting_for_dynamic_object = False
        return False
    
    def process_uwb_control(self, uwb_distances):
        """Process UWB data to control robot movement"""
        A0, A1, A2 = uwb_distances['A0'], uwb_distances['A1'], uwb_distances['A2']
        
        # Print UWB distances
        print("\n--- UWB Distances (cm) ---")
        print(f"A0: {A0:.2f} | A1: {A1:.2f} | A2: {A2:.2f}")
        
        # Logic for UWB-based control
        if A0 <= self.stop_threshold:
            print(f"Target reached (A0 <= {self.stop_threshold} cm). Stopping.")
            self.stop()
            return
        
        # Enhanced UWB control logic with smoother transitions
        diff = A2 - A1
        if abs(diff) < 15:  # Target is roughly straight ahead
            print("Move Forward - Target centered")
            self.move(-self.speed, self.speed)
            self.current_direction = "FORWARD"
        elif A0 < 150:  # When close to target, make sharper turns
            if diff < 0:  # A2 < A1 - target is to the right
                turn_ratio = min(1.0, abs(diff) / 40.0)
                right_speed = -self.speed * (1.0 - turn_ratio * 0.8)
                left_speed = self.speed
                print(f"Close target right turn (ratio: {turn_ratio:.2f})")
                self.move(right_speed, left_speed)
                self.current_direction = "RIGHT"
            else:  # A1 < A2 - target is to the left
                turn_ratio = min(1.0, abs(diff) / 40.0)
                right_speed = -self.speed
                left_speed = self.speed * (1.0 - turn_ratio * 0.8)
                print(f"Close target left turn (ratio: {turn_ratio:.2f})")
                self.move(right_speed, left_speed)
                self.current_direction = "LEFT"
        elif (A0 > A1 or A0 > A2):
            print("Rotasi")
            self.move(self.speed / self.rotation_factor, self.speed / self.rotation_factor)
        else:  # Further from target, smoother turns
            if diff < 0:  # A2 < A1 - target is to the right
                print("Turn right - target to right")
                self.move(-self.speed / self.rotation_factor, self.speed)
                self.current_direction = "RIGHT"
            else:  # A1 < A2 - target is to the left
                print("Turn left - target to left")
                self.move(-self.speed, self.speed / self.rotation_factor)
                self.current_direction = "LEFT"
    
    def process_control(self, uwb_distances, lidar):
        """Main control logic integrating UWB and LIDAR data with enhanced safety"""
        # First, check for critical obstacles that require immediate action
        if self.check_critical_obstacles(lidar):
            return  # Critical obstacle or dynamic object detected, stop immediately
        
        # Determine if we have a clear view of the target and update LIDAR with target info
        target_direction, target_distance = uwb_tracker.estimate_target_direction(uwb_distances)
        lidar.set_target_info(target_direction, target_distance)
        
        # Check if target is in close proximity and centered
        target_in_view = (
            uwb_distances['A0'] < 200 and  # Within 2 meters
            abs(uwb_distances['A1'] - uwb_distances['A2']) < 30  # Roughly centered
        )
        
        # Enhanced safety: Run obstacle avoidance with dynamic object detection
        obstacle_action_needed = self.handle_obstacle_avoidance(lidar, target_in_view)
        
        # If no critical obstacle or dynamic object, use UWB for following
        if not obstacle_action_needed and not self.emergency_stop and not self.waiting_for_dynamic_object:
            self.process_uwb_control(uwb_distances)

# FollowingRobotNode Class
class FollowingRobotNode(Node):
    def __init__(self):
        super().__init__('following_robot_node')

        # Get the dynamic IP address from Wi-Fi interface
        self.udp_ip = get_ip_from_wifi()
        if not self.udp_ip:
            raise ValueError("Unable to retrieve IP address from Wi-Fi interface.")

        # Optionally, modify the last part of the IP address
        target_last_digit = 128
        self.udp_ip = get_ip_from_subnet(self.udp_ip, target_last_digit)

        print(f"Robot will use dynamic IP: {self.udp_ip}")
        
        self.udp_port = 5005
        
        # Configuration
        self.r_wheel_port = "/dev/ttyRS485-1"
        self.l_wheel_port = "/dev/ttyRS485-2"
        
        # Performance configuration
        self.control_frequency = CONTROL_FREQUENCY
        self.lidar_skip_frames = LIDAR_SKIP_FRAMES
        self.uwb_timeout = UWB_TIMEOUT
        self.max_loop_time = MAX_LOOP_TIME
        
        # Frame counter
        self.lidar_frame_counter = 0
        self.last_control_update = 0
        self.last_loop_time = 0
        self._processing_lidar = False
        
        # Initialize components
        self.uwb_tracker = UWBTracker()
        self.lidar = LidarProcessor()
        self.controller = RobotController(self.r_wheel_port, self.l_wheel_port)

        self.lidar_cb_group = ReentrantCallbackGroup()
        
        # LIDAR subscription
        lidar_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=LIDAR_BUFFER_SIZE
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback_optimized,
            qos_profile=lidar_qos,
            callback_group=self.lidar_cb_group
        )
        
        # UWB setup
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, UWB_BUFFER_SIZE)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.settimeout(self.uwb_timeout)
        
        # UWB data
        self.raw_uwb_distances = {'A0': 1000, 'A1': 1000, 'A2': 1000}
        self.corrected_uwb_distances = {'A0': 1000, 'A1': 1000, 'A2': 1000}
        
        # Status control
        self.running = True
        self.last_uwb_update = 0
        
        # Control timer
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency,
            self.control_loop
        )
        
        # Performance monitoring
        self.last_lidar_time = time.time()
        self.lidar_update_count = 0
        self.lidar_update_rate = 0
        
        # Log statistics periodically
        self.stats_timer = self.create_timer(2.0, self.log_statistics)
        
        self.get_logger().info(f'Enhanced Safety Following Robot Node started with {self.control_frequency}Hz control frequency')
        self.get_logger().info('Features: Dynamic object detection, Static object avoidance, Industrial safety protocols')

        # Turn on GPIO 17 to indicate the program is running
        gpio_pin_17.on()

    async def scan_callback_optimized(self, msg):
        """Callback LIDAR yang dioptimasi dengan frame skipping"""
        if hasattr(self, '_processing_lidar') and self._processing_lidar:
            return
        
        self._processing_lidar = True
        try:
            if self.lidar_frame_counter % self.lidar_skip_frames == 0:
                self.lidar.process_scan(msg)
            self.lidar_frame_counter += 1
            self.lidar_update_count += 1
        finally:
            self._processing_lidar = False

    def check_critical_obstacles_fast(self):
        """Check critical obstacles dengan minimal processing"""
        if (time.time() - self.lidar.last_scan_time > 0.01 or 
            not hasattr(self.lidar, 'critical_danger')):
            return False
        
        if self.lidar.critical_danger or self.lidar.dynamic_detector.waiting_for_dynamic_object:
            self.controller.stop()
            return True
        return False

    def process_uwb_data_fast(self):
        """Process UWB data dengan timeout minimal"""
        try:
            self.sock.setblocking(False)
            ready = select.select([self.sock], [], [], self.uwb_timeout)
            if ready[0]:
                data, addr = self.sock.recvfrom(UWB_BUFFER_SIZE)
                parts = data.decode().split(",")
                
                if len(parts) >= 3:
                    self.raw_uwb_distances = {
                        'A0': float(parts[0]),
                        'A1': float(parts[1]),
                        'A2': float(parts[2])
                    }
                    
                    self.corrected_uwb_distances = self.uwb_tracker.apply_bias_correction(
                        self.raw_uwb_distances
                    )
                    self.last_uwb_update = time.time()
                    return True
                    
        except (socket.error, ValueError, IndexError):
            pass
        
        return False

    def execute_control_decision(self):
        """Execute control decision berdasarkan data sensor terbaru"""
        uwb_data_age = time.time() - self.last_uwb_update
        uwb_data_valid = uwb_data_age < 0.5
        
        if uwb_data_valid:
            self.controller.process_control(self.corrected_uwb_distances, self.lidar)
        else:
            self.controller.handle_obstacle_avoidance(self.lidar)
    
    def log_statistics(self):
        """Log statistics about sensor update rates and safety status"""
        current_time = time.time()
        update_interval = current_time - self.last_lidar_time
        if update_interval > 0:
            self.lidar_update_rate = self.lidar_update_count / update_interval
            self.last_lidar_time = current_time
            self.lidar_update_count = 0
            
            # Enhanced logging with safety information
            status = self.lidar.get_obstacle_status()
            self.get_logger().info(
                f"LIDAR: {self.lidar_update_rate:.2f}Hz | "
                f"UWB age: {time.time() - self.last_uwb_update:.2f}s | "
                f"Dynamic objs: {status['dynamic_objects']} | "
                f"Static objs: {status['static_objects']} | "
                f"Waiting for dynamic: {status['waiting_for_dynamic']}"
            )
    
    def control_loop(self):
        """Control loop yang dioptimasi untuk real-time dengan enhanced safety"""
        if not self.running:
            return
        start_time = time.time()
        try:
            # PRIORITAS 1: Critical obstacle check (termasuk dynamic objects)
            if self.check_critical_obstacles_fast():
                return
            
            # PRIORITAS 2: UWB data processing
            uwb_updated = self.process_uwb_data_fast()
            
            # PRIORITAS 3: Control decision
            if uwb_updated or time.time() - self.last_control_update > 0.05:
                self.execute_control_decision()
                self.last_control_update = time.time()
        except Exception as e:
            gpio_pin_27.on()
            self.get_logger().error(f"Control loop error: {e}")
        
        # Monitor performa loop
        loop_time = time.time() - start_time
        if loop_time > 0.015:
            self.get_logger().warn(f"Slow control loop: {loop_time * 1000:.1f}ms")

    def stop(self):
        """Stop the robot and clean up"""
        self.running = False
        self.controller.stop()
        self.sock.close()
        gpio_pin_17.off()
        gpio_pin_27.off()
        self.get_logger().info("Enhanced Safety Robot systems shut down.")

def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create global reference to UWB tracker
    global uwb_tracker
    uwb_tracker = UWBTracker()
    
    # Create node
    node = FollowingRobotNode()
    
    # Create multi-threaded executor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        # Run until interrupted
        executor.spin()
    except KeyboardInterrupt:
        print("\nUser interrupted. Shutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Clean up
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
