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


# Konfigurasi Global EXISTING
SCAN_THRESHOLD = 3000  # Jarak aman dalam mm
DANGER_THRESHOLD = 500  # Jarak bahaya dalam mm
MIN_VALID_DISTANCE = 100  # Jarak minimum valid (mm) untuk menghindari noise
CRITICAL_DANGER_THRESHOLD = 300  # Jarak kritis untuk override UWB (mm)

# Motor speed configuration
DEFAULT_SPEED = 65
ROTATION_FACTOR = 2
STOP_THRESHOLD = 80  # cm

# TAMBAHKAN KONFIGURASI PERFORMA REAL-TIME DI SINI
# =====================================================
# Konfigurasi performa real-time
CONTROL_FREQUENCY = 1000  # Hz
LIDAR_SKIP_FRAMES = 1    # Process setiap 2 frame
UWB_TIMEOUT = 0.0001      # 1ms timeout
MAX_LOOP_TIME = 0.0015    # 15ms warning threshold

# Buffer sizes
LIDAR_BUFFER_SIZE = 1
UWB_BUFFER_SIZE = 1024
# =====================================================

# Definisi sudut untuk deteksi rintangan (existing code continues...)
FRONT_REGION = [(330, 360), (0, 30)]
RIGHT_REGION = (31, 140)
LEFT_REGION  = (220,329)
TARGET_EXCLUSION_ANGLE = 10

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
            'A1': 1.1,  # Scale factor
            'A2': 1.1   # Scale factor
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
        # Simple triangulation to estimate direction
        # If A1 < A2, target is more to the left
        # If A2 < A1, target is more to the right
        A0, A1, A2 = distances['A0'], distances['A1'], distances['A2']
        
        # Convert to mm for consistency with LIDAR
        target_distance = A0 * 10  # cm to mm
        
        # Improved direction estimation using the difference between A1 and A2
        diff = A2 - A1
        if abs(diff) < 20:  # Target is roughly straight ahead
            target_direction = 270  # Front is 270 degrees
        elif diff < 0:  # A2 < A1, target is more to the right
            # Proportionally adjust the angle based on difference magnitude
            angle_offset = min(45, abs(diff) * 1.5)  # Scale factor to tune sensitivity
            target_direction = 270 + angle_offset  # Right of front
        else:  # A1 < A2, target is more to the left
            # Proportionally adjust the angle based on difference magnitude
            angle_offset = min(45, abs(diff) * 1.5)  # Scale factor to tune sensitivity
            target_direction = 270 - angle_offset  # Left of front
        
        self.target_direction = target_direction
        self.target_distance = target_distance
        
        return target_direction, target_distance

class LidarProcessor:
    """Processes LIDAR data from ROS2 LaserScan messages"""
    
    def __init__(self):
        self.scan_data = {}  # Dictionary untuk menyimpan data scan (angle -> distance)
        self.lock = threading.Lock()
        
        # Status rintangan
        self.front_obstacle = False
        self.left_obstacle = False
        self.right_obstacle = False
        self.danger_zone = False
        self.critical_danger = False  # New flag for critical danger (immediate stop)
        
        # Jarak minimum di setiap region
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        
        # Timestamp data terakhir
        self.last_scan_time = 0
        
        # Save raw scan for visualization
        self.last_scan_msg = None
        
        # Target information (from UWB)
        self.target_direction = None
        self.target_distance = None
        
        # Improved filtering
        self.moving_avg_window = 4
        self.distance_history = {}  # Store recent distance values for filtering
    
    def set_target_info(self, direction, distance):
        """Set target information from UWB to avoid treating target as obstacle"""
        self.target_direction = direction
        self.target_distance = distance
    
    def process_scan(self, scan_msg):
        """Process ROS2 LaserScan message dengan optimasi performa"""
        with self.lock:
            self.last_scan_msg = scan_msg
            self.last_scan_time = time.time()
            self.scan_data.clear()
            
            ranges = scan_msg.ranges
            angle_increment = scan_msg.angle_increment
            angle_min = scan_msg.angle_min
            
            # OPTIMASI: Downsampling agresif - ambil setiap 10 data
            step_size = 10  # Meningkat dari 5 ke 10 untuk performa lebih cepat
            
            for i in range(0, len(ranges), step_size):
                distance = ranges[i]
                
                # Skip invalid measurements dengan check cepat
                if distance < 0.01 or distance > 10.0 or math.isinf(distance):
                    continue
                
                angle_rad = angle_min + (i * angle_increment)
                angle_deg = int(math.degrees(angle_rad) % 360)
                distance_mm = int(distance * 1000)
                
                # Simplified filtering - hanya simpan nilai terbaru
                self.scan_data[angle_deg] = distance_mm

            # Analisis obstacle dengan algoritma yang disederhanakan
            self._analyze_obstacles_fast()


    
    def _is_angle_in_region(self, angle, region):
        """Check if angle is in specified region, handling wraparound at 360°"""
        if isinstance(region, tuple):
            start, end = region
            if start <= end:
                return start <= angle <= end
            else:  # Wraparound case (e.g. 350° to 10°)
                return angle >= start or angle <= end
        else:  # For LEFT_REGION which is a list of tuples
            for r in region:
                start, end = r
                if start <= end:
                    if start <= angle <= end:
                        return True
                else:  # Wraparound case
                    if angle >= start or angle <= end:
                        return True
            return False
    
    def _is_angle_near_target(self, angle):
        """Check if this angle is near the UWB target direction"""
        if self.target_direction is None:
            return False
            
        # Calculate angular distance to target
        angular_dist = min(
            abs(angle - self.target_direction),
            abs(angle - (self.target_direction + 360)),
            abs((angle + 360) - self.target_direction)
        )
        
        # If angle is within exclusion zone of target, return True
        return angular_dist < TARGET_EXCLUSION_ANGLE
    
    def _analyze_obstacles_fast(self):
        """Analisis obstacle dengan algoritma yang dipercepat"""
        if not self.scan_data:
            return
        
        # Reset status dengan operasi batch
        self.front_obstacle = self.left_obstacle = self.right_obstacle = False
        self.danger_zone = self.critical_danger = False
        self.front_distance = self.left_distance = self.right_distance = float('inf')
        
        # Pre-compute region checks untuk efisiensi
        front_angles = set(range(330, 360)) | set(range(0, 30))
        right_angles = set(range(30, 151))
        left_angles = set(range(210, 330))
        
        # Single pass analysis
        for angle, distance in self.scan_data.items():
            # Skip target exclusion check jika tidak ada target
            if (self.target_direction is not None and 
                abs(angle - self.target_direction) < TARGET_EXCLUSION_ANGLE):
                continue
            
            # Batch processing untuk setiap region
            if angle in front_angles:
                if distance < self.front_distance:
                    self.front_distance = distance
                if distance < SCAN_THRESHOLD:
                    self.front_obstacle = True
                if distance < DANGER_THRESHOLD:
                    self.danger_zone = True
                if distance < CRITICAL_DANGER_THRESHOLD:
                    self.critical_danger = True
                    
            elif angle in right_angles:
                if distance < self.right_distance:
                    self.right_distance = distance
                if distance < SCAN_THRESHOLD:
                    self.right_obstacle = True
                    
            elif angle in left_angles:
                if distance < self.left_distance:
                    self.left_distance = distance
                if distance < SCAN_THRESHOLD:
                    self.left_obstacle = True

    
    def get_obstacle_status(self):
        """Get current obstacle status"""
        with self.lock:
            scan_age = time.time() - self.last_scan_time
            data_valid = scan_age < 0.6 # Reduced from 1.0 to 0.5 seconds for faster response
            
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
                'danger_zone': self.danger_zone if data_valid else False,
                'critical_danger': self.critical_danger if data_valid else False,
                'data_valid': data_valid,
                'scan_points': len(self.scan_data) if data_valid else 0,
                'scan_age': scan_age
            }
            
            return status
    
    def get_safe_direction(self):
        """Determine the safest direction to move based on obstacles"""
        status = self.get_obstacle_status()
        
        if not status['data_valid']:
            print("Warning: LIDAR data not valid or too old")
            return None
        
        if status['critical_danger']:
            return "STOP"
            
        if not status['front']['obstacle'] and not status['left']['obstacle'] and not status['right']['obstacle']:
            return "FORWARD"  # No obstacles
            
        # If front is blocked
        if status['front']['obstacle']:
            # Both sides clear - choose the one with more space
            if not status['left']['obstacle'] and not status['right']['obstacle']:
                if status['left']['distance'] > status['right']['distance']:
                    return "LEFT"
                else:
                    return "RIGHT"
            # Only left is clear
            elif not status['left']['obstacle']:
                return "LEFT"
            # Only right is clear
            elif not status['right']['obstacle']:
                return "RIGHT"
            # All directions blocked - choose least bad option
            else:
                distances = [
                    ('FORWARD', status['front']['distance']),
                    ('LEFT', status['left']['distance']),
                    ('RIGHT', status['right']['distance'])
                ]
                best_direction = max(distances, key=lambda x: x[1])[0]
                return best_direction
        
        # Front is clear but sides have obstacles
        return "FORWARD"

class RobotController:
    """Controls robot movement based on UWB and LIDAR data"""
    
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

        # Command batching untuk mengurangi serial communication
        self.last_command = (0, 0)
        self.command_threshold = 5  # Minimal perubahan speed
        self.last_command_time = 0
        self.min_command_interval = 0.02  # 50Hz max command rate
    
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
        
        # Only act if LIDAR data is valid and we have a critical danger
        if status['data_valid'] and status['critical_danger']:
            print("CRITICAL DANGER DETECTED - EMERGENCY STOP")
            self.stop()
            self.emergency_stop = True
            return True
        
        self.emergency_stop = False
        return False
    
    def handle_obstacle_avoidance(self, lidar, target_in_view=False):
        """Process LIDAR data for obstacle avoidance"""
        # Get obstacle status and safe direction
        status = lidar.get_obstacle_status()
        
        # Skip full processing if data isn't valid
        if not status['data_valid']:
            if status['scan_age'] > 0.01:  # Reduced threshold for warning
                print(f"LIDAR data too old: {status['scan_age']:.2f} seconds")
            return False
        
        # If we're close to the target and the target is in view, ignore obstacles
        # except for critical danger situations
        if target_in_view and lidar.target_distance and lidar.target_distance < STOP_THRESHOLD * 10:
            if not status['critical_danger']:
                print("Target in close proximity, ignoring non-critical obstacles")
                return False
        
        # For debugging - print obstacle status only if valid
        print(f"\n--- LIDAR Status (Age: {status['scan_age']:.3f}s) ---")
        print(f"Front: {'BLOCKED' if status['front']['obstacle'] else 'clear'} ({status['front']['distance']}mm)")
        print(f"Left: {'BLOCKED' if status['left']['obstacle'] else 'clear'} ({status['left']['distance']}mm)")
        print(f"Right: {'BLOCKED' if status['right']['obstacle'] else 'clear'} ({status['right']['distance']}mm)")
        print(f"Danger Zone: {'YES' if status['danger_zone'] else 'NO'}")
        print(f"Critical Danger: {'YES' if status['critical_danger'] else 'NO'}")
        print(f"Points in scan: {status['scan_points']}")
        
        # Handle critical danger cases immediately
        if status['critical_danger']:
            print("CRITICAL DANGER DETECTED - STOPPING")
            self.stop()
            self.obstacle_avoidance_active = True
            return True
            
        # For non-critical cases, only intervene if needed
        # We'll only do full obstacle avoidance if there's a danger zone
        # or if the front obstacle is very close
        if (status['danger_zone'] or 
            (status['front']['obstacle'] and status['front']['distance'] < DANGER_THRESHOLD * 1.5)):
            
            safe_direction = lidar.get_safe_direction()
            print(f"Safe direction: {safe_direction}")
            
            if safe_direction == "STOP":
                print("DANGER ZONE DETECTED - STOPPING")
                self.stop()
                self.obstacle_avoidance_active = True
                return True
                
            elif safe_direction == "LEFT":
                print("Obstacle avoidance: Turning LEFT")
                self.move(-self.speed, self.speed/(self.rotation_factor*2))
                self.current_direction = "LEFT"
                self.obstacle_avoidance_active = True
                return True
                
            elif safe_direction == "RIGHT":
                print("Obstacle avoidance: Turning RIGHT")
                self.move(-self.speed/(self.rotation_factor*2), self.speed)
                self.current_direction = "RIGHT"
                self.obstacle_avoidance_active = True
                return True

            # New condition for turning if the left or right distance is less than 300mm
            # Add this block to rotate the wheels based on the distances
            if status['left']['distance'] < DANGER_THRESHOLD:  # Left distance less than DANGER_THRESHOLDmm
                print("Left side is blocked, rotating right")
                self.move(-self.speed/(self.rotation_factor*2), self.speed)  # Rotate right
                self.current_direction = "RIGHT"
                self.obstacle_avoidance_active = True
                return True
            
            elif status['right']['distance'] < DANGER_THRESHOLD:  # Right distance less than DANGER_THRESHOLDmm
                print("Right side is blocked, rotating left")
                self.move(-self.speed, self.speed/(self.rotation_factor*2))  # Rotate left
                self.current_direction = "LEFT"
                self.obstacle_avoidance_active = True
                return True
        
        # If we get here, no immediate obstacle avoidance is required
        self.obstacle_avoidance_active = False
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
        if abs(diff) < 15:  # Target is roughly straight ahead - tight threshold
            print("Move Forward - Target centered")
            self.move(-self.speed, self.speed)
            self.current_direction = "FORWARD"
        elif A0 < 150:  # When close to target, make sharper turns
            if diff < 0:  # A2 < A1 - target is to the right
                turn_ratio = min(1.0, abs(diff) / 40.0)  # Calculate turn sharpness
                right_speed = -self.speed * (1.0 - turn_ratio * 0.8)
                left_speed = self.speed
                print(f"Close target right turn (ratio: {turn_ratio:.2f})")
                self.move(right_speed, left_speed)
                self.current_direction = "RIGHT"
            else:  # A1 < A2 - target is to the left
                turn_ratio = min(1.0, abs(diff) / 40.0)  # Calculate turn sharpness
                right_speed = -self.speed
                left_speed = self.speed * (1.0 - turn_ratio * 0.8)
                print(f"Close target left turn (ratio: {turn_ratio:.2f})")
                self.move(right_speed, left_speed)
                self.current_direction = "LEFT"
        elif (A0 > A1 or A0 > A2):
            print("Rotasi")
            self.move(self.speed/self.rotation_factor, self.speed/self.rotation_factor) #Rotation
        else:  # Further from target, smoother turns
            if diff < 0:  # A2 < A1 - target is to the right
                print("Turn right - target to right")
                self.move(-self.speed/self.rotation_factor, self.speed)
                self.current_direction = "RIGHT"
            else:  # A1 < A2 - target is to the left
                print("Turn left - target to left")
                self.move(-self.speed, self.speed/self.rotation_factor)
                self.current_direction = "LEFT"
    
    def process_control(self, uwb_distances, lidar):
        """Main control logic integrating UWB and LIDAR data"""
        # First, check for critical obstacles that require immediate action
        if self.check_critical_obstacles(lidar):
            return  # Critical obstacle detected, stop immediately
        
        # Determine if we have a clear view of the target and update LIDAR with target info
        target_direction, target_distance = uwb_tracker.estimate_target_direction(uwb_distances)
        lidar.set_target_info(target_direction, target_distance)
        
        # Check if target is in close proximity and centered
        target_in_view = (
            uwb_distances['A0'] < 200 and  # Within 2 meters
            abs(uwb_distances['A1'] - uwb_distances['A2']) < 30  # Roughly centered
        )
        
        # MODIFIED: Prioritize UWB following, with obstacle avoidance only for critical cases
        # Run lighter obstacle check that only looks for critical dangers
        obstacle_action_needed = self.handle_obstacle_avoidance(lidar, target_in_view)
        
        # If no critical obstacle, use UWB for following (prioritizing UWB)
        if not obstacle_action_needed and not self.emergency_stop:
            self.process_uwb_control(uwb_distances)

class FollowingRobotNode(Node):
    def __init__(self):
        super().__init__('following_robot_node')

        # Get the dynamic IP address from Wi-Fi interface (e.g., wlan0)
        self.udp_ip = get_ip_from_wifi()  # Get the dynamic IP
        if not self.udp_ip:
            raise ValueError("Unable to retrieve IP address from Wi-Fi interface.")

        # Optionally, modify the last part of the IP address
        target_last_digit = 128  # For example, we want the last part of the IP to be 128
        self.udp_ip = get_ip_from_subnet(self.udp_ip, target_last_digit)

        print(f"Robot will use dynamic IP: {self.udp_ip}")
        
        self.udp_port = 5005  # Define your UDP port
        
        # Configuration
        self.r_wheel_port = "/dev/ttyRS485-1"
        self.l_wheel_port = "/dev/ttyRS485-2"
        
        # TAMBAHKAN KONFIGURASI PERFORMA REAL-TIME
        self.control_frequency = CONTROL_FREQUENCY
        self.lidar_skip_frames = LIDAR_SKIP_FRAMES
        self.uwb_timeout = UWB_TIMEOUT
        self.max_loop_time = MAX_LOOP_TIME
        
        # Frame counter untuk skip processing
        self.lidar_frame_counter = 0
        self.last_control_update = 0
        self.last_loop_time = 0
        self._processing_lidar = False
        
        # Initialize components
        self.uwb_tracker = UWBTracker()
        self.lidar = LidarProcessor()
        self.controller = RobotController(self.r_wheel_port, self.l_wheel_port)

        self.lidar_cb_group = ReentrantCallbackGroup()
        
        # Gunakan callback group ini di subscription
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
        
        # UWB and LIDAR setup and the rest of your node's initialization
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, UWB_BUFFER_SIZE)
        self.sock.bind((self.udp_ip, self.udp_port))  # Bind the socket to the dynamic IP address
        self.sock.settimeout(self.uwb_timeout)
        
        # UWB data
        self.raw_uwb_distances = {'A0': 1000, 'A1': 1000, 'A2': 1000}
        self.corrected_uwb_distances = {'A0': 1000, 'A1': 1000, 'A2': 1000}
        
        # Status control
        self.running = True
        self.last_uwb_update = 0
        
        # Timer dengan frekuensi yang dikonfigurasi - GUNAKAN METHOD YANG ADA
        self.control_timer = self.create_timer(
            1.0/self.control_frequency,  # Gunakan CONTROL_FREQUENCY
            self.control_loop  # GUNAKAN METHOD YANG SUDAH ADA
        )
        
        # Performance monitoring
        self.last_lidar_time = time.time()
        self.lidar_update_count = 0
        self.lidar_update_rate = 0
        
        # Log statistics periodically
        self.stats_timer = self.create_timer(2.0, self.log_statistics)
        
        self.get_logger().info(f'Following Robot Node started with {self.control_frequency}Hz control frequency')

        # Turn on GPIO 17 to indicate the program is running
        gpio_pin_17.on()
        # gpio_pin_27.on()  # This will turn on GPIO 27 directly


    def scan_callback_optimized(self, msg):
        """Callback LIDAR yang dioptimasi dengan frame skipping"""
        # Skip processing jika masih memproses data sebelumnya
        if hasattr(self, '_processing_lidar') and self._processing_lidar:
            return
        
        self._processing_lidar = True
        try:
            # Process dengan skip frame untuk performa
            if self.lidar_frame_counter % self.lidar_skip_frames == 0:
                self.lidar.process_scan(msg)
            self.lidar_frame_counter += 1
            
            # Update counter untuk statistik
            self.lidar_update_count += 1
        finally:
            self._processing_lidar = False

    def control_loop_optimized(self):
        """Control loop yang dioptimasi dengan monitoring performa"""
        if not self.running:
            return
        
        start_time = time.time()
        
        try:
            # PRIORITAS 1: Critical obstacle check (paling cepat)
            if self.check_critical_obstacles_fast():
                return  # Stop immediately jika ada bahaya kritis
            
            # PRIORITAS 2: UWB data processing (non-blocking)
            uwb_updated = self.process_uwb_data_fast()
            
            # PRIORITAS 3: Control decision (berdasarkan data terbaru)
            if uwb_updated or time.time() - self.last_control_update > 0.05:
                self.execute_control_decision()
                self.last_control_update = time.time()
                
        except Exception as e:
            self.get_logger().error(f"Control loop error: {e}")
        
        # Monitor performa loop
        loop_time = time.time() - start_time
        self.last_loop_time = loop_time
        
        if loop_time > self.max_loop_time:
            self.get_logger().warn(f"Slow control loop: {loop_time*1000:.1f}ms (threshold: {self.max_loop_time*1000:.1f}ms)")

    def check_critical_obstacles_fast(self):
        """Check critical obstacles dengan minimal processing"""
        # Hanya check jika data LIDAR valid dan fresh
        if (time.time() - self.lidar.last_scan_time > 0.01 or 
            not hasattr(self.lidar, 'critical_danger')):
            return False
        
        if self.lidar.critical_danger:
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
                    
                    # Apply correction
                    self.corrected_uwb_distances = self.uwb_tracker.apply_bias_correction(
                        self.raw_uwb_distances
                    )
                    self.last_uwb_update = time.time()
                    return True
                    
        except (socket.error, ValueError, IndexError):
            pass  # Ignore errors untuk performa
        
        return False

    def execute_control_decision(self):
        """Execute control decision berdasarkan data sensor terbaru"""
        # Calculate UWB data age
        uwb_data_age = time.time() - self.last_uwb_update
        uwb_data_valid = uwb_data_age < 0.5
        
        # Process control based on sensor data
        if uwb_data_valid:
            # We have valid UWB data - use integrated control with UWB priority
            self.controller.process_control(self.corrected_uwb_distances, self.lidar)
        else:
            # No valid UWB data - just do basic obstacle avoidance
            self.controller.handle_obstacle_avoidance(self.lidar)

    
    def log_statistics(self):
        """Log statistics about sensor update rates"""
        # Calculate and log LIDAR update rate
        current_time = time.time()
        update_interval = current_time - self.last_lidar_time
        if update_interval > 0:
            self.lidar_update_rate = self.lidar_update_count / update_interval
            self.last_lidar_time = current_time
            self.lidar_update_count = 0
            
            self.get_logger().info(f"LIDAR update rate: {self.lidar_update_rate:.2f} Hz")
            self.get_logger().info(f"UWB data age: {time.time() - self.last_uwb_update:.2f}s")
    
    def control_loop(self):
        """Control loop yang dioptimasi untuk real-time"""
        if not self.running:
            return
        start_time = time.time()
        try:
            # PRIORITAS 1: Critical obstacle check (paling cepat)
            if self.check_critical_obstacles_fast():
                return  # Stop immediately jika ada bahaya kritis
            # PRIORITAS 2: UWB data processing (non-blocking)
            uwb_updated = self.process_uwb_data_fast()
            # PRIORITAS 3: Control decision (berdasarkan data terbaru)
            if uwb_updated or time.time() - self.last_control_update > 0.05:
                self.execute_control_decision()
                self.last_control_update = time.time()
        except Exception as e:
            # On error, turn on GPIO 27 to indicate error
            gpio_pin_27.on()
            self.get_logger().error(f"Control loop error: {e}")
        # Monitor performa loop
        loop_time = time.time() - start_time
        if loop_time > 0.015:  # Warn jika loop > 15ms
            self.get_logger().warn(f"Slow control loop: {loop_time*1000:.1f}ms")

    def stop(self):
        """Stop the robot and clean up"""
        self.running = False
        self.controller.stop()
        self.sock.close()
        # Turn off both GPIO pins when stopping
        gpio_pin_17.off()
        gpio_pin_27.off()
        self.get_logger().info("Robot systems shut down.")

    def check_critical_obstacles_fast(self):
        """Check critical obstacles dengan minimal processing"""
        # Hanya check jika data LIDAR valid dan fresh
        if (time.time() - self.lidar.last_scan_time > 0.01 or 
            not hasattr(self.lidar, 'critical_danger')):
            return False
        
        if self.lidar.critical_danger:
            self.controller.stop()
            return True
        return False

    def process_uwb_data_fast(self):
        """Process UWB data dengan timeout minimal"""
        try:
            self.sock.setblocking(False)
            data, addr = self.sock.recvfrom(1024)
            
            # Parse data dengan error handling minimal
            parts = data.decode().split(",")
            if len(parts) >= 3:
                self.raw_uwb_distances = {
                    'A0': float(parts[0]),
                    'A1': float(parts[1]),
                    'A2': float(parts[2])
                }
                
                # Apply correction
                self.corrected_uwb_distances = self.uwb_tracker.apply_bias_correction(
                    self.raw_uwb_distances
                )
                self.last_uwb_update = time.time()
                return True
                
        except (socket.error, ValueError, IndexError):
            pass  # Ignore errors untuk performa
        
        return False

    
    def stop(self):
        """Stop the robot and clean up"""
        self.running = False
        self.controller.stop()
        self.sock.close()
        self.get_logger().info("Robot systems shut down.")

def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create global reference to UWB tracker for coordinate system 
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


if __name__ == "__main__":
    main()