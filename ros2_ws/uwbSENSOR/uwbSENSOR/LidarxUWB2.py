
#!/usr/bin/env python3

import socket
import numpy as np
import math
import time
import threading
import sys
import os
import select

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from ddsm115 import MotorControl

# Konfigurasi Global
SCAN_THRESHOLD = 6000  # Jarak aman dalam mm
DANGER_THRESHOLD = 1300  # Jarak bahaya dalam mm
MIN_VALID_DISTANCE = 250  # Jarak minimum valid (mm) untuk menghindari noise
CRITICAL_DANGER_THRESHOLD = 700  # Jarak kritis untuk override UWB (mm)

# UWB UDP Configuration
UDP_IP = "192.168.243.128"
UDP_PORT = 5005

# Motor speed configuration
DEFAULT_SPEED = 70
ROTATION_FACTOR = 2
STOP_THRESHOLD = 70  # cm

# Definisi sudut untuk deteksi rintangan
FRONT_REGION = (250,290)  # Depan: 180-360°
LEFT_REGION = (120, 250)    # Kiri: 90-180°
RIGHT_REGION = (290, 60)     # Kanan: 0-90°

# Target exclusion zone - don't treat objects in this area as obstacles
TARGET_EXCLUSION_ANGLE = 30  # Increased from 30 to 40 degrees for wider target exclusion zone

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
            'A1': 1.06,  # Scale factor
            'A2': 1.05   # Scale factor
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
        self.moving_avg_window = 3
        self.distance_history = {}  # Store recent distance values for filtering
    
    def set_target_info(self, direction, distance):
        """Set target information from UWB to avoid treating target as obstacle"""
        self.target_direction = direction
        self.target_distance = distance
    
    def process_scan(self, scan_msg):
        """Process ROS2 LaserScan message"""
        with self.lock:
            # Save scan message and update timestamp immediately
            self.last_scan_msg = scan_msg
            self.last_scan_time = time.time()
            
            # Clear old scan data
            self.scan_data.clear()
            
            # Extract ranges and convert to dictionary
            ranges = scan_msg.ranges
            angle_increment = scan_msg.angle_increment
            angle_min = scan_msg.angle_min
            
            # Process scan data - optimized for speed
            for i, distance in enumerate(ranges):
                # Skip invalid measurements immediately
                if distance < MIN_VALID_DISTANCE/1000.0 or math.isinf(distance):
                    continue
                
                # Calculate angle in degrees
                angle_rad = angle_min + (i * angle_increment)
                angle_deg = int(math.degrees(angle_rad) % 360)
                
                # Convert to mm and store
                distance_mm = int(distance * 1000)
                
                # Apply simple moving average filter if we have history
                if angle_deg in self.distance_history:
                    history = self.distance_history[angle_deg]
                    history.append(distance_mm)
                    if len(history) > self.moving_avg_window:
                        history.pop(0)
                    # Use median filter for more robustness against outliers
                    self.scan_data[angle_deg] = int(np.median(history))
                else:
                    self.distance_history[angle_deg] = [distance_mm]
                    self.scan_data[angle_deg] = distance_mm
            
            # Analyze obstacles
            self._analyze_obstacles()
    
    def _is_angle_in_region(self, angle, region):
        """Check if angle is in specified region, handling wraparound at 360°"""
        start, end = region
        if start <= end:
            return start <= angle <= end
        else:  # Wraparound case (e.g. 350° to 10°)
            return angle >= start or angle <= end
    
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
    
    def _analyze_obstacles(self):
        """Analyze scan data to detect obstacles in different regions"""
        if not self.scan_data:
            # No valid scan data
            return
        
        # Reset obstacle status
        self.front_obstacle = False
        self.left_obstacle = False
        self.right_obstacle = False
        self.danger_zone = False
        self.critical_danger = False
        
        # Reset distances
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        
        # Count points in each region for confidence assessment
        front_points = 0
        left_points = 0
        right_points = 0
        
        # Analyze each region
        for angle, distance in self.scan_data.items():
            # Skip points that are near the UWB target direction
            if self._is_angle_near_target(angle):
                continue
                
            # Front region
            if self._is_angle_in_region(angle, FRONT_REGION):
                front_points += 1
                if distance < self.front_distance:
                    self.front_distance = distance
                if distance < SCAN_THRESHOLD:
                    self.front_obstacle = True
                if distance < DANGER_THRESHOLD:
                    self.danger_zone = True
                if distance < CRITICAL_DANGER_THRESHOLD:
                    self.critical_danger = True
            
            # Left region
            elif self._is_angle_in_region(angle, LEFT_REGION):
                left_points += 1
                if distance < self.left_distance:
                    self.left_distance = distance
                if distance < SCAN_THRESHOLD:
                    self.left_obstacle = True
            
            # Right region
            elif self._is_angle_in_region(angle, RIGHT_REGION):
                right_points += 1
                if distance < self.right_distance:
                    self.right_distance = distance
                if distance < SCAN_THRESHOLD:
                    self.right_obstacle = True
    
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
    
    def move(self, right_speed, left_speed):
        """Move the robot with specified motor speeds"""
        self.right_motor.send_rpm(1, right_speed)
        self.left_motor.send_rpm(1, left_speed)
        self.last_command_time = time.time()
    
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
            if status['scan_age'] > 0.001:  # Reduced threshold for warning
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
            uwb_distances['A0'] < 400 and  # Within 2 meters
            abs(uwb_distances['A1'] - uwb_distances['A2']) < 30  # Roughly centered
        )
        
        # MODIFIED: Prioritize UWB following, with obstacle avoidance only for critical cases
        # Run lighter obstacle check that only looks for critical dangers
        obstacle_action_needed = self.handle_obstacle_avoidance(lidar, target_in_view)
        
        # If no critical obstacle, use UWB for following (prioritizing UWB)
        if not obstacle_action_needed and not self.emergency_stop:
            self.process_uwb_control(uwb_distances)

class FollowingRobotNode(Node):
    """ROS2 Node for human-following robot with obstacle avoidance"""
    
    def __init__(self):
        super().__init__('following_robot_node')
        
        # Configuration
        self.r_wheel_port = "/dev/ttyRS485-1"
        self.l_wheel_port = "/dev/ttyRS485-2"
        
        # Initialize components
        self.uwb_tracker = UWBTracker()
        self.lidar = LidarProcessor()
        self.controller = RobotController(self.r_wheel_port, self.l_wheel_port)
        
        # Configure ROS2 QoS for better LIDAR data handling
        lidar_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Use BEST_EFFORT for sensor data
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1  # Only need latest scan
        )
        
        # Create LiDAR subscription with optimized QoS
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Topic for RPLidar A2M12
            self.scan_callback,
            qos_profile=lidar_qos
        )
        
        # Create socket for UWB data
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.sock.settimeout(0.01)  # Reduced timeout for faster response
        
        # UWB data
        self.raw_uwb_distances = {'A0': 1000, 'A1': 1000, 'A2': 1000}
        self.corrected_uwb_distances = {'A0': 1000, 'A1': 1000, 'A2': 1000}
        
        # Status control
        self.running = True
        self.last_uwb_update = 0
        
        # Create timers for different control operations to reduce overall delay
        self.control_timer = self.create_timer(0.001, self.control_loop)  # 20Hz control loop
        
        # Performance monitoring
        self.last_lidar_time = time.time()
        self.lidar_update_count = 0
        self.lidar_update_rate = 0
        
        # Log statistics periodically
        self.stats_timer = self.create_timer(0.01, self.log_statistics)
        
        self.get_logger().info('Following Robot Node started')
    
    def scan_callback(self, msg):
        """Process incoming LaserScan messages"""
        # Measure performance
        current_time = time.time()
        self.lidar_update_count += 1
        
        # Process scan data immediately - this is now lightweight and optimized
        self.lidar.process_scan(msg)
    
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
        """Main control loop executed on timer"""
        if not self.running:
            return
        
        try:
            # Process UWB data first for prioritization
            try:
                self.sock.setblocking(False)
                ready = select.select([self.sock], [], [], 0.05 )  # Reduced timeout
                if ready[0]:
                    data, addr = self.sock.recvfrom(1024)
                    parts = data.decode().split(",")
                    
                    # Update raw distances
                    self.raw_uwb_distances = {
                        'A0': float(parts[0]),
                        'A1': float(parts[1]),
                        'A2': float(parts[2])
                    }
                    
                    # Apply bias correction
                    self.corrected_uwb_distances = self.uwb_tracker.apply_bias_correction(self.raw_uwb_distances)
                    self.last_uwb_update = time.time()
                    
                    # Debug info
                    self.get_logger().debug(f"Raw UWB: {self.raw_uwb_distances}")
                    self.get_logger().debug(f"Corrected: {self.corrected_uwb_distances}")
            except Exception as e:
                if not str(e).startswith('Resource temporarily unavailable'):  # Ignore common non-blocking socket errors
                    self.get_logger().error(f"Error processing UWB data: {e}")
            
            # Calculate UWB data age
            uwb_data_age = time.time() - self.last_uwb_update
            uwb_data_valid = uwb_data_age < 0.5  # Reduced threshold for faster response
            
            # Process control based on sensor data
            if uwb_data_valid:
                # We have valid UWB data - use integrated control with UWB priority
                self.controller.process_control(self.corrected_uwb_distances, self.lidar)
            else:
                # No valid UWB data - just do basic obstacle avoidance
                self.controller.handle_obstacle_avoidance(self.lidar)
                
        except Exception as e:
            self.get_logger().error(f"Error in control loop: {e}")
    
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
    
    try:
        # Run until interrupted
        rclpy.spin(node)
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

