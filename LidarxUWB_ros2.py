
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

import socket
import numpy as np
import math
import time
import threading
import sys
import os

# Import RPLIDAR driver
try:
    from rplidar import RPLidar
except ImportError:
    print("RPLIDAR library not found. Please install it using:")
    print("pip install rplidar-driver")
    sys.exit(1)

# Import motor controller
from ddsm115 import MotorControl

# Konfigurasi Global
LIDAR_PORT = '/dev/ttyUSB0'  # Sesuaikan dengan port RPLIDAR
SCAN_THRESHOLD = 500  # Jarak aman dalam mm
DANGER_THRESHOLD = 300  # Jarak bahaya dalam mm
MIN_VALID_DISTANCE = 100  # Jarak minimum valid (mm)

# UWB UDP Configuration
UDP_IP = "192.168.150.128"
UDP_PORT = 5005

# Motor speed configuration
DEFAULT_SPEED = 80
ROTATION_FACTOR = 2
STOP_THRESHOLD = 70  # cm

# Definisi sudut untuk deteksi rintangan
FRONT_REGION = (330, 30)  # Depan: 330-360° dan 0-30°
LEFT_REGION = (30, 90)    # Kiri: 30-90°
RIGHT_REGION = (270, 330) # Kanan: 270-330°


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
            'A1': 1.02,  # Scale factor
            'A2': 1.02   # Scale factor
        }
    
    def apply_bias_correction(self, distances):
        """Koreksi bias dan scaling pada pengukuran jarak"""
        corrected_distances = {
            'A0': max((distances['A0'] * 100 * self.scale_factor['A0']) - self.bias['A0'], 0),
            'A1': max((distances['A1'] * 100 * self.scale_factor['A1']) - self.bias['A1'], 0),
            'A2': max((distances['A2'] * 100 * self.scale_factor['A2']) - self.bias['A2'], 0)
        }
        return corrected_distances


class LidarProcessor:
    """Processes LIDAR data using RPLIDAR SDK directly"""
    
    def __init__(self, port=LIDAR_PORT, logger=None):
        self.port = port
        self.lidar = None
        self.scan_data = {}  # Dictionary untuk menyimpan data scan (angle -> distance)
        self.running = False
        self.scan_thread = None
        self.lock = threading.Lock()
        self.logger = logger  # ROS logger
        
        # Status rintangan
        self.front_obstacle = False
        self.left_obstacle = False
        self.right_obstacle = False
        self.danger_zone = False
        
        # Jarak minimum di setiap region
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        
        # Timestamp data terakhir
        self.last_scan_time = 0
        
        # LaserScan data for ROS
        self.latest_scan = None
        self.scan_updated = False
        
    def log_info(self, msg):
        """Log info message using ROS logger if available"""
        if self.logger:
            self.logger.info(msg)
        else:
            print(msg)
            
    def log_error(self, msg):
        """Log error message using ROS logger if available"""
        if self.logger:
            self.logger.error(msg)
        else:
            print(f"ERROR: {msg}")
        
    def start(self):
        """Start LIDAR scanning"""
        if self.running:
            return True
        
        try:
            self.log_info(f"Connecting to RPLIDAR on port {self.port}...")
            self.lidar = RPLidar(self.port)
            
            # Get device info
            info = self.lidar.get_info()
            self.log_info(f"RPLIDAR Info: {info}")
            
            # Get device health
            health = self.lidar.get_health()
            self.log_info(f"RPLIDAR Health: {health}")
            
            if health[0] == 'Error':
                self.log_info("RPLIDAR reports error, attempting reset...")
                self.lidar.reset()
                time.sleep(1)
                health = self.lidar.get_health()
                self.log_info(f"RPLIDAR Health after reset: {health}")
            
            # Start motor
            self.lidar.start_motor()
            time.sleep(1)  # Allow motor to reach full speed
            
            # Start scanning in separate thread
            self.running = True
            self.scan_thread = threading.Thread(target=self._scan_loop)
            self.scan_thread.daemon = True
            self.scan_thread.start()
            
            self.log_info("RPLIDAR scanning started successfully.")
            return True
            
        except Exception as e:
            self.log_error(f"Error starting RPLIDAR: {e}")
            self.stop()
            return False
    
    def stop(self):
        """Stop LIDAR scanning"""
        self.running = False
        
        if self.lidar:
            try:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
                self.log_info("RPLIDAR stopped and disconnected.")
            except Exception as e:
                self.log_error(f"Error stopping RPLIDAR: {e}")
        
        self.lidar = None
    
    def _scan_loop(self):
        """Continuous scan loop in a separate thread"""
        try:
            for scan in self.lidar.iter_scans(max_buf_meas=2000):
                if not self.running:
                    break
                
                with self.lock:
                    # Clear old scan data
                    self.scan_data.clear()
                    
                    # Create arrays for LaserScan message
                    angles = []
                    distances = []
                    
                    # Update scan data
                    for _, angle, distance in scan:
                        # Ignore invalid measurements (too close or infinity)
                        if distance < MIN_VALID_DISTANCE:
                            continue
                            
                        # Convert to integers for efficiency and store
                        int_angle = int(angle)
                        self.scan_data[int_angle] = int(distance)
                        
                        # Store data for ROS LaserScan
                        angles.append(float(angle))
                        distances.append(float(distance) / 1000.0)  # Convert to meters for ROS
                    
                    # Prepare LaserScan message
                    if angles:
                        # Create LaserScan data structure (not a ROS message yet)
                        self.latest_scan = {
                            'angle_min': math.radians(min(angles)),
                            'angle_max': math.radians(max(angles)),
                            'ranges': distances,
                            'angles': angles
                        }
                        self.scan_updated = True
                    
                    # Analyze obstacle data
                    self._analyze_obstacles()
                    
                    # Update timestamp
                    self.last_scan_time = time.time()
                
                # Short sleep to reduce CPU usage
                time.sleep(0.01)
                
        except Exception as e:
            self.log_error(f"Error in LIDAR scan loop: {e}")
            self.stop()
    
    def get_latest_scan_data(self):
        """Get latest scan data for ROS publishing"""
        with self.lock:
            if self.scan_updated:
                self.scan_updated = False
                return self.latest_scan
            return None
    
    def _is_angle_in_region(self, angle, region):
        """Check if angle is in specified region, handling wraparound at 360°"""
        start, end = region
        if start <= end:
            return start <= angle <= end
        else:  # Wraparound case (e.g. 350° to 10°)
            return angle >= start or angle <= end
    
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
        
        # Reset distances
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        
        # Analyze each region
        for angle, distance in self.scan_data.items():
            # Front region
            if self._is_angle_in_region(angle, FRONT_REGION):
                if distance < self.front_distance:
                    self.front_distance = distance
                if distance < SCAN_THRESHOLD:
                    self.front_obstacle = True
                if distance < DANGER_THRESHOLD:
                    self.danger_zone = True
            
            # Left region
            elif self._is_angle_in_region(angle, LEFT_REGION):
                if distance < self.left_distance:
                    self.left_distance = distance
                if distance < SCAN_THRESHOLD:
                    self.left_obstacle = True
            
            # Right region
            elif self._is_angle_in_region(angle, RIGHT_REGION):
                if distance < self.right_distance:
                    self.right_distance = distance
                if distance < SCAN_THRESHOLD:
                    self.right_obstacle = True
    
    def get_obstacle_status(self):
        """Get current obstacle status"""
        with self.lock:
            scan_age = time.time() - self.last_scan_time
            data_valid = scan_age < 1.0  # Data valid if less than 1 second old
            
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
                'data_valid': data_valid,
                'scan_points': len(self.scan_data) if data_valid else 0
            }
            
            return status
    
    def get_safe_direction(self):
        """Determine the safest direction to move based on obstacles"""
        status = self.get_obstacle_status()
        
        if not status['data_valid']:
            self.log_info("Warning: LIDAR data not valid or too old")
            return None
        
        if status['danger_zone']:
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
    
    def __init__(self, r_wheel_port, l_wheel_port, logger=None):
        # Motor controllers
        self.right_motor = MotorControl(device=r_wheel_port)
        self.left_motor = MotorControl(device=l_wheel_port)
        self.right_motor.set_drive_mode(1, 2)
        self.left_motor.set_drive_mode(1, 2)
        
        # ROS logger
        self.logger = logger
        
        # Speed configuration
        self.speed = DEFAULT_SPEED
        self.rotation_factor = ROTATION_FACTOR
        self.stop_threshold = STOP_THRESHOLD
        
        # Status flags
        self.obstacle_avoidance_active = False
        self.last_command_time = time.time()
        self.current_direction = "STOP"
    
    def log_info(self, msg):
        """Log info message using ROS logger if available"""
        if self.logger:
            self.logger.info(msg)
        else:
            print(msg)
    
    def move(self, right_speed, left_speed):
        """Move the robot with specified motor speeds"""
        self.right_motor.send_rpm(1, right_speed)
        self.left_motor.send_rpm(1, left_speed)
        self.last_command_time = time.time()
    
    def stop(self):
        """Stop the robot"""
        self.move(0, 0)
        self.current_direction = "STOP"
    
    def handle_obstacle_avoidance(self, lidar):
        """Process LIDAR data for obstacle avoidance"""
        # Get obstacle status and safe direction
        status = lidar.get_obstacle_status()
        safe_direction = lidar.get_safe_direction()
        
        # Check if LIDAR data is valid
        if not status['data_valid'] or safe_direction is None:
            self.log_info("LIDAR data not valid, continuing with UWB control")
            self.obstacle_avoidance_active = False
            return False
        
        # Print obstacle status for debugging
        self.log_info(f"\n--- LIDAR Status ---")
        self.log_info(f"Front: {'BLOCKED' if status['front']['obstacle'] else 'clear'} ({status['front']['distance']}mm)")
        self.log_info(f"Left: {'BLOCKED' if status['left']['obstacle'] else 'clear'} ({status['left']['distance']}mm)")
        self.log_info(f"Right: {'BLOCKED' if status['right']['obstacle'] else 'clear'} ({status['right']['distance']}mm)")
        self.log_info(f"Danger Zone: {'YES' if status['danger_zone'] else 'NO'}")
        self.log_info(f"Points in scan: {status['scan_points']}")
        self.log_info(f"Safe direction: {safe_direction}")
        
        # Handle obstacle avoidance based on safe direction
        if safe_direction == "STOP" or status['danger_zone']:
            self.log_info("DANGER ZONE DETECTED - STOPPING")
            self.stop()
            self.obstacle_avoidance_active = True
            return True
            
        elif safe_direction == "LEFT":
            self.log_info("Obstacle avoidance: Turning LEFT")
            self.move(-self.speed/self.rotation_factor, 0)
            self.current_direction = "LEFT"
            self.obstacle_avoidance_active = True
            return True
            
        elif safe_direction == "RIGHT":
            self.log_info("Obstacle avoidance: Turning RIGHT")
            self.move(0, self.speed/self.rotation_factor)
            self.current_direction = "RIGHT"
            self.obstacle_avoidance_active = True
            return True
            
        else:  # "FORWARD" or None
            # No obstacle that requires immediate action
            self.obstacle_avoidance_active = False
            return False
    
    def process_uwb_control(self, uwb_distances):
        """Process UWB data to control robot movement"""
        A0, A1, A2 = uwb_distances['A0'], uwb_distances['A1'], uwb_distances['A2']
        
        # Print UWB distances
        self.log_info("\n--- UWB Distances (cm) ---")
        self.log_info(f"A0: {A0:.2f} | A1: {A1:.2f} | A2: {A2:.2f}")
        
        # Logic for UWB-based control
        if A0 <= self.stop_threshold:
            self.log_info(f"Target reached (A0 <= {self.stop_threshold} cm). Stopping.")
            self.stop()
            return
        
        if A0 < A1 and A0 < A2:
            self.log_info("Move Forward - A0 is closest")
            self.move(-self.speed, self.speed)
            self.current_direction = "FORWARD"
        elif (A1 > A2 and A1 > A0):
            self.log_info("Rotate Right - A2 is closest")
            self.move(0, self.speed/self.rotation_factor)
            self.current_direction = "RIGHT"
        elif (A2 > A1 and A2 > A0):
            self.log_info("Rotate Left - A1 is closest")
            self.move(-self.speed/self.rotation_factor, 0)
            self.current_direction = "LEFT"
        elif (A2 < A1):
            self.log_info("Rotate Right - A2 is closest")
            self.move(0, self.speed/self.rotation_factor)
            self.current_direction = "RIGHT"
        elif (A1 < A2):
            self.log_info("Rotate Left - A1 is closest")
            self.move(-self.speed/self.rotation_factor, 0)
            self.current_direction = "LEFT"
        elif (A0 > A1 or A0 > A2):
            self.log_info("Rotasi")
            self.move(self.speed/self.rotation_factor, self.speed/self.rotation_factor)
            self.current_direction = "ROTATE"
        else:
            self.log_info("Maintaining current movement - No clear direction")
    
    def process_control(self, uwb_distances, lidar):
        """Main control logic integrating UWB and LIDAR data"""
        # First priority: Obstacle avoidance (safety)
        obstacle_action_taken = self.handle_obstacle_avoidance(lidar)
        
        # If LIDAR didn't require immediate action, use UWB for following
        if not obstacle_action_taken:
            self.process_uwb_control(uwb_distances)
        
        # Return current direction for ROS status
        return self.current_direction, self.obstacle_avoidance_active


class FollowingRobotNode(Node):
    """ROS 2 Node for human-following robot with obstacle avoidance"""
    
    def __init__(self):
        super().__init__('following_robot')
        
        # Configuration parameters
        self.declare_parameter('lidar_port', '/dev/ttyUSB0')
        self.declare_parameter('r_wheel_port', '/dev/ttyRS485-1')
        self.declare_parameter('l_wheel_port', '/dev/ttyRS485-2')
        self.declare_parameter('uwb_ip', '192.168.150.128')
        self.declare_parameter('uwb_port', 5005)
        
        # Get parameters
        self.lidar_port = self.get_parameter('lidar_port').value
        self.r_wheel_port = self.get_parameter('r_wheel_port').value
        self.l_wheel_port = self.get_parameter('l_wheel_port').value
        self.uwb_ip = self.get_parameter('uwb_ip').value
        self.uwb_port = self.get_parameter('uwb_port').value
        
        # Initialize ROS publishers
        self.scan_publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.uwb_publisher = self.create_publisher(Float32MultiArray, 'uwb_distances', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Initialize components
        self.uwb_tracker = UWBTracker()
        self.lidar = LidarProcessor(port=self.lidar_port, logger=self.get_logger())
        self.controller = RobotController(self.r_wheel_port, self.l_wheel_port, logger=self.get_logger())
        
        # Socket for UWB data
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.uwb_ip, self.uwb_port))
        self.sock.settimeout(0.1)  # Non-blocking with 100ms timeout
        
        # UWB data
        self.raw_uwb_distances = {'A0': 1000, 'A1': 1000, 'A2': 1000}
        self.corrected_uwb_distances = {'A0': 1000, 'A1': 1000, 'A2': 1000}
        self.last_uwb_update = 0
        
        # Setup timers for ROS operations
        self.timer_period = 0.05  # 20Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.scan_timer = self.create_timer(0.1, self.publish_scan)  # 10Hz for scan publishing
        
        # Status variables
        self.running = False
        
        # Start components
        self.start()
        
    def start(self):
        """Start the robot and subsystems"""
        self.get_logger().info("Starting robot systems...")
        
        # Start LIDAR
        if not self.lidar.start():
            self.get_logger().error("Failed to start LIDAR. Robot will operate in UWB-only mode.")
        
        self.running = True
        self.get_logger().info("Robot systems started successfully.")
    
    def stop(self):
        """Stop the robot and subsystems"""
        self.running = False
        self.controller.stop()
        self.lidar.stop()
        self.get_logger().info("Robot systems shut down.")
    
    def timer_callback(self):
        """Main control loop callback (20Hz)"""
        if not self.running:
            return
            
        # Process UWB data
        try:
            self.sock.settimeout(0.05)  # Short timeout to not block timer
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
            
            # Publish UWB data to ROS
            self.publish_uwb_data()
            
            # Debug info
            self.get_logger().info(f"UWB Raw: A0={self.raw_uwb_distances['A0']:.2f}, " 
                                + f"A1={self.raw_uwb_distances['A1']:.2f}, "
                                + f"A2={self.raw_uwb_distances['A2']:.2f}")
            
        except socket.timeout:
            # No new UWB data, continue with just LIDAR
            pass
        except Exception as e:
            self.get_logger().error(f"Error processing UWB data: {e}")
        
        # Calculate UWB data age
        uwb_data_age = time.time() - self.last_uwb_update
        uwb_data_valid = uwb_data_age < 1.0  # Valid if less than 1 second old
        
        # Process control based on sensor data
        if uwb_data_valid:
            # We have valid UWB data - use integrated control
            direction, obstacle_mode = self.controller.process_control(
                self.corrected_uwb_distances, self.lidar)
            
            # Publish current robot state as cmd_vel
            self.publish_robot_velocity(direction, obstacle_mode)
        else:
            # No valid UWB data - just do obstacle avoidance
            self.controller.handle_obstacle_avoidance(self.lidar)
    
    def publish_uwb_data(self):
        """Publish UWB data to ROS topic"""
        msg = Float32MultiArray()
        msg.data = [
            float(self.corrected_uwb_distances['A0']),
            float(self.corrected_uwb_distances['A1']),
            float(self.corrected_uwb_distances['A2'])
        ]
        self.uwb_publisher.publish(msg)
    
    def publish_scan(self):
        """Publish LIDAR scan data to ROS topic"""
        scan_data = self.lidar.get_latest_scan_data()
        if not scan_data:
            return
            
        # Create LaserScan message
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'
        
        # Calculate angle parameters
        msg.angle_min = float(scan_data['angle_min'])
        msg.angle_max = float(scan_data['angle_max'])
        msg.angle_increment = 0.0  # Variable increments, not fixed
        
        # Fill in range data
        msg.range_min = 0.05  # 5cm minimum range
        msg.range_max = 10.0  # 10m maximum range
        
        # Create range array - convert to proper format for LaserScan
        angles_rad = [math.radians(angle) for angle in scan_data['angles']]
        indices = np.argsort(angles_rad)
        
        # Order ranges by angle
        ordered_ranges = []
        ordered_angles = []
        for i in indices:
            ordered_ranges.append(scan_data['ranges'][i])
            ordered_angles.append(angles_rad[i])
        
        # Calculate proper angle increment from ordered angles
        if len(ordered_angles) > 1:
            # Use average increment
            increments = []
            for i in range(1, len(ordered_angles)):
                inc = ordered_angles[i] - ordered_angles[i-1]
                if inc > 0:  # Ignore wraparounds
                    increments.append(inc)
            if increments:
                msg.angle_increment = sum(increments) / len(increments)
        
        msg.ranges = ordered_ranges
        msg.intensities = []  # No intensity data
        
        # Publish scan
        self.scan_publisher.publish(msg)
    
    def publish_robot_velocity(self, direction, obstacle_mode):
        """Publish robot velocity as Twist message"""
        msg = Twist()
        
        # Convert direction to linear and angular velocity
        if direction == "FORWARD":
            msg.linear.x = 0.2
            msg.angular.z = 0.0
        elif direction == "LEFT":
            msg.linear.x = 0.0
            msg.angular.z = 0.5
        elif direction == "RIGHT":
            msg.linear.x = 0.0
            msg.angular.z = -0.5
        elif direction == "ROTATE":
            msg.linear.x = 0.0
            msg.angular.z = 0.3
        else:  # "STOP"
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        
        # Publish the message
        self.cmd_vel_publisher.publish(msg)

    def destroy_node(self):
        """Clean shutdown of node"""
        self.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = FollowingRobotNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

