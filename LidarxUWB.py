
#!/usr/bin/env python3

import socket
import numpy as np
import math
import time
import threading
import sys
import os

# Tambahkan path ke SDK RPLIDAR
# Sesuaikan dengan lokasi SDK di sistem kamu
# Misalnya, jika repo RPLIDAR sudah di-clone dan berada di folder yang sama
SDK_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../rplidar_ros/sdk/python")
if os.path.exists(SDK_PATH):
    sys.path.append(SDK_PATH)
else:
    # Alternatif: gunakan pip install
    # Jika belum diinstall: pip install rplidar-driver
    pass

from rplidar import RPLidar
from ddsm115 import MotorControl

# Konfigurasi Global
LIDAR_PORT = '/dev/ttyUSB0'  # Sesuaikan dengan port RPLIDAR
SCAN_THRESHOLD = 500  # Jarak aman dalam mm
DANGER_THRESHOLD = 300  # Jarak bahaya dalam mm
MIN_VALID_DISTANCE = 100  # Jarak minimum valid (mm) untuk menghindari noise

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
    
    def __init__(self, port=LIDAR_PORT):
        self.port = port
        self.lidar = None
        self.scan_data = {}  # Dictionary untuk menyimpan data scan (angle -> distance)
        self.running = False
        self.scan_thread = None
        self.lock = threading.Lock()
        
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
        
    def start(self):
        """Start LIDAR scanning"""
        if self.running:
            return True
        
        try:
            print(f"Connecting to RPLIDAR on port {self.port}...")
            self.lidar = RPLidar(self.port)
            
            # Get device info
            info = self.lidar.get_info()
            print(f"RPLIDAR Info: {info}")
            
            # Get device health
            health = self.lidar.get_health()
            print(f"RPLIDAR Health: {health}")
            
            if health[0] == 'Error':
                print("RPLIDAR reports error, attempting reset...")
                self.lidar.reset()
                time.sleep(1)
                health = self.lidar.get_health()
                print(f"RPLIDAR Health after reset: {health}")
            
            # Start motor
            self.lidar.start_motor()
            time.sleep(1)  # Allow motor to reach full speed
            
            # Start scanning in separate thread
            self.running = True
            self.scan_thread = threading.Thread(target=self._scan_loop)
            self.scan_thread.daemon = True
            self.scan_thread.start()
            
            print("RPLIDAR scanning started successfully.")
            return True
            
        except Exception as e:
            print(f"Error starting RPLIDAR: {e}")
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
                print("RPLIDAR stopped and disconnected.")
            except Exception as e:
                print(f"Error stopping RPLIDAR: {e}")
        
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
                    
                    # Update scan data
                    for _, angle, distance in scan:
                        # Ignore invalid measurements (too close or infinity)
                        if distance < MIN_VALID_DISTANCE:
                            continue
                            
                        # Convert to integers for efficiency and store
                        self.scan_data[int(angle)] = int(distance)
                    
                    # Analyze obstacle data
                    self._analyze_obstacles()
                    
                    # Update timestamp
                    self.last_scan_time = time.time()
                
                # Short sleep to reduce CPU usage
                time.sleep(0.01)
                
        except Exception as e:
            print(f"Error in LIDAR scan loop: {e}")
            self.stop()
    
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
            print("Warning: LIDAR data not valid or too old")
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
            print("LIDAR data not valid, continuing with UWB control")
            self.obstacle_avoidance_active = False
            return False
        
        # Print obstacle status for debugging
        print(f"\n--- LIDAR Status ---")
        print(f"Front: {'BLOCKED' if status['front']['obstacle'] else 'clear'} ({status['front']['distance']}mm)")
        print(f"Left: {'BLOCKED' if status['left']['obstacle'] else 'clear'} ({status['left']['distance']}mm)")
        print(f"Right: {'BLOCKED' if status['right']['obstacle'] else 'clear'} ({status['right']['distance']}mm)")
        print(f"Danger Zone: {'YES' if status['danger_zone'] else 'NO'}")
        print(f"Points in scan: {status['scan_points']}")
        print(f"Safe direction: {safe_direction}")
        
        # Handle obstacle avoidance based on safe direction
        if safe_direction == "STOP" or status['danger_zone']:
            print("DANGER ZONE DETECTED - STOPPING")
            self.stop()
            self.obstacle_avoidance_active = True
            return True
            
        elif safe_direction == "LEFT":
            print("Obstacle avoidance: Turning LEFT")
            self.move(-self.speed/self.rotation_factor, 0)
            self.current_direction = "LEFT"
            self.obstacle_avoidance_active = True
            return True
            
        elif safe_direction == "RIGHT":
            print("Obstacle avoidance: Turning RIGHT")
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
        print("\n--- UWB Distances (cm) ---")
        print(f"A0: {A0:.2f} | A1: {A1:.2f} | A2: {A2:.2f}")
        
        # Logic for UWB-based control
        if A0 <= self.stop_threshold:
            print(f"Target reached (A0 <= {self.stop_threshold} cm). Stopping.")
            self.stop()
            return
        
        if A0 < A1 and A0 < A2:
            print("Move Forward - A0 is closest")
            self.move(-self.speed, self.speed)
            self.current_direction = "FORWARD"
        elif (A1 > A2 and A1 > A0):
            print("Rotate Right - A2 is closest")
            self.move(0, self.speed/self.rotation_factor)
            self.current_direction = "RIGHT"
        elif (A2 > A1 and A2 > A0):
            print("Rotate Left - A1 is closest")
            self.move(-self.speed/self.rotation_factor, 0)
            self.current_direction = "LEFT"
        elif (A2 < A1):
            print("Rotate Right - A2 is closest")
            self.move(0, self.speed/self.rotation_factor)
            self.current_direction = "RIGHT"
        elif (A1 < A2):
            print("Rotate Left - A1 is closest")
            self.move(-self.speed/self.rotation_factor, 0)
            self.current_direction = "LEFT"
        elif (A0 > A1 or A0 > A2):
            print("Rotasi")
            self.move(self.speed/self.rotation_factor, self.speed/self.rotation_factor)
            self.current_direction = "ROTATE"
        else:
            print("Maintaining current movement - No clear direction")
    
    def process_control(self, uwb_distances, lidar):
        """Main control logic integrating UWB and LIDAR data"""
        # First priority: Obstacle avoidance (safety)
        obstacle_action_taken = self.handle_obstacle_avoidance(lidar)
        
        # If LIDAR didn't require immediate action, use UWB for following
        if not obstacle_action_taken:
            self.process_uwb_control(uwb_distances)


class FollowingRobot:
    """Main class for human-following robot with obstacle avoidance"""
    
    def __init__(self):
        # Configuration
        self.r_wheel_port = "/dev/ttyRS485-1"
        self.l_wheel_port = "/dev/ttyRS485-2"
        
        # Initialize components
        self.uwb_tracker = UWBTracker()
        self.lidar = LidarProcessor(port=LIDAR_PORT)
        self.controller = RobotController(self.r_wheel_port, self.l_wheel_port)
        
        # Socket for UWB data
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.sock.settimeout(0.1)  # Non-blocking with 100ms timeout
        
        # UWB data
        self.raw_uwb_distances = {'A0': 1000, 'A1': 1000, 'A2': 1000}
        self.corrected_uwb_distances = {'A0': 1000, 'A1': 1000, 'A2': 1000}
        
        # Status control
        self.running = False
        self.last_uwb_update = 0
    
    def start(self):
        """Start the robot and all subsystems"""
        print("Starting robot systems...")
        
        # Start LIDAR
        if not self.lidar.start():
            print("Failed to start LIDAR. Robot cannot operate safely.")
            return False
        
        # Everything is ready
        self.running = True
        print("Robot systems started successfully.")
        return True
    
    def stop(self):
        """Stop the robot and shut down all subsystems"""
        self.running = False
        self.controller.stop()
        self.lidar.stop()
        self.sock.close()
        print("Robot systems shut down.")
    
    def run(self):
        """Main robot loop"""
        if not self.start():
            return
        
        print("Robot is now operational. Press Ctrl+C to stop.")
        
        try:
            while self.running:
                # Process UWB data
                try:
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
                    print(f"\n=== New data cycle ===")
                    print(f"Raw UWB: {self.raw_uwb_distances}")
                    print(f"Corrected: {self.corrected_uwb_distances}")
                    
                except socket.timeout:
                    # No new UWB data
                    pass
                except Exception as e:
                    print(f"Error processing UWB data: {e}")
                
                # Calculate UWB data age
                uwb_data_age = time.time() - self.last_uwb_update
                uwb_data_valid = uwb_data_age < 1.0  # Valid if less than 1 second old
                
                # Process control based on sensor data
                if uwb_data_valid:
                    # We have valid UWB data - use integrated control
                    self.controller.process_control(self.corrected_uwb_distances, self.lidar)
                else:
                    # No valid UWB data - just do obstacle avoidance
                    self.controller.handle_obstacle_avoidance(self.lidar)
                
                # Short delay to reduce CPU usage
                time.sleep(0.05)
                
        except KeyboardInterrupt:
            print("\nUser interrupted. Shutting down...")
        except Exception as e:
            print(f"Error in main loop: {e}")
        finally:
            self.stop()


if __name__ == "__main__":
    robot = FollowingRobot()
    robot.run()

