import socket
import numpy as np
import math
import time
import ddsm115 as motor
from rplidar import RPLidar
import threading

# Configuration for speed and parameters
speed = 80
rotasi = 2
batas = 135

# LIDAR parameters for obstacle avoidance
OBSTACLE_THRESHOLD = 400  # Distance in mm to detect obstacles (400mm = 40cm)
FRONT_ANGLES = list(range(340, 360)) + list(range(0, 20))  # Front angles (340° - 20°)
LEFT_ANGLES = list(range(20, 90))    # Left angles (20° - 90°)
RIGHT_ANGLES = list(range(270, 340)) # Right angles (270° - 340°)

class UWBTracker:
    """Handles UWB data processing and position estimation"""
    def __init__(self):
        # Default bias correction values
        self.bias = {
            'A0': 50.0,  # Example bias value in cm
            'A1': 50.0,   # Example bias value in cm
            'A2': 50.0   # Example bias value in cm
        }
        
        # Default scale factor values
        self.scale_factor = {
            'A0': 1.03,  # Example scale factor
            'A1': 1.05,  # Example scale factor
            'A2': 1.05   # Example scale factor
        }

    def apply_bias_correction(self, distances):
        """Apply bias correction and scaling to distance measurements"""
        corrected_distances = {
            'A0': max((distances['A0'] * 100 * self.scale_factor['A0']) - self.bias['A0'], 0),
            'A1': max((distances['A1'] * 100 * self.scale_factor['A1']) - self.bias['A1'], 0),
            'A2': max((distances['A2'] * 100 * self.scale_factor['A2']) - self.bias['A2'], 0)
        }
        return corrected_distances

class LidarProcessor:
    """Processes LIDAR data for obstacle detection"""
    def __init__(self, port='/dev/ttyUSB0'):
        self.lidar = None
        self.port = port
        self.scan_data = [0] * 360
        self.running = False
        self.obstacle_detected = {
            'front': False,
            'left': False,
            'right': False
        }
        self.lock = threading.Lock()  # For thread safety
        self.scan_thread = None
        self.retry_count = 0
        self.max_retries = 5
    
    def start(self):
        """Start Lidar scanning"""
        # Initialize LIDAR with longer timeout for A2M12
        try:
            print("Initializing LIDAR A2M12...")
            self.lidar = RPLidar(self.port, timeout=3)
            self.lidar.connect()
            self.lidar.start_motor()
            time.sleep(2)  # Give time for motor to reach proper speed
            
            # Check health
            health = self.lidar.get_health()
            print(f"LIDAR health status: {health}")
            
            if health[0] == 'Error':
                raise Exception(f"LIDAR health error: {health[1]}")
                
            self.running = True
            self.scan_thread = threading.Thread(target=self._scan)
            self.scan_thread.daemon = True
            self.scan_thread.start()
            print("LIDAR A2M12 started and scanning.")
            return True
        except Exception as e:
            print(f"Failed to start LIDAR: {e}")
            self.retry_start()
            return False

    def retry_start(self):
        """Retry starting the LIDAR if initial attempt fails"""
        if self.retry_count < self.max_retries:
            self.retry_count += 1
            print(f"Retrying LIDAR start (attempt {self.retry_count}/{self.max_retries})...")
            time.sleep(2)  # Wait before retry
            if self.lidar:
                try:
                    self.lidar.stop_motor()
                    self.lidar.disconnect()
                except:
                    pass
            self.start()
        else:
            print("Maximum retry attempts reached. Please check LIDAR connection and hardware.")

    def _scan(self):
        """Continuous LIDAR data collection"""
        try:
            print("Collecting LIDAR data...")
            scan_count = 0
            
            # For A2M12, use iter_measures instead of iter_scans for better compatibility
            for i, measure in enumerate(self.lidar.iter_measures()):
                if not self.running:
                    break
                    
                # Unpack measurement data
                _, angle, distance = measure
                
                with self.lock:
                    angle_int = int(angle)
                    if 0 <= angle_int < 360:  # Validate angle
                        self.scan_data[angle_int] = distance
                
                # Only process after collecting a good amount of data points
                scan_count += 1
                if scan_count >= 90:  # Process after every ~quarter scan
                    with self.lock:
                        self._detect_obstacles()
                    scan_count = 0
                
                # A small sleep to reduce CPU usage
                time.sleep(0.0001)
        except Exception as e:
            print(f"LIDAR scanning error: {e}")
            # Try to recover
            if self.running:
                print("Attempting to restart LIDAR scan...")
                self.restart_scan()
        finally:
            self.stop()

    def restart_scan(self):
        """Restart scanning after an error"""
        try:
            if self.lidar:
                self.lidar.stop_motor()
                time.sleep(1)
                self.lidar.start_motor()
                time.sleep(2)
                
                # Start a new scan thread
                if self.running:
                    self.scan_thread = threading.Thread(target=self._scan)
                    self.scan_thread.daemon = True
                    self.scan_thread.start()
        except Exception as e:
            print(f"Failed to restart LIDAR scan: {e}")

    def _detect_obstacles(self):
        """Detect obstacles in front, left, and right"""
        # Collect valid distances
        front_distances = []
        left_distances = []
        right_distances = []
        
        for angle in FRONT_ANGLES:
            if self.scan_data[angle] > 0:
                front_distances.append(self.scan_data[angle])
                
        for angle in LEFT_ANGLES:
            if self.scan_data[angle] > 0:
                left_distances.append(self.scan_data[angle])
                
        for angle in RIGHT_ANGLES:
            if self.scan_data[angle] > 0:
                right_distances.append(self.scan_data[angle])
        
        # Calculate minimum distances if valid data exists
        front_min = min(front_distances) if front_distances else float('inf')
        left_min = min(left_distances) if left_distances else float('inf')
        right_min = min(right_distances) if right_distances else float('inf')
        
        # Update obstacle detection status based on thresholds
        self.obstacle_detected['front'] = front_min < OBSTACLE_THRESHOLD
        self.obstacle_detected['left'] = left_min < OBSTACLE_THRESHOLD
        self.obstacle_detected['right'] = right_min < OBSTACLE_THRESHOLD
    
    def get_obstacle_status(self):
        """Get the current obstacle detection status"""
        with self.lock:
            # Get the minimum distances in each direction for decision making
            front_distances = [self.scan_data[angle] for angle in FRONT_ANGLES if self.scan_data[angle] > 0]
            left_distances = [self.scan_data[angle] for angle in LEFT_ANGLES if self.scan_data[angle] > 0]
            right_distances = [self.scan_data[angle] for angle in RIGHT_ANGLES if self.scan_data[angle] > 0]
            
            front_min = min(front_distances) if front_distances else float('inf')
            left_min = min(left_distances) if left_distances else float('inf')
            right_min = min(right_distances) if right_distances else float('inf')
            
            return {
                'front': self.obstacle_detected['front'],
                'left': self.obstacle_detected['left'],
                'right': self.obstacle_detected['right'],
                'front_distance': front_min,
                'left_distance': left_min,
                'right_distance': right_min
            }
        
    def stop(self):
        """Stop LIDAR scanning"""
        self.running = False
        if self.scan_thread and self.scan_thread.is_alive():
            self.scan_thread.join(timeout=1.0)
        
        if self.lidar:
            try:
                self.lidar.stop_motor()
                self.lidar.disconnect()
                print("LIDAR stopped and disconnected.")
            except Exception as e:
                print(f"Error stopping LIDAR: {e}")

class RobotController:
    """Controls robot movement based on UWB and LIDAR data"""
    def __init__(self, r_wheel_port, l_wheel_port):
        # Initialize motor controllers
        self.r_motor = motor.Motor(r_wheel_port)
        self.l_motor = motor.Motor(l_wheel_port)
        
        # Flag to track if we're currently avoiding obstacles
        self.avoiding_obstacles = False
        self.last_obstacle_time = 0
    
    def move(self, left_speed, right_speed):
        """Set motor speeds with safety limits"""
        # Apply safety limits
        left_speed = max(min(left_speed, 100), -100)
        right_speed = max(min(right_speed, 100), -100)
        
        # Send commands to motors
        self.l_motor.setSpeed(left_speed)
        self.r_motor.setSpeed(right_speed)
    
    def stop(self):
        """Stop all motors"""
        self.l_motor.setSpeed(0)
        self.r_motor.setSpeed(0)
    
    def analyze_and_act(self, distances, obstacle_status):
        """Analyze sensor data and control robot movement"""
        # Check for obstacles first
        front_obstacle = obstacle_status['front']
        left_obstacle = obstacle_status['left']
        right_obstacle = obstacle_status['right']
        
        # Print obstacle status
        print("\n--- Obstacle Status ---")
        print(f"Front: {front_obstacle} ({obstacle_status['front_distance']:.0f}mm)")
        print(f"Left: {left_obstacle} ({obstacle_status['left_distance']:.0f}mm)")
        print(f"Right: {right_obstacle} ({obstacle_status['right_distance']:.0f}mm)")
        
        # If obstacle detected, prioritize avoidance
        if front_obstacle or left_obstacle or right_obstacle:
            self.last_obstacle_time = time.time()
            
            if front_obstacle:
                # If obstacle in front, check left and right
                if not right_obstacle and (right_obstacle or obstacle_status['right_distance'] > obstacle_status['left_distance']):
                    print("Obstacle avoidance: Turn right")
                    self.move(0, speed)  # Turn right
                elif not left_obstacle:
                    print("Obstacle avoidance: Turn left")
                    self.move(-speed, 0)  # Turn left
                else:
                    # If all directions blocked, back up slightly
                    print("Obstacle avoidance: Backing up")
                    self.move(speed/2, -speed/2)  # Back up
                return
            elif left_obstacle and not right_obstacle:
                print("Obstacle avoidance: Avoiding left obstacle")
                self.move(0, speed)  # Turn right
                return
            elif right_obstacle and not left_obstacle:
                print("Obstacle avoidance: Avoiding right obstacle")
                self.move(-speed, 0)  # Turn left
                return

        # If no obstacles, follow UWB navigation
        A0, A1, A2 = distances['A0'], distances['A1'], distances['A2']
        
        # Print current distance values
        print("\n--- UWB Corrected Distances (cm) ---")
        print(f"A0: {A0:.2f} | A1: {A1:.2f} | A2: {A2:.2f}")
        
        # If target reached
        if A0 <= batas:
            print(f"Stopping as target reached (A0 <= {batas} cm)")
            self.stop()
            return
        
        # If UWB available, navigate based on UWB tracker
        if A0 < A1 and A0 < A2:
            print("Move Forward - A0 is closest")
            self.move(-speed, speed)  # Move forward with equal speed
        elif (A1 < A2):
            if (A2 > A1 and A2 > A0):
                print("Rotate Left - A1 is close")
                self.move(-speed/rotasi, 0)  # Left rotation
            else:
                print("Rotate Left - A1 is closest")
                self.move(-speed/rotasi, 0)  # Left rotation
        elif (A2 < A1):
            if (A1 > A2 and A1 > A0):
                print("Rotate Right - A2 is closest")
                self.move(0, speed/rotasi)  # Right rotation
            else:
                print("Rotate Right - A2 is closest")
                self.move(0, speed/rotasi)  # Right rotation
        elif (A0 > A1 and A0 > A2):
            print("Rotasi")
            self.move(speed, -speed)
        else:
            print("Maintaining current movement - No clear direction")

def main_robot_loop():
    # Communication setup
    r_wheel_port = "/dev/robot_rwheel"
    l_wheel_port = "/dev/robot_lwheel"
    UDP_IP = "192.168.80.113"
    UDP_PORT = 5005
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    
    # Initialize UWB distances
    raw_uwb_distances = {'A0': 1000, 'A1': 1000, 'A2': 1000}
    print("Initial UWB distances:", raw_uwb_distances)
    
    # Initialize UWB tracker with bias correction
    uwb_tracker = UWBTracker()
    
    # Initialize robot controller
    robot = RobotController(r_wheel_port, l_wheel_port)
    
    # Initialize LIDAR
    lidar_processor = LidarProcessor(port="/dev/robot_lidar")
    if not lidar_processor.start():
        print("Failed to start LIDAR. Running without obstacle avoidance.")
    
    try:
        print("Starting robot...")
        while True:
            # Receive UWB distance data with timeout
            try:
                sock.settimeout(0.5)  # Set timeout for UWB data reception
                data, addr = sock.recvfrom(1024)
                parts = data.decode().split(",")
                
                # Update raw distances
                if len(parts) >= 3:
                    try:
                        raw_uwb_distances['A0'] = float(parts[0])
                        raw_uwb_distances['A1'] = float(parts[1])
                        raw_uwb_distances['A2'] = float(parts[2])
                        
                        print("\n--- New UWB Data Received ---")
                        print(f"Raw data: {data.decode()}")
                        print(f"Raw distances - A0: {raw_uwb_distances['A0']}, A1: {raw_uwb_distances['A1']}, A2: {raw_uwb_distances['A2']}")
                        
                        # Apply bias correction
                        corrected_distances = uwb_tracker.apply_bias_correction(raw_uwb_distances)
                        print(f"Corrected distances - A0: {corrected_distances['A0']:.2f}, A1: {corrected_distances['A1']:.2f}, A2: {corrected_distances['A2']:.2f}")
                    except ValueError as e:
                        print(f"Error parsing UWB data: {e}")
                        continue
                else:
                    print("Invalid UWB data format")
                    continue
            except socket.timeout:
                print("UWB data reception timeout, continuing...")
                
            # Get obstacle status from LIDAR
            if lidar_processor.running:
                obstacle_status = lidar_processor.get_obstacle_status()
            else:
                # Default empty obstacle status if LIDAR is not running
                obstacle_status = {
                    'front': False, 'left': False, 'right': False,
                    'front_distance': float('inf'), 
                    'left_distance': float('inf'), 
                    'right_distance': float('inf')
                }
                
            # Analyze and act based on corrected UWB distances and LIDAR data
            robot.analyze_and_act(corrected_distances, obstacle_status)
            
            # Small sleep to prevent CPU hogging
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Terminating robot process.")
    except Exception as e:
        print(f"Unexpected error in main loop: {e}")
    finally:
        lidar_processor.stop()
        robot.stop()
        sock.close()
        print("All systems shut down.")

if __name__ == "__main__":
    main_robot_loop()
