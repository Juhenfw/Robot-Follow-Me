import socket
import numpy as np
import math
import time
import ddsm115 as motor
from rplidar import RPLidar
import threading

# Konfigurasi kecepatan dan parameter
speed = 80
rotasi = 2
batas = 135

# Parameter LIDAR untuk penghindaran objek
OBSTACLE_THRESHOLD = 400  # Jarak dalam mm untuk mendeteksi rintangan (400mm = 40cm)
FRONT_ANGLES = list(range(340, 360)) + list(range(0, 20))  # Sudut depan (340° - 20°)
LEFT_ANGLES = list(range(20, 90))    # Sudut kiri (20° - 90°)
RIGHT_ANGLES = list(range(270, 340)) # Sudut kanan (270° - 340°)

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
        """Koreksi bias dan scaling pada pengukuran jarak"""
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
        self.lock = threading.Lock()  # Untuk thread safety
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
                    if 0 <= angle_int < 360:  # Validasi sudut
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
        """Deteksi rintangan di depan, kiri, dan kanan"""
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
        
        # Calculate minimums with fallbacks
        front_min = min(front_distances, default=float('inf'))
        left_min = min(left_distances, default=float('inf'))
        right_min = min(right_distances, default=float('inf'))
        
        # Apply obstacle detection
        self.obstacle_detected['front'] = front_min < OBSTACLE_THRESHOLD and front_min > 0
        self.obstacle_detected['left'] = left_min < OBSTACLE_THRESHOLD and left_min > 0
        self.obstacle_detected['right'] = right_min < OBSTACLE_THRESHOLD and right_min > 0

    def get_obstacle_status(self):
        """Mendapatkan status rintangan"""
        with self.lock:
            # Collect valid distances for each sector
            front_valid = [d for d in [self.scan_data[angle] for angle in FRONT_ANGLES] if d > 0]
            left_valid = [d for d in [self.scan_data[angle] for angle in LEFT_ANGLES] if d > 0]
            right_valid = [d for d in [self.scan_data[angle] for angle in RIGHT_ANGLES] if d > 0]
            
            return {
                'front': self.obstacle_detected['front'],
                'left': self.obstacle_detected['left'], 
                'right': self.obstacle_detected['right'],
                'front_distance': min(front_valid, default=float('inf')),
                'left_distance': min(left_valid, default=float('inf')),
                'right_distance': min(right_valid, default=float('inf'))
            }

    def stop(self):
        """Stop LIDAR scanning"""
        self.running = False
        time.sleep(1)  # Memberi waktu thread untuk selesai
        
        if self.lidar:
            try:
                self.lidar.stop_motor()
                time.sleep(0.5)
                self.lidar.disconnect()
            except Exception as e:
                print(f"Error stopping LIDAR: {e}")
                
        print("LIDAR stopped.")

class RobotController:
    """Controls the robot's movement based on UWB and LIDAR data"""
    def __init__(self, r_wheel_port, l_wheel_port):
        self.right_motor = motor.MotorControl(device=r_wheel_port)
        self.left_motor = motor.MotorControl(device=l_wheel_port)
        self.right_motor.set_drive_mode(1, 2)
        self.left_motor.set_drive_mode(1, 2)
        self.last_obstacle_time = 0
        self.obstacle_memory_duration = 2  # Ingat rintangan selama 2 detik

    def move(self, right_speed, left_speed):
        """Move the robot"""
        try:
            self.right_motor.send_rpm(1, right_speed)
            self.left_motor.send_rpm(1, left_speed)
        except Exception as e:
            print(f"Motor control error: {e}")

    def stop(self):
        """Stop the robot movement"""
        try:
            self.move(0, 0)
        except Exception as e:
            print(f"Error stopping motors: {e}")

    def analyze_and_act(self, distances, obstacle_status):
        """
        Decide the robot's action based on UWB sensor distances and LIDAR obstacle detection.
        Prioritizes obstacle avoidance over UWB-based navigation.
        """
        # Cek status rintangan dari LIDAR
        front_obstacle = obstacle_status['front']
        left_obstacle = obstacle_status['left']
        right_obstacle = obstacle_status['right']
        
        # Log LIDAR data
        print("\n--- LIDAR Obstacle Status ---")
        print(f"Front: {front_obstacle} ({obstacle_status['front_distance']:.0f}mm)")
        print(f"Left: {left_obstacle} ({obstacle_status['left_distance']:.0f}mm)")
        print(f"Right: {right_obstacle} ({obstacle_status['right_distance']:.0f}mm)")
        
        # Jika ada rintangan, prioritaskan penghindaran
        if front_obstacle or left_obstacle or right_obstacle:
            self.last_obstacle_time = time.time()
            
            if front_obstacle:
                # Jika rintangan di depan, cek kiri dan kanan
                if not right_obstacle and (right_obstacle or obstacle_status['right_distance'] > obstacle_status['left_distance']):
                    print("Obstacle avoidance: Turn right")
                    self.move(0, speed)  # Belok kanan
                elif not left_obstacle:
                    print("Obstacle avoidance: Turn left")
                    self.move(-speed, 0)  # Belok kiri
                else:
                    # Jika semua arah terhalang, mundur sedikit
                    print("Obstacle avoidance: Backing up")
                    self.move(speed/2, -speed/2)  # Mundur
                return
            elif left_obstacle and not right_obstacle:
                print("Obstacle avoidance: Avoiding left obstacle")
                self.move(0, speed)  # Belok kanan
                return
            elif right_obstacle and not left_obstacle:
                print("Obstacle avoidance: Avoiding right obstacle")
                self.move(-speed, 0)  # Belok kiri
                return
        
        # Jika tidak ada rintangan, ikuti navigasi UWB
        A0, A1, A2 = distances['A0'], distances['A1'], distances['A2']
        
        # Print current distance values
        print("\n--- UWB Corrected Distances (cm) ---")
        print(f"A0: {A0:.2f} | A1: {A1:.2f} | A2: {A2:.2f}")
        
        # Jika sudah mencapai target
        if A0 <= batas:
            print(f"Stopping as target reached (A0 <= {batas} cm)")
            self.stop()
            return
        
        # Jika UWB avaliable, navigasi berdasarkan UWB tracker
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
