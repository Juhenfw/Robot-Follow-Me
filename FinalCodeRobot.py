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
OBSTACLE_THRESHOLD = 500  # Jarak dalam mm untuk mendeteksi rintangan
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
        self.lidar = RPLidar(port)
        self.scan_data = [0] * 360
        self.running = True
        self.obstacle_detected = {
            'front': False,
            'left': False,
            'right': False
        }
        self.lock = threading.Lock()  # Untuk thread safety

    def start(self):
        """Start Lidar scanning"""
        self.lidar.connect()
        self.lidar.start_motor()
        self.running = True
        self.scan_thread = threading.Thread(target=self._scan)
        self.scan_thread.daemon = True  # Daemon thread closes when main program exits
        self.scan_thread.start()
        print("LIDAR started and scanning.")

    def _scan(self):
        """Continuous LIDAR data collection"""
        try:
            print("Collecting LIDAR data...")
            for scan in self.lidar.iter_scans():
                if not self.running:
                    break
                with self.lock:
                    for _, angle, distance in scan:
                        angle_int = int(angle)
                        if angle_int < 360:  # Validasi sudut
                            self.scan_data[angle_int] = distance
                    self._detect_obstacles()
        except Exception as e:
            print(f"LIDAR error: {e}")
        finally:
            self.stop()

    def _detect_obstacles(self):
        """Deteksi rintangan di depan, kiri, dan kanan"""
        front_distances = [self.scan_data[angle] for angle in FRONT_ANGLES]
        left_distances = [self.scan_data[angle] for angle in LEFT_ANGLES]
        right_distances = [self.scan_data[angle] for angle in RIGHT_ANGLES]
        
        # Filter nilai 0 (mungkin error) dan ambil minimum yang valid
        front_min = min([d for d in front_distances if d > 0], default=float('inf'))
        left_min = min([d for d in left_distances if d > 0], default=float('inf'))
        right_min = min([d for d in right_distances if d > 0], default=float('inf'))
        
        self.obstacle_detected['front'] = front_min < OBSTACLE_THRESHOLD
        self.obstacle_detected['left'] = left_min < OBSTACLE_THRESHOLD
        self.obstacle_detected['right'] = right_min < OBSTACLE_THRESHOLD

    def get_obstacle_status(self):
        """Mendapatkan status rintangan"""
        with self.lock:
            return {
                'front': self.obstacle_detected['front'],
                'left': self.obstacle_detected['left'], 
                'right': self.obstacle_detected['right'],
                'front_distance': min([self.scan_data[angle] for angle in FRONT_ANGLES if self.scan_data[angle] > 0], default=float('inf')),
                'left_distance': min([self.scan_data[angle] for angle in LEFT_ANGLES if self.scan_data[angle] > 0], default=float('inf')),
                'right_distance': min([self.scan_data[angle] for angle in RIGHT_ANGLES if self.scan_data[angle] > 0], default=float('inf'))
            }

    def stop(self):
        """Stop LIDAR scanning"""
        self.running = False
        time.sleep(1)  # Memberi waktu thread untuk selesai
        try:
            self.lidar.stop_motor()
            self.lidar.disconnect()
        except:
            pass
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
        self.right_motor.send_rpm(1, right_speed)
        self.left_motor.send_rpm(1, left_speed)

    def stop(self):
        """Stop the robot movement"""
        self.move(0, 0)

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
                print("Rotate Left - A1 is closest")
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
            self.move(speed/rotasi, speed/rotasi) # Left Rotation
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
    lidar_processor.start()
    
    try:
        print("Starting robot...")
        while True:
            # Receive UWB distance data
            data, addr = sock.recvfrom(1024)
            parts = data.decode().split(",")
            
            # Update raw distances
            raw_uwb_distances['A0'] = float(parts[0])
            raw_uwb_distances['A1'] = float(parts[1])
            raw_uwb_distances['A2'] = float(parts[2])
            
            print("\n--- New UWB Data Received ---")
            print(f"Raw data: {data.decode()}")
            print(f"Raw distances - A0: {raw_uwb_distances['A0']}, A1: {raw_uwb_distances['A1']}, A2: {raw_uwb_distances['A2']}")
            
            # Apply bias correction
            corrected_distances = uwb_tracker.apply_bias_correction(raw_uwb_distances)
            print(f"Corrected distances - A0: {corrected_distances['A0']:.2f}, A1: {corrected_distances['A1']:.2f}, A2: {corrected_distances['A2']:.2f}")
            
            # Get obstacle status from LIDAR
            obstacle_status = lidar_processor.get_obstacle_status()
            
            # Analyze and act based on corrected UWB distances and LIDAR data
            robot.analyze_and_act(corrected_distances, obstacle_status)
            
            time.sleep(0.00001)
    except KeyboardInterrupt:
        print("Terminating robot process.")
    finally:
        lidar_processor.stop()
        robot.stop()
        sock.close()
        print("All systems shut down.")

if __name__ == "__main__":
    main_robot_loop()
