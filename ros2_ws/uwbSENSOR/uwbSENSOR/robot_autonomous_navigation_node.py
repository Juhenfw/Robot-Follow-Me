import socket
import numpy as np
import math
import time
# import ddsm115 as motor
from ddsm115 import MotorControl
import threading

speed = 70
rotasi = 5
batas = 60

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
            'A0': 1.0,  # Example scale factor
            'A1': 1.06,  # Example scale factor
            'A2': 1.06   # Example scale factor
        }
    
    def apply_bias_correction(self, distances):
        """Koreksi bias dan scaling pada pengukuran jarak"""
        corrected_distances = {
            'A0': max((distances['A0'] * 100 * self.scale_factor['A0']) - self.bias['A0'], 0),
            'A1': max((distances['A1'] * 100 * self.scale_factor['A1']) - self.bias['A1'], 0),
            'A2': max((distances['A2'] * 100 * self.scale_factor['A2']) - self.bias['A2'], 0)
        }
        return corrected_distances


# class LidarProcessor:
#     """Processes LIDAR data for obstacle detection"""
    
#     def __init__(self, port='/dev/ttyUSB0'):
#         # self.lidar = RPLidar(port)
#         self.scan_data = [0] * 360
#         self.running = True

#     def start(self):
#         """Start Lidar scanning"""
#         self.lidar.connect()
#         self.lidar.start_motor()
#         self.running = True
#         self.scan_thread = threading.Thread(target=self._scan)
#         self.scan_thread.start()
#         print("LIDAR started and scanning.")

#     def _scan(self):
#         """Continuous LIDAR data collection"""
#         try:
#             print("Collecting LIDAR data...")
#             for scan in self.lidar.iter_scans():
#                 if not self.running:
#                     break
#                 for _, angle, distance in scan:
#                     self.scan_data[int(angle)] = distance
#         except Exception as e:
#             print(f"LIDAR error: {e}")
#         finally:
#             self.stop()

#     def stop(self):
#         """Stop LIDAR scanning"""
#         self.running = False
#         self.lidar.stop_motor()
#         self.lidar.disconnect()
#         print("LIDAR stopped.")


class RobotController:
    """Controls the robot's movement based on UWB and LIDAR data"""

    def __init__(self, r_wheel_port, l_wheel_port):
        self.right_motor = MotorControl(device=r_wheel_port)
        self.left_motor = MotorControl(device=l_wheel_port)
        self.right_motor.set_drive_mode(1, 2)
        self.left_motor.set_drive_mode(1, 2)

    def move(self, right_speed, left_speed):
        """Move the robot"""
        self.right_motor.send_rpm(1, right_speed)
        self.left_motor.send_rpm(1, left_speed)

    def stop(self):
        """Stop the robot movement"""
        self.move(0, 0)

    def analyze_and_act(self, distances):
        """
        Decide the robot's action based on UWB sensor distances.

        If:
            Jarak A0 < (A1 & A2): Robot moves forward
            Jarak A1 < (A0 & A2): Robot rotates right
            Jarak A2 < (A0 & A1): Robot rotates left
        """
        A0, A1, A2 = distances['A0'], distances['A1'], distances['A2']
        
        # Print current distance values
        print("UWB Corrected Distances (cm):")
        print(f"A0: {A0:.2f} | A1: {A1:.2f} | A2: {A2:.2f}")

        if A0 < A1 and A0 < A2 and (-20 < (A2-A1) < 20):
        # if A0 < min(A1,A2):
            print("Move Forward - A0  is closest")
            self.move(-speed, speed)  # Move forward with equal speed

        elif (A1 < A2 and A0 < A2):
            print("Serong kiri")
            self.move(-speed, speed/rotasi)  # Left serong 

        elif (A2 < A1 and A0 < A1):
            print("Serong kanan")
            self.move(-speed/rotasi, speed)  # Right serong

        elif (A2 < A1):
            print("Rotate Right - A2 is closest")
            self.move(-speed/rotasi, speed)  # Right rotation

        elif (A1 < A2):
            print("Rotate Left - A1 is closest")
            self.move(-speed, speed/rotasi)   # Left rotation

        elif (A0 > A1 or A0 > A2):
            print("Rotasi")
            self.move(speed/rotasi, speed/rotasi) #Rotation

        else:
            # self.stop()
            print("Maintaining current movement - No clear direction")

        # Stop when A0 distance reaches the threshold of 150 cm
        if A0 <= batas:
            print(f"Stopping as target reached (A0 <= {batas} cm)")
            self.stop()
    

def main_robot_loop():
    # Communication setup
    r_wheel_port = "/dev/ttyRS485-1"
    l_wheel_port = "/dev/ttyRS485-2"
    UDP_IP = "192.168.102.128"
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
    # lidar_processor = LidarProcessor(port="/dev/robot_lidar")
    # lidar_processor.start()

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

            # Analyze and act based on corrected UWB distances
            robot.analyze_and_act(corrected_distances)
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("Terminating robot process.")
    finally:
        # lidar_processor.stop()
        robot.stop()
        sock.close()
        print("All systems shut down.")


if __name__ == "__main__":
    main_robot_loop()