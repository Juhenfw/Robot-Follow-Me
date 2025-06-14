import socket
import numpy as np
import math
import time
from ddsm115 import MotorControl

speed = 80
rotasi = 2
batas = 70
max_speed = 70  # Kecepatan maksimum untuk pergerakan

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
            'A1': 1.0,  # Example scale factor
            'A2': 1.0  # Example scale factor
        }
    
    def apply_bias_correction(self, distances):
        """Koreksi bias dan scaling pada pengukuran jarak"""
        corrected_distances = {
            'A0': max((distances['A0'] * 100 * self.scale_factor['A0']) - self.bias['A0'], 0),
            'A1': max((distances['A1'] * 100 * self.scale_factor['A1']) - self.bias['A1'], 0),
            'A2': max((distances['A2'] * 100 * self.scale_factor['A2']) - self.bias['A2'], 0)
        }
        return corrected_distances


class RobotController:
    """Controls the robot's movement based on UWB data"""

    def __init__(self, r_wheel_port, l_wheel_port):
        self.right_motor = MotorControl(device=r_wheel_port)
        self.left_motor = MotorControl(device=l_wheel_port)
        self.right_motor.set_drive_mode(1, 2)
        self.left_motor.set_drive_mode(1, 2)

    def move(self, right_speed, left_speed):
        """Move the robot with precise control over both motors"""
        right_speed = np.clip(right_speed, -max_speed, max_speed)
        left_speed = np.clip(left_speed, -max_speed, max_speed)
        self.right_motor.send_rpm(1, right_speed)
        self.left_motor.send_rpm(1, left_speed)

    def stop(self):
        """Stop the robot movement"""
        self.move(0, 0)

    def analyze_and_act(self, distances):
        """
        Decide the robot's action based on UWB sensor distances with smoother movement.
        The logic uses soft and responsive control for robot movement.
        """
        A0, A1, A2 = distances['A0'], distances['A1'], distances['A2']
        
        # Print current distance values
        print("UWB Corrected Distances (cm):")
        print(f"A0: {A0:.2f} | A1: {A1:.2f} | A2: {A2:.2f}")

        if A0 <= batas:
            print(f"Stopping as target reached (A0 <= {batas} cm)")
            self.stop()
        else:
            # Menghitung error arah berdasarkan jarak sensor
            error_right = A1 - A2  # Error antara A1 dan A2 untuk menentukan arah pergerakan
            error_forward = A0 - min(A1, A2)  # Error untuk pergerakan maju

            # Kecepatan kanan dan kiri berdasarkan error yang dihitung
            right_speed = speed * (1 - error_right / 100)  # Menghitung kecepatan untuk roda kanan
            left_speed = speed * (1 + error_right / 100)   # Menghitung kecepatan untuk roda kiri

            # Jika robot terlalu dekat dengan objek di depan, mundur sedikit
            if A0 < 30:
                print("Too close, moving back slightly.")
                # right_speed = speed / 3
                # left_speed = speed / 3
            elif error_forward > 0:  # Prioritaskan maju jika A0 lebih dekat
                print("Move Forward")
                self.move(-right_speed, left_speed)
            else:  # Jika error mengarah ke kanan atau kiri, berbelok
                if error_right > 0:
                    print("Rotate Right")
                    self.move(speed / (rotasi * 2), speed / rotasi)
                elif error_right < 0:
                    print("Rotate Left")
                    self.move(-speed / rotasi, -speed / (rotasi * 2))
                else:
                    print("Move Forward - A0 is closest")
                    self.move(right_speed, left_speed)

        time.sleep(0.001)  # Delay kecil untuk pergerakan halus


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
        robot.stop()
        sock.close()
        print("All systems shut down.")


if __name__ == "__main__":
    main_robot_loop()
