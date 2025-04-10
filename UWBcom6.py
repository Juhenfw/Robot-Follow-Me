import socket
import numpy as np
import math
import time
import ddsm115 as motor
from rplidar import RPLidar
import threading


class LidarProcessor:
    """Processes LIDAR data for obstacle detection"""
    
    def __init__(self, port='/dev/ttyUSB0'):
        self.lidar = RPLidar(port)
        self.scan_data = [0] * 360
        self.running = True

    def start(self):
        """Start Lidar scanning"""
        self.lidar.connect()
        self.lidar.start_motor()
        self.running = True
        self.scan_thread = threading.Thread(target=self._scan)
        self.scan_thread.start()
        print("LIDAR started and scanning.")

    def _scan(self):
        """Continuous LIDAR data collection"""
        try:
            print("Collecting LIDAR data...")
            for scan in self.lidar.iter_scans():
                if not self.running:
                    break
                for _, angle, distance in scan:
                    self.scan_data[int(angle)] = distance
        except Exception as e:
            print(f"LIDAR error: {e}")
        finally:
            self.stop()

    def stop(self):
        """Stop LIDAR scanning"""
        self.running = False
        self.lidar.stop_motor()
        self.lidar.disconnect()
        print("LIDAR stopped.")


class RobotController:
    """Controls the robot's movement based on UWB and LIDAR data"""

    def __init__(self, r_wheel_port, l_wheel_port):
        self.right_motor = motor.MotorControl(device=r_wheel_port)
        self.left_motor = motor.MotorControl(device=l_wheel_port)
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
        print("UWB Distances (cm):")
        print(f"A0: {A0:.2f} | A1: {A1:.2f} | A2: {A2:.2f}")

        if A0 < A1 and A0 < A2:
            print("Move Forward - A0 is closest")
            self.move(100, 100)  # Move forward with equal speed
        elif A1 < A0 and A1 < A2:
            print("Rotate Right - A1 is closest")
            self.move(100, -100)  # Right rotation
        elif A2 < A0 and A2 < A1:
            print("Rotate Left - A2 is closest")
            self.move(-100, 100)  # Left rotation
        else:
            print("Maintaining current movement - No clear direction")

        # Stop when A0 distance reaches the threshold of 150 cm
        if A0 <= 150:
            print("Stopping as target reached (A0 <= 150 cm)")
            self.stop()


def main_robot_loop():
    # Communication setup
    r_wheel_port = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B0049TUZ-if00-port0"
    l_wheel_port = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B0045S9B-if00-port0"
    UDP_IP = "192.168.80.113"
    UDP_PORT = 5005
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    # Initialize UWB distances
    uwb_distances = {'A0': 1000, 'A1': 1000, 'A2': 1000}
    print("Initial UWB distances:", uwb_distances)

    # Initialize robot controller
    robot = RobotController(r_wheel_port, l_wheel_port)

    # Initialize LIDAR
    lidar_processor = LidarProcessor(port="/dev/ttyUSB0")
    lidar_processor.start()

    try:
        print("Starting robot...")
        while True:
            # Receive UWB distance data
            data, addr = sock.recvfrom(1024)
            parts = data.decode().split(",")
            
            # Update distances and print received values
            uwb_distances['A0'] = float(parts[0])
            uwb_distances['A1'] = float(parts[1])
            uwb_distances['A2'] = float(parts[2])
            
            print("\n--- New UWB Data Received ---")
            print(f"Raw data: {data.decode()}")
            print(f"A0: {uwb_distances['A0']}, A1: {uwb_distances['A1']}, A2: {uwb_distances['A2']}")

            # Analyze and act based on UWB distances
            robot.analyze_and_act(uwb_distances)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Terminating robot process.")
    finally:
        lidar_processor.stop()
        robot.stop()
        sock.close()
        print("All systems shut down.")


if __name__ == "__main__":
    main_robot_loop()
