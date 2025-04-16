import socket
import numpy as np
import math
import time
import ddsm115 as motor
from rplidar import RPLidar
import threading
import pygame
import os

speed = 80
rotasi = 2
batas = 135

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
    
    def __init__(self, port='/dev/robot_lidar'):
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

    def __init__(self, r_wheel_port, l_wheel_port, lidar_processor):
        self.right_motor = motor.MotorControl(device=r_wheel_port)
        self.left_motor = motor.MotorControl(device=l_wheel_port)
        self.right_motor.set_drive_mode(1, 2)
        self.left_motor.set_drive_mode(1, 2)
        self.lidar = lidar_processor

    def move(self, right_speed, left_speed):
        self.right_motor.send_rpm(1, right_speed)
        self.left_motor.send_rpm(1, left_speed)

    def stop(self):
        self.move(0, 0)
        time.sleep(0.2)

    def is_obstacle_detected(self, threshold=500):  # 50 cm
        # Ambil area depan: ±30 derajat dari depan (0 derajat)
        front_angles = list(range(330, 360)) + list(range(0, 31))
        for angle in front_angles:
            if 0 < self.lidar.scan_data[angle] < threshold:
                print(f"Obstacle detected at angle {angle} with distance {self.lidar.scan_data[angle]} mm")
                return True
        return False
    
    def detect_obstacle_direction(self, threshold=500):
        """
        Mengecek area sekitar (kiri, kanan, depan) untuk rintangan.
        Mengembalikan 'left', 'right', 'front', atau None.
        """
        front = list(range(330, 360)) + list(range(0, 31))
        left = list(range(60, 121))
        right = list(range(240, 301))

        obstacle_front = any(0 < self.lidar.scan_data[a] < threshold for a in front)
        obstacle_left = any(0 < self.lidar.scan_data[a] < threshold for a in left)
        obstacle_right = any(0 < self.lidar.scan_data[a] < threshold for a in right)

        if obstacle_front:
            return "front"
        elif obstacle_left:
            return "left"
        elif obstacle_right:
            return "right"
        return None

    def analyze_and_act(self, distances):
        A0, A1, A2 = distances['A0'], distances['A1'], distances['A2']

        print("UWB Corrected Distances (cm):")
        print(f"A0: {A0:.2f} | A1: {A1:.2f} | A2: {A2:.2f}")

        obstacle_direction = self.detect_obstacle_direction()
        if obstacle_direction == "front":
            print("Obstacle di depan! Berhenti.")
            self.stop()
            return
        elif obstacle_direction == "left":
            print("Obstacle di kiri! Belok kanan.")
            self.move(0, speed/rotasi)
            return
        elif obstacle_direction == "right":
            print("Obstacle di kanan! Belok kiri.")
            self.move(-speed/rotasi, 0)
            return
        if A0 < A1 and A0 < A2:
            print("Move Forward - A0 is closest")
            self.move(-speed, speed)
        elif A1 > A2 and A1 > A0:
            print("Rotate Right - A2 is closest")
            self.move(0, speed/rotasi)
        elif A2 > A1 and A2 > A0:
            print("Rotate Left - A1 is closest")
            self.move(-speed/rotasi, 0)
        elif A2 < A1:
            print("Rotate Right - A2 is closest")
            self.move(0, speed/rotasi)
        elif A1 < A2:
            print("Rotate Left - A1 is closest")
            self.move(-speed/rotasi, 0)
        elif A0 > A1 or A0 > A2:
            print("Rotasi")
            self.move(speed/rotasi, speed/rotasi)
        else:
            print("Maintaining current movement - No clear direction")

        if A0 <= batas:
            print(f"Stopping as target reached (A0 <= {batas} cm)")
            self.stop()


def visualize_lidar_pygame(lidar_processor):
    pygame.init()
    width, height = 600, 600
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("LIDAR Visualization")

    center = (width // 2, height // 2)
    scale = 0.05  # 1 mm = 0.05 pixels → 5000mm = 250px

    clock = pygame.time.Clock()

    running = True
    while running:
        screen.fill((0, 0, 0))  # Clear screen

        for angle in range(360):
            dist = lidar_processor.scan_data[angle]
            if 0 < dist < 5000:
                rad = math.radians(angle)
                x = int(center[0] + math.cos(rad) * dist * scale)
                y = int(center[1] - math.sin(rad) * dist * scale)
                pygame.draw.circle(screen, (0, 255, 0), (x, y), 2)

        pygame.draw.circle(screen, (255, 0, 0), center, 4)  # Center point
        pygame.display.flip()

        # Handle quit event
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                break

        clock.tick(30)

    pygame.quit()


class GamepadController:
    def __init__(self):
        self.r_wheel_port = "/dev/robot_rwheel"
        self.l_wheel_port = "/dev/robot_lwheel"

        # Inisialisasi pygame
        pygame.init()
        self.screen = pygame.display.set_mode((400, 300))
        pygame.display.set_caption("Kontrol Robot DDSM115 - Gamepad Mode")
        self.clock = pygame.time.Clock()

        # Kecepatan
        self.speed = 100
        self.lambat = self.speed / 4
        self.serong = self.speed / 3
        self.drift_boost = self.speed * 1.3
        self.stop = 0

        self.running = True
        self.current_speed_rwheel = self.stop
        self.current_speed_lwheel = self.stop

        self.motor1 = None
        self.motor2 = None
        self.connect_motors()

        # Joystick
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print("Joystick terdeteksi dan diinisialisasi")
        else:
            print("Joystick tidak terdeteksi. Harap colokkan dan restart.")
            self.running = False

    def verify_port_exists(self, port):
        return os.path.exists(port)

    def connect_motors(self):
        if self.verify_port_exists(self.r_wheel_port):
            try:
                self.motor1 = motor.MotorControl(device=self.r_wheel_port)
                self.motor1.set_drive_mode(1, 2)
            except Exception as e:
                print(f"Error motor kanan: {e}")
                self.motor1 = None

        if self.verify_port_exists(self.l_wheel_port):
            try:
                self.motor2 = motor.MotorControl(device=self.l_wheel_port)
                self.motor2.set_drive_mode(1, 2)
            except Exception as e:
                print(f"Error motor kiri: {e}")
                self.motor2 = None

    def safe_send_rpm(self, motor_obj, motor_id, speed):
        if not motor_obj:
            return False
        try:
            motor_obj.send_rpm(motor_id, speed)
            return True
        except Exception as e:
            print(f"Error kirim RPM: {e}")
            return False

    def run(self, robot, lidar_processor, uwb_tracker):
        while self.running:
            self.screen.fill((0, 0, 0))
            pygame.display.flip()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

            pygame.event.pump()
            joystick = self.joystick

            axis_lr = joystick.get_axis(0)
            axis_fb = joystick.get_axis(1)
            button_y = joystick.get_button(3)
            button_a = joystick.get_button(1)
            button_x = joystick.get_button(0)
            button_b = joystick.get_button(2)
            button_rb = joystick.get_button(5)
            button_lb = joystick.get_button(4)
            button_back = joystick.get_button(8)
            button_switch = joystick.get_button(9)

            # Mode switching logic
            if button_back:  # Close the program
                print("Closing Program")
                return "close"

            if button_switch:  # Switch to Autonomous mode
                print("Switching to Autonomous Mode")
                return "autonomous"

            self.current_speed_rwheel = self.stop
            self.current_speed_lwheel = self.stop

            if button_a:  # MAJU
                self.current_speed_rwheel = -self.speed
                self.current_speed_lwheel = self.speed

            if button_x:  # MUNDUR
                self.current_speed_rwheel = self.speed
                self.current_speed_lwheel = -self.speed

            if button_rb:  # Tambah Speed
                self.speed += 10

            if button_lb:  # Kurang Speed
                self.speed -= 10

            if abs(axis_lr) > 0.2:
                speed_factor = -axis_lr
                self.current_speed_rwheel = int(-speed_factor * self.lambat)
                self.current_speed_lwheel = int(-speed_factor * self.lambat)

            self.safe_send_rpm(self.motor1, 1, self.current_speed_rwheel)
            self.safe_send_rpm(self.motor2, 1, self.current_speed_lwheel)

            self.clock.tick(30)
            time.sleep(0.01)

            print(f"Kecepatan = {self.speed}")

    def cleanup(self):
        print("Menutup koneksi motor...")
        if self.motor1:
            try:
                self.motor1.send_rpm(1, 0)
                self.motor1.close()
            except:
                pass
        if self.motor2:
            try:
                self.motor2.send_rpm(1, 0)
                self.motor2.close()
            except:
                pass
        pygame.quit()


def main_robot_loop():
    r_wheel_port = "/dev/robot_rwheel"
    l_wheel_port = "/dev/robot_lwheel"
    UDP_IP = "192.168.151.113"
    UDP_PORT = 5005
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    raw_uwb_distances = {'A0': 1000, 'A1': 1000, 'A2': 1000}
    uwb_tracker = UWBTracker()

    lidar_processor = LidarProcessor(port="/dev/robot_lidar")
    lidar_processor.start()

    visual_thread = threading.Thread(target=visualize_lidar_pygame, args=(lidar_processor,))
    visual_thread.daemon = True
    visual_thread.start()

    robot = RobotController(r_wheel_port, l_wheel_port, lidar_processor)
    gamepad_controller = GamepadController()

    try:
        print("Starting robot...")
        mode = "manual"  # Default mode is manual
        while True:
            if mode == "manual":
                mode = gamepad_controller.run(robot, lidar_processor, uwb_tracker)
            elif mode == "autonomous":
                while True:
                    data, addr = sock.recvfrom(1024)
                    parts = data.decode().split(",")
                    raw_uwb_distances['A0'] = float(parts[0])
                    raw_uwb_distances['A1'] = float(parts[1])
                    raw_uwb_distances['A2'] = float(parts[2])

                    print("\n--- New UWB Data Received ---")
                    print(f"Raw data: {data.decode()}")

                    corrected_distances = uwb_tracker.apply_bias_correction(raw_uwb_distances)
                    print(f"Corrected distances - A0: {corrected_distances['A0']:.2f}, A1: {corrected_distances['A1']:.2f}, A2: {corrected_distances['A2']:.2f}")

                    robot.analyze_and_act(corrected_distances)
                    time.sleep(0.00001)
                    # Check if we should switch back to manual mode
                    if mode != "autonomous":
                        break
    except KeyboardInterrupt:
        print("Terminating robot process.")
    finally:
        lidar_processor.stop()
        robot.stop()
        sock.close()
        print("All systems shut down.")

if __name__ == "__main__":
    main_robot_loop()
