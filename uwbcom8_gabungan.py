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
TIMEOUT = 3  # Waktu timeout untuk beralih ke mode autonomous (detik)

class UWBTracker:
    """Handles UWB data processing and position estimation"""
    
    def __init__(self):
        self.bias = {'A0': 50.0, 'A1': 50.0, 'A2': 50.0}
        self.scale_factor = {'A0': 1.03, 'A1': 1.05, 'A2': 1.05}
    
    def apply_bias_correction(self, distances):
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
        self.lidar.connect()
        self.lidar.start_motor()
        self.running = True
        self.scan_thread = threading.Thread(target=self._scan)
        self.scan_thread.start()

    def _scan(self):
        try:
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
        self.running = False
        self.lidar.stop_motor()
        self.lidar.disconnect()


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

    def is_obstacle_detected(self, threshold=500):
        front_angles = list(range(330, 360)) + list(range(0, 31))
        for angle in front_angles:
            if 0 < self.lidar.scan_data[angle] < threshold:
                return True
        return False
    
    def detect_obstacle_direction(self, threshold=500):
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

        obstacle_direction = self.detect_obstacle_direction()
        if obstacle_direction == "front":
            self.stop()
            return
        elif obstacle_direction == "left":
            self.move(0, speed/rotasi)
            return
        elif obstacle_direction == "right":
            self.move(-speed/rotasi, 0)
            return
        if A0 < A1 and A0 < A2:
            self.move(-speed, speed)
        elif A1 > A2 and A1 > A0:
            self.move(0, speed/rotasi)
        elif A2 > A1 and A2 > A0:
            self.move(-speed/rotasi, 0)
        elif A0 > A1 or A0 > A2:
            self.move(speed/rotasi, speed/rotasi)

        if A0 <= batas:
            self.stop()


class GamepadController:
    def __init__(self):
        self.r_wheel_port = "/dev/robot_rwheel"
        self.l_wheel_port = "/dev/robot_lwheel"
        pygame.init()
        self.screen = pygame.display.set_mode((400, 300))
        pygame.display.set_caption("Kontrol Robot - Gamepad Mode")
        self.clock = pygame.time.Clock()

        self.speed = 100
        self.stop = 0

        self.running = True
        self.current_speed_rwheel = self.stop
        self.current_speed_lwheel = self.stop

        self.motor1 = None
        self.motor2 = None
        self.connect_motors()

        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
        else:
            self.running = False

    def connect_motors(self):
        try:
            self.motor1 = motor.MotorControl(device=self.r_wheel_port)
            self.motor1.set_drive_mode(1, 2)
        except:
            self.motor1 = None
        try:
            self.motor2 = motor.MotorControl(device=self.l_wheel_port)
            self.motor2.set_drive_mode(1, 2)
        except:
            self.motor2 = None

    def safe_send_rpm(self, motor_obj, motor_id, speed):
        if not motor_obj:
            return False
        try:
            motor_obj.send_rpm(motor_id, speed)
            return True
        except:
            return False

    def run(self):
        axis_lr = self.joystick.get_axis(0)
        axis_fb = self.joystick.get_axis(1)
        button_a = self.joystick.get_button(1)
        button_x = self.joystick.get_button(0)
        button_b = self.joystick.get_button(2)
        button_rb = self.joystick.get_button(5)
        button_lb = self.joystick.get_button(4)
        button_back = self.joystick.get_button(8)

        self.current_speed_rwheel = self.stop
        self.current_speed_lwheel = self.stop

        if button_back:
            return "exit"

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
            self.current_speed_rwheel = int(-speed_factor * self.speed)
            self.current_speed_lwheel = int(-speed_factor * self.speed)

        self.safe_send_rpm(self.motor1, 1, self.current_speed_rwheel)
        self.safe_send_rpm(self.motor2, 1, self.current_speed_lwheel)

        self.clock.tick(30)
        return "manual"


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

    robot = RobotController(r_wheel_port, l_wheel_port, lidar_processor)
    gamepad_controller = GamepadController()

    mode = "autonomous"
    last_input_time = time.time()

    try:
        print("Starting robot...")

        while True:
            if mode == "autonomous":
                # Autonomous Mode: Check UWB and LIDAR
                data, addr = sock.recvfrom(1024)
                parts = data.decode().split(",")
                raw_uwb_distances['A0'] = float(parts[0])
                raw_uwb_distances['A1'] = float(parts[1])
                raw_uwb_distances['A2'] = float(parts[2])

                corrected_distances = uwb_tracker.apply_bias_correction(raw_uwb_distances)
                robot.analyze_and_act(corrected_distances)

                # Switch to manual mode if there is input from the gamepad
                if gamepad_controller.joystick.get_button(0) or gamepad_controller.joystick.get_button(1):
                    mode = "manual"
                    last_input_time = time.time()

                time.sleep(0.1)

            elif mode == "manual":
                # Manual Mode: Gamepad control
                result = gamepad_controller.run()
                if result == "exit":
                    print("Exiting program...")
                    break
                elif result == "manual":
                    last_input_time = time.time()

                # Switch to autonomous mode if no input after timeout
                if time.time() - last_input_time > TIMEOUT:
                    mode = "autonomous"

    except KeyboardInterrupt:
        print("Terminating robot process.")
    finally:
        lidar_processor.stop()
        robot.stop()
        sock.close()
        print("All systems shut down.")


if __name__ == "__main__":
    main_robot_loop()
