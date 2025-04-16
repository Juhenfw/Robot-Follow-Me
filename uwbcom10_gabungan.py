import socket
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
        self.bias = {'A0': 50.0, 'A1': 50.0, 'A2': 50.0}  # Bias values in cm
        self.scale_factor = {'A0': 1.03, 'A1': 1.05, 'A2': 1.05}  # Scale factors
    
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
        """Move the robot"""
        self.right_motor.send_rpm(1, right_speed)
        self.left_motor.send_rpm(1, left_speed)

    def stop(self):
        """Stop the robot movement"""
        self.move(0, 0)
        time.sleep(0.2)

    def analyze_and_act(self, distances):
        """Analyze UWB distances and act accordingly"""
        A0, A1, A2 = distances['A0'], distances['A1'], distances['A2']
        print("UWB Corrected Distances (cm):")
        print(f"A0: {A0:.2f} | A1: {A1:.2f} | A2: {A2:.2f}")

        # Move based on distances
        if A0 < A1 and A0 < A2:
            self.move(-speed, speed)
        elif A1 > A2 and A1 > A0:
            self.move(0, speed / rotasi)
        elif A2 > A1 and A2 > A0:
            self.move(-speed / rotasi, 0)

        # Stop when A0 reaches threshold
        if A0 <= batas:
            self.stop()


def visualize_lidar_pygame(lidar_processor):
    """Visualize LIDAR data using Pygame"""
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

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                break

        clock.tick(30)

    pygame.quit()


class GamepadController:
    """Handles gamepad inputs for manual control"""

    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((400, 300))
        pygame.display.set_caption("Kontrol Robot - Gamepad Mode")
        self.clock = pygame.time.Clock()
        self.speed = 100
        self.stop = 0
        self.current_speed_rwheel = self.stop
        self.current_speed_lwheel = self.stop
        self.running = True

        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            print("Joystick terdeteksi dan diinisialisasi")
        else:
            print("Joystick tidak terdeteksi. Harap colokkan dan restart.")
            self.running = False

    def run(self, robot, lidar_processor, uwb_tracker):
        """Run gamepad control loop"""
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

            pygame.event.pump()
            joystick = self.joystick

            # Handle button presses
            button_back = joystick.get_button(8)  # Back button to switch to autonomous mode

            if button_back:
                print("Switching to Autonomous Mode")
                return "autonomous"

            # Get the axis values for movement and rotation
            axis_lr = joystick.get_axis(0)  # Left-Right Axis for turning
            axis_fb = joystick.get_axis(1)  # Forward-Backward Axis for moving

            # Speed adjustment based on axis
            self.current_speed_rwheel = int(axis_fb * self.speed)
            self.current_speed_lwheel = int(-axis_fb * self.speed)

            # If axis_lr is moved, control the turning speed
            if abs(axis_lr) > 0.2:
                # Rotate the robot based on the axis_lr
                self.current_speed_rwheel = int(self.current_speed_rwheel * (1 - axis_lr))
                self.current_speed_lwheel = int(self.current_speed_lwheel * (1 + axis_lr))

            # Apply the movement command
            robot.move(self.current_speed_rwheel, self.current_speed_lwheel)

            self.clock.tick(30)
            time.sleep(0.01)

    def cleanup(self):
        """Clean up gamepad resources"""
        pygame.quit()


def main_robot_loop():
    """Main loop for robot operations"""
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

    mode = "gamepad"  # Start in gamepad mode
    try:
        print("Starting robot...")
        while True:
            if mode == "gamepad":
                mode = gamepad_controller.run(robot, lidar_processor, uwb_tracker)
            elif mode == "autonomous":
                data, addr = sock.recvfrom(1024)
                parts = data.decode().split(",")
                raw_uwb_distances['A0'] = float(parts[0])
                raw_uwb_distances['A1'] = float(parts[1])
                raw_uwb_distances['A2'] = float(parts[2])

                corrected_distances = uwb_tracker.apply_bias_correction(raw_uwb_distances)
                robot.analyze_and_act(corrected_distances)
                time.sleep(0.00001)

    except KeyboardInterrupt:
        print("Terminating robot process.")
    finally:
        lidar_processor.stop()
        robot.stop()
        sock.close()
        pygame.quit()
        print("All systems shut down.")


if __name__ == "__main__":
    main_robot_loop()
