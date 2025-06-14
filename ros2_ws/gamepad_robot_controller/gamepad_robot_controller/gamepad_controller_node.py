import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import pygame
import time
import os
import threading
import signal
import sys


# Import the motor control library
try:
    # from gamepad_robot_controller import ddsm115 as motor
    from gamepad_robot_controller.ddsm115 import MotorControl
except ImportError:
    print("Warning: ddsm115 module not found. Running in simulation mode.")
    motor = None

class GamepadRobotController(Node):
    def __init__(self):
        super().__init__('gamepad_robot_controller')

        # Log environment for debugging
        display = os.environ.get('DISPLAY', 'Not set')
        self.get_logger().info(f"DISPLAY environment variable: {display}")

        # Define parameters for motor ports
        self.declare_parameter('r_wheel_port', '/dev/ttyRS485-1')
        self.declare_parameter('l_wheel_port', '/dev/ttyRS485-2')
        self.declare_parameter('update_rate', 30.0)  # Hz

        self.r_wheel_port = self.get_parameter('r_wheel_port').value
        self.l_wheel_port = self.get_parameter('l_wheel_port').value
        self.update_rate = self.get_parameter('update_rate').value

        # Publishers
        self.right_wheel_pub = self.create_publisher(Float32, 'right_wheel_speed', 10)
        self.left_wheel_pub = self.create_publisher(Float32, 'left_wheel_speed', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initialize pygame in a safer way
        try:
            # Try to use x11 driver first
            os.environ['SDL_VIDEODRIVER'] = 'x11'
            pygame.init()
            self.pygame_initialized = True

            # Try to create a minimal window
            try:
                self.screen = pygame.display.set_mode((300, 200), pygame.NOFRAME)
            except pygame.error:
                # Fall back to a headless mode if window creation fails
                self.get_logger().warn("Couldn't create pygame window, falling back to headless mode")
                os.environ['SDL_VIDEODRIVER'] = 'dummy'
                pygame.display.quit()
                pygame.display.init()
                self.screen = pygame.display.set_mode((1, 1))

            pygame.display.set_caption("ROS 2 Robot Controller")
            self.clock = pygame.time.Clock()

        except pygame.error as e:
            self.get_logger().error(f"Failed to initialize pygame: {e}")
            self.pygame_initialized = False
            self.running = False
            return

        # Speed settings
        self.speed = 100
        self.lambat = self.speed / 4
        self.serong = self.speed / 3
        self.drift_boost = self.speed * 1.3
        self.stop = 0

        self.running = True
        self.current_speed_rwheel = self.stop
        self.current_speed_lwheel = self.stop

        # Direct motor control (optional)
        self.motor1 = None
        self.motor2 = None
        self.connect_motors()

        # Joystick initialization
        self.joystick = None
        try:
            pygame.joystick.init()
            joystick_count = pygame.joystick.get_count()
            self.get_logger().info(f"Joystick count: {joystick_count}")

            if joystick_count > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.get_logger().info("Joystick detected and initialized")
            else:
                self.get_logger().error("Joystick not detected. Please connect and restart.")
                self.running = False
        except pygame.error as e:
            self.get_logger().error(f"Joystick initialization error: {e}")
            self.running = False

        # Create a timer for regular updates
        self.timer = self.create_timer(1.0/self.update_rate, self.update_callback)

        # Thread for pygame event processing
        self.pygame_thread = None
        if self.pygame_initialized and self.running:
            self.pygame_thread = threading.Thread(target=self.process_pygame_events)
            self.pygame_thread.daemon = True
            self.pygame_thread.start()

        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        self.get_logger().info("ROS 2 Gamepad Robot Controller initialized")

    def signal_handler(self, sig, frame):
        self.get_logger().info(f"Received signal {sig}, shutting down...")
        self.running = False

    def verify_port_exists(self, port):
        return os.path.exists(port)

    def connect_motors(self):
        if MotorControl is None:
            self.get_logger().warn("Running in simulation mode - no direct motor control")
            return

        if self.verify_port_exists(self.r_wheel_port):
            try:
                self.motor1 = MotorControl(device=self.r_wheel_port)
                self.motor1.set_drive_mode(1, 2)
                self.get_logger().info(f"Right motor connected to {self.r_wheel_port}")
            except Exception as e:
                self.get_logger().error(f"Right motor error: {e}")
                self.motor1 = None

        if self.verify_port_exists(self.l_wheel_port):
            try:
                self.motor2 = MotorControl(device=self.l_wheel_port)
                self.motor2.set_drive_mode(1, 2)
                self.get_logger().info(f"Left motor connected to {self.l_wheel_port}")
            except Exception as e:
                self.get_logger().error(f"Left motor error: {e}")
                self.motor2 = None

    def safe_send_rpm(self, motor_obj, motor_id, speed):
        if not motor_obj:
            return False
        try:
            motor_obj.send_rpm(motor_id, speed)
            return True
        except Exception as e:
            self.get_logger().error(f"Error sending RPM: {e}")
            return False

    def process_pygame_events(self):
        while self.running and self.pygame_initialized:
            try:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self.running = False
                time.sleep(0.01)
            except pygame.error:
                break

    def publish_wheel_speeds(self):
        if not self.running:
            return

        # Publish motor speeds to ROS topics
        right_msg = Float32()
        right_msg.data = float(self.current_speed_rwheel)
        self.right_wheel_pub.publish(right_msg)

        left_msg = Float32()
        left_msg.data = float(self.current_speed_lwheel)
        self.left_wheel_pub.publish(left_msg)

        # Also publish as cmd_vel for compatibility
        twist_msg = Twist()
        avg_speed = (abs(self.current_speed_rwheel) + abs(self.current_speed_lwheel)) / 2.0
        linear_x = 0.0
        angular_z = 0.0

        if self.current_speed_rwheel < 0 and self.current_speed_lwheel > 0:
            # Forward
            linear_x = avg_speed / 100.0  # Scale to m/s
        elif self.current_speed_rwheel > 0 and self.current_speed_lwheel < 0:
            # Backward
            linear_x = -avg_speed / 100.0  # Scale to m/s
        if (self.current_speed_rwheel < 0 and self.current_speed_lwheel < 0) or \
           (self.current_speed_rwheel > 0 and self.current_speed_lwheel > 0):
            # Rotation
            angular_z = (self.current_speed_rwheel) / 100.0  # Scale to rad/s
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.cmd_vel_pub.publish(twist_msg)

    def update_callback(self):
        if not self.running or not self.joystick or not self.pygame_initialized:
            return

        try:
            # Update display
            try:
                self.screen.fill((0, 0, 0))
                pygame.display.flip()
            except pygame.error:
                pass

            # Get joystick input
            try:
                axis_lr = float(self.joystick.get_axis(0))
                button_a = self.joystick.get_button(1)
                button_x = self.joystick.get_button(0)
                button_rb = self.joystick.get_button(5)
                button_lb = self.joystick.get_button(4)
                button_back = self.joystick.get_button(8)
            except pygame.error:
                self.get_logger().error("Failed to get joystick input")
                return

            deadzone = 0.00390625
            self.current_speed_rwheel = self.stop
            self.current_speed_lwheel = self.stop

            # ======== Kombinasi Tombol untuk Belok & Drift ========
            if button_a and (float(axis_lr) < deadzone):  # Maju + Belok Kiri
                self.current_speed_rwheel = -self.speed
                self.current_speed_lwheel = self.serong
            elif button_a and (float(axis_lr) > deadzone):  # Maju + Belok Kanan
                self.current_speed_rwheel = -self.serong
                self.current_speed_lwheel = self.speed
            elif button_x and (float(axis_lr) < deadzone):  # Mundur + Belok Kiri
                self.current_speed_rwheel = self.speed
                self.current_speed_lwheel = -self.serong
            elif button_x and (float(axis_lr) > deadzone):  # Mundur + Belok Kanan
                self.current_speed_rwheel = self.serong
                self.current_speed_lwheel = -self.speed

            # ======== Kontrol Gerakan Normal ========
            elif button_a:  # Maju
                self.current_speed_rwheel = -self.speed
                self.current_speed_lwheel = self.speed
            elif button_x:  # Mundur
                self.current_speed_rwheel = self.speed
                self.current_speed_lwheel = -self.speed
            elif (float(axis_lr) < (deadzone-0.001)):  # Rotasi kiri
                self.current_speed_rwheel = -self.lambat
                self.current_speed_lwheel = -self.lambat
            elif (float(axis_lr) > (deadzone+0.001)):  # Rotasi kanan
                self.current_speed_rwheel = self.lambat
                self.current_speed_lwheel = self.lambat
            elif button_back:  # Keluar
                self.running = False
            else:
                self.current_speed_rwheel = self.stop
                self.current_speed_lwheel = self.stop

            if button_rb:  # Tambah Speed
                self.speed += 10
                self.get_logger().info(f"Increasing speed to {self.speed}")

            if button_lb:  # Kurang Speed
                self.speed -= 10
                self.get_logger().info(f"Decreasing speed to {self.speed}")

            # Publish to ROS 2 topics
            self.publish_wheel_speeds()

            # Also send direct commands if motors are connected
            self.safe_send_rpm(self.motor1, 1, self.current_speed_rwheel)
            self.safe_send_rpm(self.motor2, 1, self.current_speed_lwheel)

            self.clock.tick(30)

        except Exception as e:
            self.get_logger().error(f"Update error: {e}")

    def cleanup(self):
        self.get_logger().info("Cleaning up and stopping motors...")

        if self.pygame_initialized:
            try:
                self.running = False
                if self.pygame_thread and self.pygame_thread.is_alive():
                    self.pygame_thread.join(timeout=1.0)

                if self.motor1:
                    try:
                        self.motor1.send_rpm(1, 0)
                        self.motor1.close()
                    except Exception as e:
                        self.get_logger().error(f"Error closing motor1: {e}")

                if self.motor2:
                    try:
                        self.motor2.send_rpm(1, 0)
                        self.motor2.close()
                    except Exception as e:
                        self.get_logger().error(f"Error closing motor2: {e}")

                pygame.quit()

            except Exception as e:
                self.get_logger().error(f"Error during cleanup: {e}")

def main(args=None):
    rclpy.init(args=args)

    try:
        controller = GamepadRobotController()

        if controller.running:
            try:
                rclpy.spin(controller)
            except KeyboardInterrupt:
                pass
            finally:
                controller.cleanup()

        controller.destroy_node()

    except Exception as e:
        if rclpy.ok():
            print(f"Error in main: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()