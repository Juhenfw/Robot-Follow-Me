import pygame
import time
import ddsm115 as motor
import os

class RobotController:
    def __init__(self):
        self.r_wheel_port = "/dev/robot_rwheel"
        self.l_wheel_port = "/dev/robot_lwheel"

        # Inisialisasi pygame
        pygame.init()
        self.screen = pygame.display.set_mode((400, 300))
        pygame.display.set_caption("Kontrol Robot DDSM115 - Gamepad Mode")
        self.clock = pygame.time.Clock()

        # Kecepatan
        self.base_speed = 100
        self.speed = self.base_speed
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

    def run(self):
        while self.running:
            self.screen.fill((0, 0, 0))
            pygame.display.flip()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

            pygame.event.pump()
            joystick = self.joystick

            axis_lr = joystick.get_axis(0)  # Sumbu kiri-kanan
            axis_fb = joystick.get_axis(1)  # Sumbu depan-belakang
            button_y = joystick.get_button(3)  # Tombol Y - Tambah Kecepatan
            button_a = joystick.get_button(1)  # Tombol A - Maju
            button_x = joystick.get_button(0)  # Tombol X - Mundur
            button_b = joystick.get_button(2)  # Tombol B - Putar Kiri
            button_rb = joystick.get_button(5)  # Tombol RB - Putar Kanan
            button_lb = joystick.get_button(4)  # Tombol LB - Tambah Speed
            button_back = joystick.get_button(8)  # Tombol Back - Exit

            deadzone = 0.2
            self.current_speed_rwheel = self.stop
            self.current_speed_lwheel = self.stop

            # Kecepatan maju atau mundur
            if abs(axis_fb) > deadzone:  # Maju/Mundur
                speed_factor = -axis_fb
                self.current_speed_rwheel = int(speed_factor * self.speed)
                self.current_speed_lwheel = int(speed_factor * self.speed)

            # Rotasi berdasarkan kiri-kanan
            if abs(axis_lr) > deadzone:  # Putar Kiri/Kanan
                rot_speed = -axis_lr
                self.current_speed_rwheel = int(rot_speed * self.serong)
                self.current_speed_lwheel = int(rot_speed * -self.serong)

            # Tombol Y - Tambah Kecepatan
            if button_y:
                self.speed = min(self.base_speed * 2, self.speed + 10)  # Batas atas kecepatan

            # Tombol X - Mundur
            if button_x:
                self.current_speed_rwheel = self.speed
                self.current_speed_lwheel = -self.speed

            # Tombol B - Putar Kiri
            if button_b:
                self.current_speed_rwheel = -self.speed
                self.current_speed_lwheel = self.speed

            # Tombol RB - Putar Kanan
            if button_rb:
                self.current_speed_rwheel = self.speed
                self.current_speed_lwheel = -self.speed

            # Tombol LB - Kurang Kecepatan
            if button_lb:
                self.speed = max(self.base_speed / 2, self.speed - 10)  # Batas bawah kecepatan

            # Tombol Back untuk keluar
            if button_back:
                self.running = False

            # Kirim kecepatan ke motor
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

if __name__ == "__main__":
    robot = RobotController()
    try:
        robot.run()
    finally:
        robot.cleanup()
