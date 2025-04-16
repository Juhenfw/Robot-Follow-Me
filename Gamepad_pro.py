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

            # Mengakses axis (Sumbu joystick)
            axis_lr = joystick.get_axis(0)  # Sumbu kiri-kanan (belok) - Joystick kiri
            axis_fb = joystick.get_axis(1)  # Sumbu depan-belakang (maju/mundur) - Joystick kiri
            axis_lr_r = joystick.get_axis(2)  # Sumbu kanan-kanan untuk rotasi
            axis_fb_r = joystick.get_axis(3)  # Sumbu vertikal kanan untuk rotasi

            # Tombol-tombol aksi
            button_a = joystick.get_button(0)  # Tombol A (Maju)
            button_x = joystick.get_button(2)  # Tombol X (Mundur)
            button_rb = joystick.get_button(5)  # Tombol RB (Tambah Kecepatan)
            button_lb = joystick.get_button(4)  # Tombol LB (Kurang Kecepatan)
            button_back = joystick.get_button(6)  # Tombol Back untuk keluar

            deadzone = 0.2
            self.current_speed_rwheel = self.stop
            self.current_speed_lwheel = self.stop

            # Kontrol maju (A) dan mundur (X) dengan tombol
            if button_a:  # MAJU (Gas)
                self.current_speed_rwheel = int(self.speed * axis_fb)
                self.current_speed_lwheel = int(self.speed * axis_fb)

            if button_x:  # MUNDUR
                self.current_speed_rwheel = int(-self.speed * axis_fb)
                self.current_speed_lwheel = int(-self.speed * axis_fb)

            # Sistem belok kompleks dengan joystick analog kiri (kiri-kanan)
            if abs(axis_lr) > deadzone or abs(axis_fb) > deadzone:
                self.current_speed_rwheel = int(self.speed * axis_fb - axis_lr * self.serong)
                self.current_speed_lwheel = int(self.speed * axis_fb + axis_lr * self.serong)

            # Rotasi halus dengan joystick kanan (axis_lr_r dan axis_fb_r)
            if abs(axis_lr_r) > deadzone or abs(axis_fb_r) > deadzone:
                self.current_speed_rwheel += int(self.speed * axis_fb_r - axis_lr_r * self.serong)
                self.current_speed_lwheel += int(self.speed * axis_fb_r + axis_lr_r * self.serong)

            # Tombol RB untuk menambah kecepatan
            if button_rb:
                self.speed = min(self.base_speed * 2, self.speed + 10)

            # Tombol LB untuk mengurangi kecepatan
            if button_lb:
                self.speed = max(self.base_speed / 2, self.speed - 10)

            # Tombol Back untuk keluar dari program
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
