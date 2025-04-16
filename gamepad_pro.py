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

    def run(self):
        while self.running:
            self.screen.fill((0, 0, 0))
            pygame.display.flip()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False

            pygame.event.pump()
            joystick = self.joystick

            axis_lr = float(joystick.get_axis(0))
            print(axis_lr)
            # axis_lr_analog = joystick.get_axis(4)
            axis_fb = joystick.get_axis(1)
            button_y = joystick.get_button(3)
            button_a = joystick.get_button(1)
            button_x = joystick.get_button(0)
            button_b = joystick.get_button(2)
            button_rb = joystick.get_button(5)
            button_lb = joystick.get_button(4)
            button_back = joystick.get_button(8)

            deadzone = 0.00390625
            self.current_speed_rwheel = self.stop
            self.current_speed_lwheel = self.stop

            # ======== Kombinasi Tombol untuk Belok & Drift ========
            if button_a and (float(axis_lr) < deadzone):  # Maju + Belok Kiri
                self.current_speed_rwheel = -self.speed
                self.current_speed_lwheel = self.serong
            elif button_a and (float(axis_lr) > deadzone): # Maju + Belok Kanan
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
            elif (float(axis_lr) < deadzone):  # Rotasi kiri
                self.current_speed_rwheel = -self.lambat
                self.current_speed_lwheel = -self.lambat
            elif (float(axis_lr) > deadzone):  # Rotasi kanan
                self.current_speed_rwheel = self.lambat
                self.current_speed_lwheel = self.lambat
            elif button_back:  # Keluar
                self.running = False
            else:
                self.current_speed_rwheel = self.stop
                self.current_speed_lwheel = self.stop

            # if button_a:  # MAJU
            #     if abs(axis_lr) > deadzone:
            #         turn = axis_lr * self.serong
            #         self.current_speed_rwheel = int(-self.speed + turn)
            #         self.current_speed_lwheel = int(self.speed + turn)
            #     else:
            #         self.current_speed_rwheel = -self.speed
            #         self.current_speed_lwheel = self.speed

            
            # if button_a and (abs(axis_lr) > deadzone): # MAJU KANAN
            #     speed_factor = -axis_lr
            #     self.current_speed_rwheel = int(-speed_factor * self.serong)
            #     self.current_speed_lwheel = int(speed_factor * self.speed)

            # if button_a and (abs(axis_lr) > deadzone): # MAJU KIRI
            #     speed_factor = -axis_lr
            #     self.current_speed_rwheel = int(-speed_factor * self.speed)
            #     self.current_speed_lwheel = int(speed_factor * self.serong)

            # if button_x:  # MUNDUR
            #     self.current_speed_rwheel = self.speed
            #     self.current_speed_lwheel = -self.speed
            
            if button_rb:  # Tambah Speed
                self.speed += 10

            if button_lb:  # Kurang Speed
                self.speed -= 10
            
            # if abs(axis_lr_analog) > deadzone:
            #     speed_factor = -axis_lr_analog
            #     self.current_speed_rwheel = int(-speed_factor * self.speed)
            #     self.current_speed_lwheel = int(-speed_factor * self.speed)

            # if button_x:  # KIRI
            #     self.current_speed_rwheel = -self.speed
            #     self.current_speed_lwheel = -self.speed

            # if button_b:  # KANAN
            #     self.current_speed_rwheel = self.speed
            #     self.current_speed_lwheel = self.speed

            # if button_back:
            #     self.running = False

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
