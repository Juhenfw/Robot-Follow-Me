import pygame
import time
import ddsm115 as motor
import os
import usb
import struct

USB_VENDOR = 0x046d
USB_PRODUCT = 0xc21d
default_state = (0, 20, 0, 0, 0, 0, 123, 251, 128, 0, 128, 0, 128, 0, 0, 0, 0, 0, 0, 0)

class Gamepad(object):

    def __init__(self, serial=None):
        """ Initialize the gamepad.
        Args:
            serial: Serial number of the gamepad. If None, the first gamepad found is used.
        """
        self.is_initialized = False
        d = None
        busses = usb.busses()
        for bus in busses:
            devs = bus.devices
            for dev in devs:
                if dev.idVendor == 0x046d and dev.idProduct == 0xc21d:
                    if serial is not None and usb.util.get_string(dev.dev, dev.iSerialNumber) != serial:
                        continue
                    d = dev
        if not d is None:
            self._dev = d.open()
            try:
                self._dev.detachKernelDriver(0)
            except usb.core.USBError:
                pass
            self._dev.setConfiguration(1)
            self._dev.claimInterface(0)
            self._dev.interruptWrite(0x02, struct.pack('<BBB', 0x01, 0x03, 0x04))
            self._state = default_state
            self._old_state = default_state
            self.is_initialized = True
            print("Gamepad initialized")
        else:
            if serial is not None:
                raise RuntimeError(f"Device with serial number '{serial}' not found")
            raise RuntimeError("Could not initialize Gamepad")

    def _getState(self, timeout):
        try:
            data = self._dev.interruptRead(0x81, 0x20, timeout=timeout)
            data = struct.unpack('<' + 'B' * 20, data)
            return data
        except usb.core.USBError as e:
            return None

    def read_gamepad(self, timeout=200):
        state = self._getState(timeout=timeout)
        self.changed = state is not None
        if self.changed:
            self._old_state = self._state
            self._state = state

    def get_state(self):
        return self._state[:]

    def get_A(self):
        return self._state[3] == 16

    def get_X(self):
        return self._state[3] == 64

    def get_analogL_x(self):
        return self._state[6]

    def get_analogL_y(self):
        return self._state[8]

    def get_analogR_x(self):
        return self._state[11]

    def get_analogR_y(self):
        return self._state[12]

    def __del__(self):
        if self.is_initialized:
            self._dev.releaseInterface()
            self._dev.reset()


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

        # Inisialisasi Gamepad
        self.gamepad = Gamepad()

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

            # Membaca status gamepad
            self.gamepad.read_gamepad()

            if self.gamepad.get_A():  # Tombol A untuk maju
                self.current_speed_rwheel = self.speed
                self.current_speed_lwheel = self.speed

            if self.gamepad.get_X():  # Tombol X untuk mundur
                self.current_speed_rwheel = -self.speed
                self.current_speed_lwheel = -self.speed

            # Kontrol analog untuk belok dan maju/mundur
            axis_lr = self.gamepad.get_analogL_x()  # Joystick kiri - kiri/kanan
            axis_fb = self.gamepad.get_analogL_y()  # Joystick kiri - atas/bawah

            # Mengatur kecepatan dan belok
            if abs(axis_lr) > 0.2 or abs(axis_fb) > 0.2:
                self.current_speed_rwheel = int(self.speed * axis_fb - axis_lr * self.serong)
                self.current_speed_lwheel = int(self.speed * axis_fb + axis_lr * self.serong)

            # Kirim perintah RPM ke motor
            self.safe_send_rpm(self.motor1, 1, self.current_speed_rwheel)
            self.safe_send_rpm(self.motor2, 1, self.current_speed_lwheel)

            self.clock.tick(30)
            time.sleep(0.01)

            print(f"Kecepatan: {self.speed}")

    def cleanup(self):
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
