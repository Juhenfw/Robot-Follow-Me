import socket
import math
import ddsm115 as motor

# Konfigurasi UDP (Menerima dari Raspberry Pi 2)
UDP_IP = "0.0.0.0"  # Dengarkan semua IP
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Menerima data posisi T0 di port {UDP_PORT}")

# Konfigurasi Motor
r_wheel = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B0049TUZ-if00-port0"
l_wheel = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B0045S9B-if00-port0"

motor_kanan = motor.MotorControl(device=r_wheel)
motor_kanan.set_drive_mode(1, 2)

motor_kiri = motor.MotorControl(device=l_wheel)
motor_kiri.set_drive_mode(1, 2)

# Fungsi Hitung Sudut & Jarak
def hitung_jarak_sudut(x, y):
    jarak = math.sqrt(x**2 + y**2)
    sudut = math.degrees(math.atan2(y, x))
    return jarak, sudut

# Loop utama
while True:
    data, addr = sock.recvfrom(1024)  # Terima data dari Raspberry Pi 2
    x_t0, y_t0, z_t0 = map(float, data.decode().split(","))
    
    print(f"Posisi T0 diterima: X={x_t0}, Y={y_t0}, Z={z_t0}")

    # Hitung jarak & sudut dari robot ke T0
    jarak, sudut = hitung_jarak_sudut(x_t0, y_t0)

    # Logika Gerakan Robot
    speed = 100
    stop = 0

    if jarak > 70:  # Jarak cukup jauh, robot maju
        if sudut > 15:  # Belok kanan
            motor_kanan.send_rpm(1, -speed // 2)
            motor_kiri.send_rpm(1, speed)
        elif sudut < -15:  # Belok kiri
            motor_kanan.send_rpm(1, -speed)
            motor_kiri.send_rpm(1, speed // 2)
        else:  # Maju lurus
            motor_kanan.send_rpm(1, -speed)
            motor_kiri.send_rpm(1, speed)
    else:  # Jika T0 terlalu dekat, berhenti
        motor_kanan.send_rpm(1, stop)
        motor_kiri.send_rpm(1, stop)
        print("Berhenti: T0 terlalu dekat!")

    print(f"Jarak: {jarak:.2f} cm | Sudut: {sudut:.2f}°")
