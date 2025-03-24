import socket
import math
import ddsm115 as motor
from collections import deque

# Konfigurasi UDP (Menerima dari Raspberry Pi 2)
UDP_IP = "0.0.0.0"
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

# Posisi Anchor dalam cm (Relatif terhadap robot)
A1_POSITION = (-20, 0)  # Kiri belakang
A2_POSITION = (20, 0)   # Kanan belakang
A3_POSITION = (0, -30)  # Tengah belakang

# Koreksi bias dari hasil kalibrasi manual
bias_A1 = 15
bias_A2 = 10
bias_A3 = 8

def correct_bias(distance, bias):
    return max(0, distance - bias)

# Buffer untuk Moving Average (Filter Noise)
buffer_size = 5
jarak_buffer = deque(maxlen=buffer_size)

def filter_data(jarak_baru):
    jarak_buffer.append(jarak_baru)
    return sum(jarak_buffer) / len(jarak_buffer)

# Fungsi Hitung Jarak & Sudut
def hitung_jarak_sudut(x, y):
    jarak = math.sqrt(x**2 + y**2)
    sudut = math.degrees(math.atan2(y, x))
    return jarak, sudut

# Fungsi Hitung Jarak dari Setiap Anchor ke T0
def hitung_jarak_anchor(x_t0, y_t0):
    jarak_a1 = correct_bias(math.sqrt((x_t0 - A1_POSITION[0])**2 + (y_t0 - A1_POSITION[1])**2), bias_A1)
    jarak_a2 = correct_bias(math.sqrt((x_t0 - A2_POSITION[0])**2 + (y_t0 - A2_POSITION[1])**2), bias_A2)
    jarak_a3 = correct_bias(math.sqrt((x_t0 - A3_POSITION[0])**2 + (y_t0 - A3_POSITION[1])**2), bias_A3)
    return jarak_a1, jarak_a2, jarak_a3

# Loop utama
while True:
    data, addr = sock.recvfrom(1024)  # Terima data dari Raspberry Pi 2
    try:
        timestamp, x_t0, y_t0, z_t0 = map(float, data.decode().split(","))
    except ValueError:
        print("Data tidak valid")
        continue

    # Hitung jarak dari setiap anchor ke T0
    jarak_a1, jarak_a2, jarak_a3 = hitung_jarak_anchor(x_t0, y_t0)

    # Hitung jarak & sudut dari robot ke T0
    jarak, sudut = hitung_jarak_sudut(x_t0, y_t0)
    jarak = filter_data(jarak)  # Gunakan Moving Average

    # Logika Gerakan Robot
    speed = 100
    stop = 0
    status_motor = "Berhenti"

    if jarak > 70:  # Jarak cukup jauh, robot maju
        if sudut > 15:  # Belok kanan
            motor_kanan.send_rpm(1, -speed // 2)
            motor_kiri.send_rpm(1, speed)
            status_motor = "Belok Kanan"
        elif sudut < -15:  # Belok kiri
            motor_kanan.send_rpm(1, -speed)
            motor_kiri.send_rpm(1, speed // 2)
            status_motor = "Belok Kiri"
        else:  # Maju lurus
            motor_kanan.send_rpm(1, -speed)
            motor_kiri.send_rpm(1, speed)
            status_motor = "Maju Lurus"
    else:  # Jika T0 terlalu dekat, berhenti
        motor_kanan.send_rpm(1, stop)
        motor_kiri.send_rpm(1, stop)
        status_motor = "Berhenti (T0 terlalu dekat)"

    # **Menampilkan Data di Terminal**
    print("\n" + "="*50)
    print(f"Posisi T0 diterima: X={x_t0:.2f} cm, Y={y_t0:.2f} cm, Z={z_t0:.2f} cm")
    print("-"*50)
    print(f"Jarak dari Anchor ke T0:")
    print(f"🔹 A1 (Kiri Belakang)  → {jarak_a1:.2f} cm")
    print(f"🔹 A2 (Kanan Belakang) → {jarak_a2:.2f} cm")
    print(f"🔹 A3 (Tengah Belakang) → {jarak_a3:.2f} cm")
    print("-"*50)
    print(f"🔹 Jarak T0 ke Robot  → {jarak:.2f} cm")
    print(f"🔹 Sudut T0 ke Robot  → {sudut:.2f}°")
    print(f"🔹 Status Motor       → {status_motor}")
    print("="*50)
