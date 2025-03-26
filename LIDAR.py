import math
import numpy as np
import time
import serial
from rplidar import RPLidar
from threading import Thread

# Konfigurasi UWB
port_uwb = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
baudrate_uwb = 115200
timeout_uwb = 1

# Konfigurasi LiDAR
PORT_LIDAR = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_f235ae4105e1d247940e6441b646a0b3-if00-port0"
lidar = RPLidar(PORT_LIDAR, baudrate=256000)

# Robot motion functions
def move_robot(command):
    """
    Fungsi untuk menggerakkan robot sesuai perintah.
    command: 'move' untuk bergerak, 'stop' untuk berhenti
    """
    if command == "move":
        print("Robot is moving.")
        # Implementasikan kontrol motor atau roda untuk pergerakan robot
        # Misalnya: motor.move_forward()
    elif command == "stop":
        print("Robot is stopped.")
        # Implementasikan kontrol motor atau roda untuk menghentikan robot
        # Misalnya: motor.stop()

# Koreksi Bias UWB
def correct_bias(raw_distance, bias):
    """
    Fungsi untuk mengoreksi bias pada pembacaan sensor UWB.
    """
    return raw_distance - bias

# Fungsi untuk menghitung rotasi yang diperlukan berdasarkan sudut target dan sudut saat ini
def calculate_rotation(target_angle, current_angle):
    """
    Fungsi untuk menghitung rotasi yang dibutuhkan agar robot menghadap orang.
    """
    angle_diff = target_angle - current_angle
    if angle_diff > 180:
        angle_diff -= 360  # Putar ke arah pendek
    elif angle_diff < -180:
        angle_diff += 360  # Putar ke arah pendek
    return angle_diff

# Fungsi untuk memutar robot
def rotate_robot(rotation_angle):
    """
    Fungsi untuk memutar robot dengan sudut tertentu.
    """
    print(f"Rotating robot by {rotation_angle:.2f} degrees")
    # Implementasikan kode untuk menggerakkan motor servo atau roda robot sesuai perintah rotasi
    # Misalnya: servo_motor.rotate(rotation_angle)

# Fungsi untuk melacak posisi orang berdasarkan data LiDAR
def track_person_with_lidar(lidar):
    """
    Fungsi untuk melacak posisi orang yang memegang sensor UWB menggunakan data LiDAR.
    """
    angles = []
    distances = []

    for scan in lidar.iter_scans():
        angles.clear()
        distances.clear()

        for _, angle, distance in scan:
            if distance <= 2000:  # Batas jarak (misalnya 2 meter)
                angles.append(np.deg2rad(angle))  # Ubah sudut menjadi radian
                distances.append(distance)
        
        if distances:
            # Temukan orang yang paling dekat (dengan jarak terpendek)
            closest_person_distance = min(distances)
            closest_person_angle = angles[distances.index(closest_person_distance)]
            
            return closest_person_angle, closest_person_distance

    return None, None

# Fungsi untuk membaca data dari sensor UWB
def read_uwb_data():
    """
    Fungsi untuk membaca data dari sensor UWB.
    """
    try:
        ser = serial.Serial(port_uwb, baudrate_uwb, timeout=timeout_uwb)
        print(f"Connected to {port_uwb} at {baudrate_uwb} baud.")
        
        while True:
            if ser.in_waiting > 0:
                data = ser.readline().decode("utf-8").strip()
                if data.startswith("$KT0"):
                    parts = data.split(",")
                    if len(parts) >= 4:
                        raw_values = parts[1:4]
                        processed_values = []
                        for value in raw_values:
                            if value.lower() == "null":
                                processed_values.append(0.0)
                            else:
                                processed_values.append(float(value))
                        A0, A1, A2 = processed_values

                        # Koreksi bias
                        cal_A0 = correct_bias(A0*100, 15)  # Misalnya bias A0 adalah 15 cm
                        print(f"UWB Distance (A0): {cal_A0:.2f} cm")

                        # Tentukan perintah bergerak atau berhenti berdasarkan nilai cal_A0
                        if cal_A0 >= 50:
                            move_robot("move")
                        else:
                            move_robot("stop")
                    
    except serial.SerialException as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("Exiting program.")
    finally:
        if "ser" in locals() and ser.is_open:
            ser.close()
            print("Serial connection closed.")

# Fungsi utama untuk kontrol robot menggunakan UWB dan LiDAR
def control_lidar_and_robot():
    """
    Fungsi utama untuk mengontrol robot berdasarkan data dari UWB dan LiDAR.
    """
    global running, thread  # Ensure the variable 'running' is declared global
    if not running:
        running = True
        thread = Thread(target=update_lidar, daemon=True)
        thread.start()

    try:
        while True:
            # Dapatkan posisi orang yang memegang sensor UWB menggunakan LiDAR
            target_angle, target_distance = track_person_with_lidar(lidar)
            
            if target_angle is not None:
                # Tentukan posisi orang yang memegang UWB ditemukan di sudut tertentu
                print(f"Person detected at angle: {np.degrees(target_angle):.2f} degrees, distance: {target_distance:.2f} meters")

                # Tentukan posisi robot saat ini (misalnya robot menghadap 0 derajat)
                current_angle = 0  # Asumsi robot menghadap 0 derajat

                # Hitung rotasi yang diperlukan untuk menghadap orang
                rotation_angle = calculate_rotation(np.degrees(target_angle), current_angle)
                rotate_robot(rotation_angle)
            
            time.sleep(0.5)  # Tunggu sebentar untuk pembaruan data LiDAR
    except KeyboardInterrupt:
        print("Stopping robot control.")
        running = False
        lidar.stop()

if __name__ == "__main__":
    # Deklarasi global untuk 'running' dan 'thread'
    global running
    running = False  # Pastikan 'running' didefinisikan
    thread = None  # Inisialisasi thread jika belum ada

    # Mulai membaca data UWB di thread terpisah
    uwb_thread = Thread(target=read_uwb_data, daemon=True)
    uwb_thread.start()

    # Mulai kontrol LiDAR di thread terpisah
    control_lidar_and_robot()
