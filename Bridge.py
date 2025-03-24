import socket
import time
import json
import math

# UDP Configuration
UDP_IP = "0.0.0.0"  # Terima dari semua interface
UDP_PORT = 5005
BUFFER_SIZE = 1024

# Socket timeout (seconds)
SOCKET_TIMEOUT = 1.0

# Batas waktu untuk data yang dianggap valid
MAX_DATA_AGE = 2.0  # seconds

# Inisialisasi buffer untuk filter data
buffer_size = 5
x_buffer = []
y_buffer = []
distance_buffer = []
angle_buffer = []

def filter_data(new_value, buffer):
    """
    Fungsi filter sederhana menggunakan rata-rata bergerak
    """
    buffer.append(new_value)
    if len(buffer) > buffer_size:
        buffer.pop(0)
    return sum(buffer) / len(buffer)

# Fungsi untuk mengontrol motor robot (contoh, sesuaikan dengan hardware Anda)
def control_motors(distance, angle):
    """
    Kontrol motor berdasarkan jarak dan sudut ke target
    
    Parameters:
    - distance: jarak ke target (cm)
    - angle: sudut ke target (-180 to 180 degrees)
                0 = target di depan robot
                +90 = target di kanan robot
                -90 = target di kiri robot
    """
    # Kecepatan dasar dan parameter kontrol
    BASE_SPEED = 50  # Kecepatan dasar (0-100)
    MAX_SPEED = 100  # Kecepatan maksimum
    MIN_SPEED = 20   # Kecepatan minimum
    
    # Konstanta proporsional untuk kontrol PID sederhana
    KP_DISTANCE = 0.5  # Pengali untuk jarak
    KP_ANGLE = 1.5     # Pengali untuk sudut
    
    # Tentukan kecepatan motor berdasarkan jarak dan sudut
    # Semakin dekat dengan target, semakin lambat robot bergerak
    speed = min(BASE_SPEED, BASE_SPEED * distance / 100) if distance < 100 else BASE_SPEED
    
    # Hitung perbedaan kecepatan untuk berbelok berdasarkan sudut
    # Sudut positif (target di kanan) = belok kanan
    # Sudut negatif (target di kiri) = belok kiri
    turn_rate = KP_ANGLE * angle
    
    # Hitung kecepatan untuk motor kiri dan kanan
    left_speed = speed - turn_rate
    right_speed = speed + turn_rate
    
    # Normalisasi kecepatan ke rentang yang valid
    max_current_speed = max(abs(left_speed), abs(right_speed))
    if max_current_speed > MAX_SPEED:
        scaling_factor = MAX_SPEED / max_current_speed
        left_speed *= scaling_factor
        right_speed *= scaling_factor
    
    # Pastikan kecepatan minimum tercapai jika robot bergerak
    if left_speed > 0 and left_speed < MIN_SPEED:
        left_speed = MIN_SPEED
    elif left_speed < 0 and left_speed > -MIN_SPEED:
        left_speed = -MIN_SPEED
        
    if right_speed > 0 and right_speed < MIN_SPEED:
        right_speed = MIN_SPEED
    elif right_speed < 0 and right_speed > -MIN_SPEED:
        right_speed = -MIN_SPEED
    
    # Berhenti jika sangat dekat dengan target
    if distance < 10:  # 10cm
        left_speed = 0
        right_speed = 0
        print("Target tercapai! Menghentikan motor.")
    
    print(f"Kontrol motor: Kiri={left_speed:.1f}, Kanan={right_speed:.1f}")
    
    # Implementasi kontrol motor sesuai dengan hardware Anda
    # Contoh:
    # set_motor_speed(LEFT_MOTOR, left_speed)
    # set_motor_speed(RIGHT_MOTOR, right_speed)

# Buat socket UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(SOCKET_TIMEOUT)

print(f"UWB Data Receiver dimulai pada port {UDP_PORT}...")
last_data_time = 0

try:
    while True:
        try:
            # Terima data dengan timeout
            data, addr = sock.recvfrom(BUFFER_SIZE)
            print(f"Menerima data dari {addr}")
            
            # Parse data yang diterima
            try:
                data_str = data.decode('utf-8')
                parts = data_str.split(",")
                
                # Buat dictionary dari data
                data_dict = {}
                for part in parts:
                    key, value = part.split("=")
                    data_dict[key] = float(value)
                
                # Ekstrak data
                current_time = time.time()
                
                # Terapkan filter pada data posisi
                x = data_dict.get("X", 0)
                y = data_dict.get("Y", 0)
                distance = data_dict.get("DIST", 0)
                angle = data_dict.get("ANGLE", 0)
                
                x_filtered = filter_data(x, x_buffer)
                y_filtered = filter_data(y, y_buffer)
                distance_filtered = filter_data(distance, distance_buffer)
                angle_filtered = filter_data(angle, angle_buffer)
                
                # Tampilkan informasi
                print(f"Posisi Target: X={x_filtered:.2f}cm, Y={y_filtered:.2f}cm")
                print(f"Jarak ke Target: {distance_filtered:.2f}cm, Sudut: {angle_filtered:.2f}°")
                print(f"Jarak Anchor: A0={data_dict.get('A0', 0):.2f}cm, A1={data_dict.get('A1', 0):.2f}cm, A2={data_dict.get('A2', 0):.2f}cm")
                
                # Kontrol motor berdasarkan jarak dan sudut yang sudah difilter
                control_motors(distance_filtered, angle_filtered)
                last_data_time = current_time
                
            except ValueError as e:
                print(f"Error parsing data: {e}")
                
        except socket.timeout:
            # Periksa apakah kita kehilangan koneksi
            current_time = time.time()
            if last_data_time > 0 and (current_time - last_data_time) > MAX_DATA_AGE:
                print("Tidak ada data posisi baru yang diterima. Menghentikan motor.")
                # Implementasi stop motor di sini
                
        except Exception as e:
            print(f"Unexpected error: {e}")
            import traceback
            traceback.print_exc()
            time.sleep(1)
            
except KeyboardInterrupt:
    print("\nProgram dihentikan oleh pengguna")
except Exception as e:
    print(f"Fatal error: {e}")
    import traceback
    traceback.print_exc()
finally:
    # Hentikan motor jika ada
    print("Menutup socket")
    sock.close()
    print("Receiver script dihentikan")
