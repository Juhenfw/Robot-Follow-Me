# SENDER
import serial
import math
import socket

# UDP Configuration (jika akan mengirim ke Raspberry Pi lain)
UDP_IP = "192.168.1.100"  # Ganti dengan alamat IP Raspberry Pi penerima
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

port = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
baudrate = 115200
timeout = 1

# Koordinat anchor pada robot (dalam centimeter)
# Mengasumsikan titik pusat robot sebagai (0,0) dan orientasi maju ke arah sumbu-Y positif
anchors = {
    "A0": (0, 20),        # Anchor tengah depan
    "A1": (-30, -15),     # Anchor kiri belakang
    "A2": (30, -15)       # Anchor kanan belakang
}

def calculate_target_position(distances):
    """
    Menghitung posisi target menggunakan trilaterasi
    """
    # Implementasi algoritma trilaterasi
    # Ini adalah pendekatan sederhana yang bisa ditingkatkan dengan algoritma yang lebih canggih
    # seperti Kalman Filter seperti disebutkan dalam penelitian <sup data-citation="1" className="inline select-none [&>a]:rounded-2xl [&>a]:border [&>a]:px-1.5 [&>a]:py-0.5 [&>a]:transition-colors shadow [&>a]:bg-ds-bg-subtle [&>a]:text-xs [&>svg]:w-4 [&>svg]:h-4 relative -top-[2px] citation-shimmer"><a href="https://www.sciencedirect.com/science/article/pii/S2405896324004932">1</a></sup>
    
    A0_pos = anchors["A0"]
    A1_pos = anchors["A1"]
    A2_pos = anchors["A2"]
    
    # Persamaan lingkaran untuk ketiga anchor
    # (x - x_a0)² + (y - y_a0)² = d_a0²
    # (x - x_a1)² + (y - y_a1)² = d_a1²
    # (x - x_a2)² + (y - y_a2)² = d_a2²
    
    # Solusi dengan metode least squares atau pendekatan numerik lainnya
    # Implementasi sederhana menggunakan weighted centroid algorithm
    weights = []
    x_target, y_target = 0, 0
    
    for anchor_name, dist in zip(["A0", "A1", "A2"], distances):
        x, y = anchors[anchor_name]
        if dist > 0:
            # Posisi perkiraan target dari anchor ini
            angle_all = 2 * math.pi * (len(weights) / 3)  # Membagi sudut 360° menjadi 3 bagian
            x_est = x + dist * math.cos(angle_all)
            y_est = y + dist * math.sin(angle_all)
            
            weight = 1 / (dist ** 2) if dist > 0 else 0  # Bobot berbanding terbalik dengan kuadrat jarak
            weights.append(weight)
            x_target += x_est * weight
            y_target += y_est * weight
    
    total_weight = sum(weights)
    if total_weight > 0:
        x_target /= total_weight
        y_target /= total_weight
    
    return x_target, y_target

def calculate_distance_and_angle(x_target, y_target):
    """
    Menghitung jarak dan sudut dari pusat robot ke target
    """
    # Jarak Euclidean dari pusat robot (0,0) ke target
    distance = math.sqrt(x_target**2 + y_target**2)
    
    # Sudut dari pusat robot ke target, diukur dari arah depan robot (sumbu-Y positif)
    # Menggunakan atan2(x, y) karena Y adalah arah depan robot
    angle = math.degrees(math.atan2(x_target, y_target))
    
    # Normalisasi sudut ke range [-180, 180]
    if angle > 180:
        angle -= 360
    elif angle < -180:
        angle += 360
    
    return distance, angle

try:
    ser = serial.Serial(port, baudrate, timeout=timeout)
    while True:
        if ser.in_waiting > 0:
            data = ser.readline().decode("utf-8").strip()
            if data.startswith("$KT0"):
                try:
                    parts = data.split(",")
                    if len(parts) >= 4:
                        raw_values = parts[1:4]
                        processed_values = []
                        for value in raw_values:
                            if value.lower() == "null":
                                processed_values.append(0.0)
                            else:
                                processed_values.append(float(value))
                        A0, A1, A2 = [val * 100 for val in processed_values]  # Konversi ke cm
                        
                        # Hitung posisi target
                        x_target, y_target = calculate_target_position([A0, A1, A2])
                        
                        # Hitung jarak dan sudut
                        distance, angle = calculate_distance_and_angle(x_target, y_target)
                        
                        output = f"A0={A0:.2f},A1={A1:.2f},A2={A2:.2f},X={x_target:.2f},Y={y_target:.2f},DIST={distance:.2f},ANGLE={angle:.2f}"
                        # Kirim data ke Raspberry Pi lain jika diperlukan
                        sock.sendto(output.encode(), (UDP_IP, UDP_PORT))
                        
                        print(f"A0={A0:.2f}cm | A1={A1:.2f}cm | A2={A2:.2f}cm | Target: X={x_target:.2f}cm, Y={y_target:.2f}cm | Jarak={distance:.2f}cm | Sudut={angle:.2f}°")
                    else:
                        print("Error: Data tidak lengkap.")
                except ValueError as e:
                    print(f"Error processing data: {e}")
except serial.SerialException as e:
    print(f"Error: {e}")
finally:
    if "ser" in locals() and ser.is_open:
        ser.close()
    print("Koneksi serial ditutup.")

