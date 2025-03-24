import socket
import math

# Alamat IP Raspberry Pi yang berada di robot
IP_ROBOT = '192.168.1.100'  # Ganti dengan alamat IP Raspberry Pi robot
PORT = 65432  # Port yang sama dengan server

# Konfigurasi Anchor dan Perhitungan
L = 65  # Panjang robot
W = 35  # Lebar robot
anchors = {
    "A0": (-L / 2, W / 2),  # Depan kiri
    "A1": (-L / 2, -W / 2),  # Depan kanan
    "A2": (L / 2, 0),  # Belakang tengah
}

# Fungsi perhitungan posisi tag berdasarkan data dari UWB
def calculate_tag_position(distances):
    weights = []
    x_tag, y_tag = 0, 0

    for i, (anchor, dist) in enumerate(zip(anchors.values(), distances)):
        x, y = anchor
        weight = 1 / dist if dist > 0 else 0
        weights.append(weight)
        x_tag += x * weight
        y_tag += y * weight

    total_weight = sum(weights)

    if total_weight > 0:
        x_tag /= total_weight
        y_tag /= total_weight

    return x_tag, y_tag

# Fungsi perhitungan jarak dan sudut
def calculate_distance_and_angle(x_tag, y_tag):
    distance = math.sqrt(x_tag**2 + y_tag**2)
    angle = math.degrees(math.atan2(y_tag, x_tag))
    return distance, angle

try:
    # Membuat koneksi ke server (Raspberry Pi robot)
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((IP_ROBOT, PORT))
    print(f"Terkoneksi dengan server {IP_ROBOT}:{PORT}")

    while True:
        # Data jarak UWB (contoh data, ganti dengan pembacaan sensor Anda)
        distances = [1.5, 2.0, 1.8]  # Jarak dalam meter
        x_tag, y_tag = calculate_tag_position(distances)
        distance, angle = calculate_distance_and_angle(x_tag, y_tag)

        # Buat pesan yang akan dikirim
        message = f"Distance: {distance:.2f} cm, Angle: {angle:.2f}°"
        client_socket.sendall(message.encode('utf-8'))
        print(f"Data dikirim: {message}")

except KeyboardInterrupt:
    print("Pengiriman data dihentikan.")
finally:
    client_socket.close()
