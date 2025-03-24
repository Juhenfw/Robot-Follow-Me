# RECEIVER
import socket

# UDP Configuration
UDP_IP = "0.0.0.0"  # Terima dari semua interface
UDP_PORT = 5005

# Buat socket UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Mendengarkan data pada port {UDP_PORT}...")

while True:
    # Terima data
    data, addr = sock.recvfrom(1024)  # Buffer size 1024 bytes
    message = data.decode()
    
    print(f"Menerima data: {message}")
    
    # Parse data
    parts = message.split(",")
    data_dict = {}
    for part in parts:
        key, value = part.split("=")
        data_dict[key] = float(value)
    
    # Contoh penggunaan data
    print(f"Jarak: {data_dict['DIST']} cm, Sudut: {data_dict['ANGLE']}°")

