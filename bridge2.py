import socket
import time
import math
import struct
from dwm1000 import DWM1000  # Library untuk modul UWB DWM1000

# Konfigurasi UWB
ANCHOR_ID = 0  # ID unik untuk anchor ini (0, 1, 2, dst)
ANCHOR_POSITIONS = {
    0: (0.0, 0.0, 0.0),      # Anchor 0 position (x, y, z) dalam meter
    1: (5.0, 0.0, 0.0),      # Anchor 1 position
    2: (0.0, 5.0, 0.0),      # Anchor 2 position
}

# Konfigurasi UDP
ROBOT_IP = "192.168.1.100"  # IP robot penerima
UDP_PORT = 5005
BROADCAST_INTERVAL = 0.1  # Interval pengiriman dalam detik

# Inisialisasi modul DWM1000
dwm = DWM1000()
dwm.initialize()
dwm.setAntennaDelay(16384)  # Kalibrasi delay antena
dwm.setAnchorPosition(ANCHOR_POSITIONS[ANCHOR_ID])
dwm.setAnchorID(ANCHOR_ID)

# Inisialisasi socket UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"UWB Anchor ID {ANCHOR_ID} dimulai pada posisi {ANCHOR_POSITIONS[ANCHOR_ID]}")
print(f"Mengirim data ke {ROBOT_IP}:{UDP_PORT}")

def measure_distances():
    """Fungsi untuk mengukur jarak ke semua tag yang terdeteksi"""
    distances = {}
    tags = dwm.detectTags()  # Deteksi tag yang berada dalam jangkauan
    
    for tag_id in tags:
        # Menggunakan SDS-TWR (Symmetrical Double-Sided Two Way Ranging)
        distance = dwm.measureDistance(tag_id)
        distances[tag_id] = distance
        
    return distances

try:
    while True:
        try:
            # Ukur jarak ke tag
            distances = measure_distances()
            
            # Jika tidak ada tag yang terdeteksi, lanjutkan loop
            if not distances:
                print("Tidak ada tag terdeteksi")
                time.sleep(BROADCAST_INTERVAL)
                continue
            
            # Format data untuk dikirim
            timestamp = time.time()
            data = f"ANCHOR={ANCHOR_ID},TIME={timestamp:.6f}"
            
            # Tambahkan data jarak untuk setiap tag
            for tag_id, distance in distances.items():
                data += f",TAG{tag_id}={distance:.3f}"
            
            # Kirim data via UDP
            sock.sendto(data.encode('utf-8'), (ROBOT_IP, UDP_PORT))
            print(f"Data terkirim: {data}")
            
            time.sleep(BROADCAST_INTERVAL)
            
        except Exception as e:
            print(f"Error: {e}")
            time.sleep(1)
            
except KeyboardInterrupt:
    print("\nProgram dihentikan oleh pengguna")
finally:
    print("Menutup socket dan modul UWB")
    sock.close()
    dwm.close()
