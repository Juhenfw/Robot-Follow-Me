import socket
import serial
import time
from collections import deque
import statistics

def main_uwb_transmitter():
    # Inisialisasi port serial UWB
    try:
        ser = serial.Serial(
            port="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",  # Ganti dengan port sesuai Raspberry Pi 1
            baudrate=115200,
            timeout=1
        )
    except serial.SerialException as e:
        print(f"Error membuka port serial: {e}")
        return
    
    # Inisialisasi socket untuk pengiriman data
    UDP_IP = "192.168.80.113"  # IP Raspberry Pi 2
    UDP_PORT = 5005           # Port untuk komunikasi
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Inisialisasi jendela median filter dengan panjang 5 untuk masing-masing nilai
    window_size = 5
    median_windows = {
        'A0': deque(maxlen=window_size),
        'A1': deque(maxlen=window_size),
        'A2': deque(maxlen=window_size)
    }
    
    try:
        while True:
            if ser.in_waiting > 0:
                # Baca data dari serial dengan decoding dan error handling
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                
                # Proses hanya pesan dengan Tag 0 ($KT0)
                if line.startswith("$KT0"):
                    try:
                        parts = line.split(",")
                        if len(parts) >= 4:
                            # Konversi data ke float; jika data "null" maka di-set ke 0.0
                            raw_values = {
                                'A0': float(parts[1]) if parts[1].lower() != "null" else 0.0,
                                'A1': float(parts[2]) if parts[2].lower() != "null" else 0.0,
                                'A2': float(parts[3]) if parts[3].lower() != "null" else 0.0
                            }
                            
                            # Update masing-masing jendela median filter
                            for key in raw_values:
                                median_windows[key].append(raw_values[key])
                            
                            # Hanya hitung median jika jendela sudah terisi (minimal satu sampel)
                            filtered_values = {}
                            for key in median_windows:
                                if len(median_windows[key]) > 0:
                                    filtered_values[key] = statistics.median(median_windows[key])
                                else:
                                    filtered_values[key] = raw_values[key]
                            
                            # Format nilai filtered menjadi string dengan 1 desimal
                            formatted_values = [f"{filtered_values['A0']:.1f}",
                                                f"{filtered_values['A1']:.1f}",
                                                f"{filtered_values['A2']:.1f}"]
                            
                            # Gabungkan data untuk dikirim
                            uwb_data = ",".join(formatted_values)
                            
                            # Kirim data ke Raspberry Pi 2 melalui UDP
                            sock.sendto(uwb_data.encode(), (UDP_IP, UDP_PORT))
                            
                            # Tampilkan output seperti pada tampilan sensor bawaan (SSCom32E)
                            print(f"Data terkirim (Tag 0 dengan Median Filter): {uwb_data}")
                            
                        # Jeda singkat untuk memastikan tidak terjadi overload
                        time.sleep(0.1)
                    
                    except Exception as e:
                        print(f"Error memproses data: {e}")
                        
    except KeyboardInterrupt:
        print("Transmitter dihentikan oleh pengguna.")
    
    finally:
        # Tutup koneksi serial
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Koneksi serial ditutup.")
        # Tutup koneksi socket
        sock.close()
        print("Koneksi socket ditutup.")

if __name__ == "__main__":
    main_uwb_transmitter()
