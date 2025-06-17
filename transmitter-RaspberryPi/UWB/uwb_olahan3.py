import socket
import serial
import time

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
    
    # Parameter untuk teknik Exponential Moving Average (EMA)
    alpha = 0.8 # Faktor smoothing (antara 0 dan 1)
    filtered_values = {'A0': None, 'A1': None, 'A2': None}
    
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
                            # Konversi data ke float; jika data 'null' maka di-set ke 0.0
                            raw_values = {
                                'A0': float(parts[1]) if parts[1].lower() != "null" else 0.0,
                                'A1': float(parts[2]) if parts[2].lower() != "null" else 0.0,
                                'A2': float(parts[3]) if parts[3].lower() != "null" else 0.0
                            }
                            
                            # Terapkan EMA untuk setiap nilai
                            for key in raw_values:
                                # Jika belum ada nilai filtered, inisialisasi dengan nilai raw
                                if filtered_values[key] is None:
                                    filtered_values[key] = raw_values[key]
                                else:
                                    filtered_values[key] = alpha * raw_values[key] + (1 - alpha) * filtered_values[key]
                            
                            # Format nilai filtered menjadi string dengan 1 desimal
                            formatted_values = [f"{filtered_values['A0']:.3f}",
                                                f"{filtered_values['A1']:.3f}",
                                                f"{filtered_values['A2']:.3f}"]
                            
                            # Gabungkan data untuk dikirim
                            uwb_data = ",".join(formatted_values)
                            
                            # Kirim data ke Raspberry Pi 2 melalui UDP
                            sock.sendto(uwb_data.encode(), (UDP_IP, UDP_PORT))
                            
                            # Tampilkan output seperti pada tampilan sensor bawaan
                            print(f"Data terkirim (Tag 0, stabilized): {uwb_data}")
                            
                        # Jeda singkat untuk memastikan tidak terjadi overload
                        time.sleep(0.001)
                    
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