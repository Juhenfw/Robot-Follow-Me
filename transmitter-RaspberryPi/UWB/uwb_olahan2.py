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
    
    # Inisialisasi socket
    UDP_IP = "192.168.80.113"  # IP Raspberry Pi 2
    UDP_PORT = 5005           # Port untuk komunikasi
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    try:
        while True:
            if ser.in_waiting > 0:
                # Baca data dari serial dan lakukan decoding dengan hati-hati
                line = ser.readline().decode("utf-8", errors="ignore").strip()
                
                # Proses hanya pesan dengan Tag 0 ($KT0)
                if line.startswith("$KT0"):
                    try:
                        # Parse data UWB
                        parts = line.split(",")
                        if len(parts) >= 4:
                            # Data sensor biasanya berasal dari format "$KT0,<val1>,<val2>,<val3>,..."
                            # Konversi data ke float, jika tidak valid (misal 'null') di-set ke 0.0
                            raw_values = [
                                float(parts[1]) if parts[1].lower() != "null" else 0.0,
                                float(parts[2]) if parts[2].lower() != "null" else 0.0,
                                float(parts[3]) if parts[3].lower() != "null" else 0.0
                            ]
                            # Untuk akurasi seperti SSCom32E, hasil konversi dapat diformat (misal dalam cm) dengan 1 desimal
                            formatted_values = [f"{val:.1f}" for val in raw_values]
                            
                            # Gabungkan kembali data untuk dikirim ke receiver
                            uwb_data = ",".join(formatted_values)
                            
                            # Kirim data ke Raspberry Pi 2
                            sock.sendto(uwb_data.encode(), (UDP_IP, UDP_PORT))
                            
                            # Tampilkan output seperti pada display sensor bawaan SSCom32E
                            print(f"Data terkirim (Tag 0): {uwb_data}")
                            
                        # Jeda sejenak untuk menghindari overload
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
        # Tutup socket
        sock.close()
        print("Koneksi socket ditutup.")

if __name__ == "__main__":
    main_uwb_transmitter()
