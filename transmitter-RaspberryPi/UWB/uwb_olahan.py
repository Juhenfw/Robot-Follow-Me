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
    UDP_PORT = 5005         # Port untuk komunikasi
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    try:
        while True:
            if ser.in_waiting > 0:
                # Baca data dari serial
                data = ser.readline().decode("utf-8").strip()
                
                # Pastikan data valid
                if data.startswith("$KT0"):
                    try:
                        # Parse data UWB
                        parts = data.split(",")
                        if len(parts) >= 4:
                            # Konversi data ke float
                            raw_values = [
                                float(parts[1]) if parts[1].lower() != "null" else 0.0,
                                float(parts[2]) if parts[2].lower() != "null" else 0.0,
                                float(parts[3]) if parts[3].lower() != "null" else 0.0
                            ]
                            
                            # Kirim data ke Raspberry Pi 2
                            uwb_data = ",".join([str(val) for val in raw_values])
                            sock.sendto(uwb_data.encode(), (UDP_IP, UDP_PORT))
                            print(f"Data terkirim: {uwb_data}")
                        
                        # Jeda untuk mencegah overload
                        time.sleep(0.5)
                    
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