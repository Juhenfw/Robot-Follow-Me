import socket
import serial
import time
import numpy as np
from collections import deque

class AdaptiveKalmanFilter:
    def __init__(self, process_variance=0.125, measurement_variance=0.5, initial_value=0):
        self.process_variance = process_variance  # Diperbesar untuk responsif terhadap perubahan
        self.measurement_variance = measurement_variance
        self.estimate = initial_value
        self.estimate_error = 1.0
        self.last_measurement = initial_value
        self.movement_detected = False
        
    def update(self, measurement, force_update=False):
        # Deteksi pergerakan
        movement = abs(measurement - self.last_measurement) > 0.1
        self.movement_detected = movement
        
        # Saat pergerakan terdeteksi, tingkatkan responsivitas
        if movement or force_update:
            proc_variance = self.process_variance * 3.0  # Lebih responsif
            meas_variance = self.measurement_variance * 0.5  # Lebih mempercayai pengukuran
        else:
            proc_variance = self.process_variance
            meas_variance = self.measurement_variance
            
        # Prediction update
        prediction_error = self.estimate_error + proc_variance
        
        # Measurement update
        kalman_gain = prediction_error / (prediction_error + meas_variance)
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
        self.estimate_error = (1 - kalman_gain) * prediction_error
        
        self.last_measurement = measurement
        return self.estimate

def median_filter(data, window_size):
    """Apply median filter to remove outliers"""
    if len(data) == 0:
        return 0.0
    # Hanya gunakan 3 nilai terakhir untuk responsivitas
    recent_data = list(data)[-3:]
    return np.median(recent_data)

def main_uwb_transmitter():
    # Inisialisasi port serial UWB
    try:
        ser = serial.Serial(
            port="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
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
    
    # Gunakan window kecil agar responsif
    window_size = 5
    
    # Data history untuk setiap anchor
    history = {
        'A0': deque(maxlen=window_size),
        'A1': deque(maxlen=window_size),
        'A2': deque(maxlen=window_size)
    }
    
    # Inisialisasi Kalman Filter dengan parameter lebih responsif
    kalman_filters = {
        'A0': AdaptiveKalmanFilter(process_variance=0.125, measurement_variance=0.5),
        'A1': AdaptiveKalmanFilter(process_variance=0.125, measurement_variance=0.5),
        'A2': AdaptiveKalmanFilter(process_variance=0.125, measurement_variance=0.5)
    }
    
    # Nilai terakhir yang valid
    last_values = {
        'A0': None,
        'A1': None,
        'A2': None
    }
    
    # Pendeteksi stabilitas - melacak berapa lama nilai stabil
    stability_counter = {
        'A0': 0,
        'A1': 0,
        'A2': 0
    }
    
    # Nilai delta untuk mendeteksi pergerakan signifikan
    movement_threshold = 0.1
    
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
                            # Konversi data ke float; jika data "null" maka di-set ke None
                            raw_values = {
                                'A0': float(parts[1]) if parts[1].lower() != "null" else None,
                                'A1': float(parts[2]) if parts[2].lower() != "null" else None,
                                'A2': float(parts[3]) if parts[3].lower() != "null" else None
                            }
                            
                            filtered_values = {}
                            movement_detected = False
                            
                            for key in raw_values:
                                # Skip jika nilai null
                                if raw_values[key] is None:
                                    # Gunakan nilai terakhir jika ada
                                    filtered_values[key] = last_values[key] if last_values[key] is not None else 0.0
                                    continue
                                
                                # Tambahkan ke history
                                history[key].append(raw_values[key])
                                
                                # Deteksi pergerakan signifikan
                                if last_values[key] is not None:
                                    delta = abs(raw_values[key] - last_values[key])
                                    if delta > movement_threshold:
                                        movement_detected = True
                                        stability_counter[key] = 0
                                    else:
                                        stability_counter[key] += 1
                                
                                # Jika stabil terlalu lama (stuck), paksa reset
                                force_reset = stability_counter[key] > 10
                                
                                # FILTERING STRATEGY:
                                # 1. Saat mendeteksi gerakan, gunakan nilai mentah dengan median filter ringan
                                # 2. Saat stabil, gunakan Kalman filter adaptif
                                
                                if movement_detected or len(history[key]) < 3:
                                    # Gunakan median filter ringan saja saat bergerak
                                    median_val = median_filter(history[key], 3)
                                    # Kalman masih diupdate tapi dengan bobot rendah (tracking mode)
                                    kalman_val = kalman_filters[key].update(median_val, force_update=True)
                                    # Saat bergerak, percayai median filter lebih banyak
                                    filtered_values[key] = median_val * 0.7 + kalman_val * 0.3
                                else:
                                    # Mode stabil - gunakan Kalman untuk smoothing
                                    # tapi tetap responsif terhadap perubahan
                                    kalman_val = kalman_filters[key].update(raw_values[key])
                                    median_val = median_filter(history[key], window_size)
                                    
                                    # Mix keduanya - tetap responsif tapi stabil
                                    filtered_values[key] = kalman_val * 0.7 + median_val * 0.3
                                
                                # Force reset jika terlalu lama stabil - mencegah "stuck"
                                if force_reset:
                                    filtered_values[key] = raw_values[key]
                                    kalman_filters[key].estimate = raw_values[key]
                                    print(f"Reset filter {key} - nilai stabil terlalu lama")
                                    stability_counter[key] = 0
                                
                                # Simpan nilai terakhir
                                last_values[key] = raw_values[key]
                            
                            # Format nilai filtered menjadi string dengan 3 desimal
                            formatted_values = [
                                f"{filtered_values['A0']:.3f}",
                                f"{filtered_values['A1']:.3f}",
                                f"{filtered_values['A2']:.3f}"
                            ]
                            
                            # Format nilai mentah untuk debug
                            raw_formatted = [
                                f"{raw_values['A0']:.3f}" if raw_values['A0'] is not None else "null",
                                f"{raw_values['A1']:.3f}" if raw_values['A1'] is not None else "null",
                                f"{raw_values['A2']:.3f}" if raw_values['A2'] is not None else "null"
                            ]
                            
                            # Gabungkan data untuk dikirim
                            uwb_data = ",".join(formatted_values)
                            
                            # Kirim data ke Raspberry Pi 2 melalui UDP
                            sock.sendto(uwb_data.encode(), (UDP_IP, UDP_PORT))
                            
                            # Tampilkan output dengan raw dan filtered untuk perbandingan
                            raw_data = ",".join(raw_formatted)
                            status = "MOVING" if movement_detected else "STABLE"
                            print(f"Raw: {raw_data} | Filtered: {uwb_data} | Status: {status}")
                            
                        # Jeda singkat
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
