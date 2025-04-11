import socket
import serial
import time
import numpy as np
from collections import deque
import math

class KalmanFilter:
    def __init__(self, process_variance=0.01, measurement_variance=0.1, initial_value=0):
        self.process_variance = process_variance  # Uncertainty in the system
        self.measurement_variance = measurement_variance  # Uncertainty in measurements
        self.estimate = initial_value  # Initial state
        self.estimate_error = 1.0  # Initial error estimate
        
    def update(self, measurement):
        # Prediction update
        prediction_error = self.estimate_error + self.process_variance
        
        # Measurement update
        kalman_gain = prediction_error / (prediction_error + self.measurement_variance)
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
        self.estimate_error = (1 - kalman_gain) * prediction_error
        
        return self.estimate

def median_filter(data, window_size):
    """Apply median filter to remove outliers"""
    if len(data) == 0:
        return 0.0
    return np.median(data)

def exponential_weighted_moving_average(data, alpha=0.2):
    """Apply EWMA to smooth data with more weight on recent values"""
    if len(data) == 0:
        return 0.0
    result = data[0]
    for i in range(1, len(data)):
        result = alpha * data[i] + (1 - alpha) * result
    return result

def detect_outliers(value, history, threshold_multiplier=2.0):
    """Detect if a value is an outlier based on historical data"""
    if len(history) < 3:
        return False  # Not enough history to determine outliers
    
    # Calculate median and MAD (Median Absolute Deviation)
    median_val = np.median(history)
    mad = np.median([abs(x - median_val) for x in history])
    
    # MAD is more robust to outliers than standard deviation
    if mad == 0:  # Avoid division by zero
        mad = 0.00001
    
    # Check if the value is an outlier using modified Z-score
    modified_z_score = 0.6745 * abs(value - median_val) / mad
    
    return modified_z_score > threshold_multiplier

def constrain_by_physics(value, prev_value, max_change=0.5, time_delta=0.1):
    """Constrain values based on physical limitations (max possible change per time)"""
    if prev_value is None:
        return value
    
    max_possible_change = max_change * time_delta  # Maximum change possible in this time frame
    
    if abs(value - prev_value) > max_possible_change:
        # Limit the change to what's physically possible
        if value > prev_value:
            return prev_value + max_possible_change
        else:
            return prev_value - max_possible_change
    
    return value

def noise_adaptive_filter(values, threshold=0.1):
    """Adapt filter parameters based on noise level"""
    if len(values) < 3:
        return np.mean(values) if values else 0
        
    # Calculate noise level (standard deviation)
    noise_level = np.std(values)
    
    if noise_level < threshold:
        # Low noise - use less aggressive filtering
        return np.mean(values)
    else:
        # High noise - use median (more robust)
        return np.median(values)

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
    
    # Window size untuk berbagai filter
    window_size = 10  # Jendela yang lebih besar untuk lingkungan bermasalah
    
    # Data history untuk setiap anchor
    history = {
        'A0': deque(maxlen=window_size),
        'A1': deque(maxlen=window_size),
        'A2': deque(maxlen=window_size)
    }
    
    # Inisialisasi Kalman Filter untuk setiap anchor
    kalman_filters = {
        'A0': KalmanFilter(process_variance=0.01, measurement_variance=0.5),
        'A1': KalmanFilter(process_variance=0.01, measurement_variance=0.5),
        'A2': KalmanFilter(process_variance=0.01, measurement_variance=0.5)
    }
    
    # Nilai terakhir yang valid untuk physical constraint
    last_valid_values = {
        'A0': None,
        'A1': None,
        'A2': None
    }
    
    # Pengukuran waktu untuk physical constraints
    last_measurement_time = time.time()
    
    # Statistik untuk adaptasi filter
    stats = {
        'A0': {'std': 0, 'values': deque(maxlen=30)},
        'A1': {'std': 0, 'values': deque(maxlen=30)},
        'A2': {'std': 0, 'values': deque(maxlen=30)}
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
                            # Hitung delta waktu sejak pengukuran terakhir
                            current_time = time.time()
                            time_delta = current_time - last_measurement_time
                            last_measurement_time = current_time
                            
                            # Konversi data ke float; jika data "null" maka di-set ke None
                            raw_values = {
                                'A0': float(parts[1]) if parts[1].lower() != "null" else None,
                                'A1': float(parts[2]) if parts[2].lower() != "null" else None,
                                'A2': float(parts[3]) if parts[3].lower() != "null" else None
                            }
                            
                            filtered_values = {}
                            
                            for key in raw_values:
                                # Skip jika nilai null
                                if raw_values[key] is None:
                                    filtered_values[key] = last_valid_values[key] if last_valid_values[key] is not None else 0.0
                                    continue
                                
                                # 1. Deteksi outlier
                                is_outlier = detect_outliers(raw_values[key], list(history[key]), 2.5)
                                
                                if is_outlier and last_valid_values[key] is not None:
                                    # Gunakan nilai sebelumnya jika outlier terdeteksi
                                    value = last_valid_values[key]
                                else:
                                    # 2. Terapkan physical constraints
                                    value = constrain_by_physics(
                                        raw_values[key], 
                                        last_valid_values[key], 
                                        max_change=1.5,  # Maksimum perubahan dalam meter/detik
                                        time_delta=time_delta
                                    )
                                    
                                    # Update history untuk filter
                                    history[key].append(value)
                                    
                                    # Update statistik untuk adaptasi filter
                                    stats[key]['values'].append(value)
                                    if len(stats[key]['values']) > 5:
                                        stats[key]['std'] = np.std(list(stats[key]['values']))
                                    
                                    # 3. Terapkan median filter untuk outlier lainnya
                                    median_value = median_filter(list(history[key]), window_size)
                                    
                                    # 4. Terapkan EWMA untuk smoothing
                                    if len(history[key]) > 2:
                                        ewma_value = exponential_weighted_moving_average(list(history[key]))
                                    else:
                                        ewma_value = value
                                    
                                    # 5. Gunakan Kalman Filter untuk final smoothing dan prediction
                                    kalman_value = kalman_filters[key].update(ewma_value)
                                    
                                    # 6. Pilih filter berdasarkan tingkat noise
                                    if stats[key]['std'] > 0.3:  # Noise tinggi
                                        # Lingkungan bermasalah (high noise) - gunakan Kalman
                                        value = kalman_value
                                    elif stats[key]['std'] > 0.1:  # Noise sedang
                                        # Noise sedang - gunakan median
                                        value = median_value
                                    else:  # Noise rendah
                                        # Noise rendah - gunakan EWMA
                                        value = ewma_value
                                
                                # Simpan nilai terakhir yang valid
                                last_valid_values[key] = value
                                filtered_values[key] = value
                            
                            # Format nilai filtered menjadi string dengan 3 desimal
                            formatted_values = [
                                f"{filtered_values['A0']:.3f}",
                                f"{filtered_values['A1']:.3f}",
                                f"{filtered_values['A2']:.3f}"
                            ]
                            
                            # Gabungkan data untuk dikirim
                            uwb_data = ",".join(formatted_values)
                            
                            # Kirim data ke Raspberry Pi 2 melalui UDP
                            sock.sendto(uwb_data.encode(), (UDP_IP, UDP_PORT))
                            
                            # Tampilkan output dengan info noise level
                            noise_levels = [f"A{i} noise: {stats[f'A{i}']['std']:.3f}" for i in range(3)]
                            noise_info = ", ".join(noise_levels)
                            print(f"Data terkirim: {uwb_data} | {noise_info}")
                            
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
