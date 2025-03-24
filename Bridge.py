import socket
import time
import math
import numpy as np
from collections import deque

# UDP Configuration
UDP_IP = "0.0.0.0"  # Terima dari semua interface
UDP_PORT = 5005
BUFFER_SIZE = 1024

# Socket timeout (seconds)
SOCKET_TIMEOUT = 1.0

# Batas waktu untuk data yang dianggap valid
MAX_DATA_AGE = 2.0  # seconds

class AngleStabilizer:
    def __init__(self, buffer_size=10, max_angle_change=30.0, alpha=0.1):
        """
        Inisialisasi stabilizer sudut dengan filter khusus untuk sudut
        
        Parameters:
        - buffer_size: ukuran buffer untuk filter median
        - max_angle_change: perubahan sudut maksimum yang diperbolehkan dalam satu langkah (degrees)
        - alpha: parameter untuk filter low-pass exponential smoothing
        """
        self.buffer = deque(maxlen=buffer_size)
        self.max_angle_change = max_angle_change
        self.alpha = alpha
        self.last_stable_angle = 0.0
        self.is_initialized = False
    
    def _normalize_angle(self, angle):
        """Normalisasi sudut ke range [-180, 180]"""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
    
    def _angle_difference(self, angle1, angle2):
        """Hitung perbedaan sudut dengan cara yang benar untuk nilai sudut"""
        diff = angle1 - angle2
        return self._normalize_angle(diff)
    
    def update(self, new_angle):
        """
        Update sudut stabil dengan nilai sudut baru
        Menggunakan kombinasi filter outlier, median, dan exponential smoothing
        """
        # Handle penyesuaian ke -180/180 boundary
        new_angle = self._normalize_angle(new_angle)
        
        # Inisialisasi jika buffer kosong
        if not self.is_initialized:
            self.buffer.extend([new_angle] * self.buffer.maxlen)
            self.last_stable_angle = new_angle
            self.is_initialized = True
            return new_angle
        
        # Filter outlier: cek apakah perubahan sudut terlalu drastis
        angle_diff = self._angle_difference(new_angle, self.last_stable_angle)
        
        if abs(angle_diff) > self.max_angle_change:
            # Jika perubahan terlalu drastis, batasi perubahan tersebut
            limited_angle = self.last_stable_angle + self.max_angle_change * (angle_diff / abs(angle_diff))
            limited_angle = self._normalize_angle(limited_angle)
            self.buffer.append(limited_angle)
        else:
            self.buffer.append(new_angle)
        
        # Filter median untuk menghilangkan noise
        sorted_angles = sorted(self.buffer)
        median_angle = sorted_angles[len(sorted_angles) // 2]
        
        # Exponential smoothing untuk transisi halus
        smooth_angle = self.last_stable_angle + self.alpha * self._angle_difference(median_angle, self.last_stable_angle)
        smooth_angle = self._normalize_angle(smooth_angle)
        
        self.last_stable_angle = smooth_angle
        return smooth_angle

class KalmanFilter:
    def __init__(self, process_variance=1e-4, measurement_variance=1e-2):
        """
        Implementasi Kalman Filter sederhana 1D untuk posisi
        
        Parameters:
        - process_variance: variance dari proses (Q)
        - measurement_variance: variance dari pengukuran (R)
        """
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = 0.0  # x(k)
        self.estimate_error = 1.0  # P(k)
        self.is_initialized = False
    
    def update(self, measurement):
        """Update filter dengan pengukuran baru dan return estimasi terbaru"""
        if not self.is_initialized:
            self.estimate = measurement
            self.is_initialized = True
            return measurement
        
        # Predict
        prediction = self.estimate
        prediction_error = self.estimate_error + self.process_variance
        
        # Update
        kalman_gain = prediction_error / (prediction_error + self.measurement_variance)
        self.estimate = prediction + kalman_gain * (measurement - prediction)
        self.estimate_error = (1 - kalman_gain) * prediction_error
        
        return self.estimate

class PositionFilter:
    def __init__(self):
        """Filter posisi menggunakan Kalman Filter untuk x, y, dan jarak"""
        self.x_filter = KalmanFilter(process_variance=1e-4, measurement_variance=1e-2)
        self.y_filter = KalmanFilter(process_variance=1e-4, measurement_variance=1e-2)
        self.distance_filter = KalmanFilter(process_variance=1e-4, measurement_variance=1e-2)
        self.angle_stabilizer = AngleStabilizer()
    
    def update(self, x, y, distance, angle):
        """Update semua filter dengan pengukuran baru"""
        filtered_x = self.x_filter.update(x)
        filtered_y = self.y_filter.update(y)
        filtered_distance = self.distance_filter.update(distance)
        
        # Hitung ulang sudut dari posisi yang difilter (lebih stabil daripada memfilter sudut langsung)
        if abs(filtered_x) > 0.001 or abs(filtered_y) > 0.001:  # Hindari pembagian dengan nol
            calculated_angle = math.degrees(math.atan2(filtered_x, filtered_y))
            # Masih menggunakan angle_stabilizer untuk smoothing final
            filtered_angle = self.angle_stabilizer.update(calculated_angle)
        else:
            # Jika hampir (0,0), gunakan sudut yang diukur langsung
            filtered_angle = self.angle_stabilizer.update(angle)
        
        return filtered_x, filtered_y, filtered_distance, filtered_angle

# Fungsi untuk mengontrol motor
def control_motors(distance, angle):
    """
    Kontrol motor berdasarkan jarak dan sudut ke target
    
    Parameters:
    - distance: jarak ke target (cm)
    - angle: sudut ke target (-180 to 180 degrees)
                0 = target di depan robot
                +90 = target di kanan robot
                -90 = target di kiri robot
    """
    # Kecepatan dasar dan parameter kontrol
    BASE_SPEED = 50  # Kecepatan dasar (0-100)
    MAX_SPEED = 100  # Kecepatan maksimum
    MIN_SPEED = 20   # Kecepatan minimum
    
    # Implementasi PID sederhana alih-alih kontrol proporsional
    # Konstanta PID
    KP_ANGLE = 1.0     # Proporsional
    KI_ANGLE = 0.01    # Integral
    KD_ANGLE = 0.5     # Derivatif
    
    # State untuk PID
    global angle_integral, last_angle, last_time
    if 'angle_integral' not in globals():
        angle_integral = 0
        last_angle = 0
        last_time = time.time()
    
    # Hitung dt (delta time)
    current_time = time.time()
    dt = current_time - last_time
    if dt <= 0: dt = 0.01  # Hindari pembagian dengan nol
    
    # Perhitungan PID
    angle_error = angle
    angle_integral += angle_error * dt
    angle_integral = max(-50, min(50, angle_integral))  # Anti-windup
    angle_derivative = (angle_error - last_angle) / dt
    
    # Update state
    last_angle = angle_error
    last_time = current_time
    
    # Hitung sinyal kontrol
    turn_rate = KP_ANGLE * angle_error + KI_ANGLE * angle_integral + KD_ANGLE * angle_derivative
    
    # Tentukan kecepatan motor berdasarkan jarak dan sudut
    # Semakin dekat target, semakin lambat robot bergerak
    speed = min(BASE_SPEED, BASE_SPEED * distance / 100) if distance < 100 else BASE_SPEED
    
    # Hitung kecepatan untuk motor kiri dan kanan
    left_speed = speed - turn_rate
    right_speed = speed + turn_rate
    
    # Normalisasi kecepatan ke rentang yang valid
    max_current_speed = max(abs(left_speed), abs(right_speed))
    if max_current_speed > MAX_SPEED:
        scaling_factor = MAX_SPEED / max_current_speed
        left_speed *= scaling_factor
        right_speed *= scaling_factor
    
    # Pastikan kecepatan minimum tercapai jika robot bergerak
    if 0 < left_speed < MIN_SPEED:
        left_speed = MIN_SPEED
    elif -MIN_SPEED < left_speed < 0:
        left_speed = -MIN_SPEED
        
    if 0 < right_speed < MIN_SPEED:
        right_speed = MIN_SPEED
    elif -MIN_SPEED < right_speed < 0:
        right_speed = -MIN_SPEED
    
    # Berhenti jika sangat dekat dengan target
    if distance < 10:  # 10cm
        left_speed = 0
        right_speed = 0
        print("Target tercapai! Menghentikan motor.")
    
    print(f"Kontrol motor: Kiri={left_speed:.1f}, Kanan={right_speed:.1f}")
    
    # Implementasi kontrol motor sesuai dengan hardware Anda
    # Contoh:
    # set_motor_speed(LEFT_MOTOR, left_speed)
    # set_motor_speed(RIGHT_MOTOR, right_speed)

# Buat position filter
position_filter = PositionFilter()

# Buat socket UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(SOCKET_TIMEOUT)

print(f"UWB Data Receiver dengan Stabilisasi Sudut dimulai pada port {UDP_PORT}...")
last_data_time = 0

try:
    while True:
        try:
            # Terima data dengan timeout
            data, addr = sock.recvfrom(BUFFER_SIZE)
            print(f"Menerima data dari {addr}")
            
            # Parse data yang diterima
            try:
                data_str = data.decode('utf-8')
                parts = data_str.split(",")
                
                # Buat dictionary dari data
                data_dict = {}
                for part in parts:
                    key, value = part.split("=")
                    data_dict[key] = float(value)
                
                # Ekstrak data
                current_time = time.time()
                
                # Dapatkan nilai dari data yang diterima
                x = data_dict.get("X", 0)
                y = data_dict.get("Y", 0)
                distance = data_dict.get("DIST", 0)
                angle = data_dict.get("ANGLE", 0)
                
                # Terapkan filter canggih pada data
                x_filtered, y_filtered, distance_filtered, angle_filtered = position_filter.update(x, y, distance, angle)
                
                # Debug: Bandingkan nilai asli vs nilai yang difilter
                print(f"Raw Data: X={x:.2f}, Y={y:.2f}, Distance={distance:.2f}, Angle={angle:.2f}")
                print(f"Filtered: X={x_filtered:.2f}, Y={y_filtered:.2f}, Distance={distance_filtered:.2f}, Angle={angle_filtered:.2f}")
                
                # Tampilkan informasi anchor
                print(f"Jarak Anchor: A0={data_dict.get('A0', 0):.2f}cm, A1={data_dict.get('A1', 0):.2f}cm, A2={data_dict.get('A2', 0):.2f}cm")
                
                # Kontrol motor berdasarkan jarak dan sudut yang sudah difilter
                control_motors(distance_filtered, angle_filtered)
                last_data_time = current_time
                
            except ValueError as e:
                print(f"Error parsing data: {e}")
                
        except socket.timeout:
            # Periksa apakah kita kehilangan koneksi
            current_time = time.time()
            if last_data_time > 0 and (current_time - last_data_time) > MAX_DATA_AGE:
                print("Tidak ada data posisi baru yang diterima. Menghentikan motor.")
                # Implementasi stop motor di sini
                
        except Exception as e:
            print(f"Unexpected error: {e}")
            import traceback
            traceback.print_exc()
            time.sleep(1)
            
except KeyboardInterrupt:
    print("\nProgram dihentikan oleh pengguna")
except Exception as e:
    print(f"Fatal error: {e}")
    import traceback
    traceback.print_exc()
finally:
    # Hentikan motor jika ada
    print("Menutup socket")
    sock.close()
    print("Receiver script dihentikan")
