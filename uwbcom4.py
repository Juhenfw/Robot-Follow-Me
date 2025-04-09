import socket
import numpy as np
from scipy.optimize import minimize
import math
import time
import ddsm115 as motor

# Fixed device paths for consistent hardware connections
RIGHT_WHEEL_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B0049TUZ-if00-port0"
LEFT_WHEEL_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B0045S9B-if00-port0"

class UWBTracker:
    def __init__(self, fixed_anchor_positions, initial_bias=None):
        """ Inisialisasi tracker UWB dengan posisi anchor dan manajemen bias """
        self.A1_pos = np.array(fixed_anchor_positions['A1'])
        self.A2_pos = np.array(fixed_anchor_positions['A2'])
        self.A0_pos = np.array(fixed_anchor_positions['A0'])  # Posisi A0 juga tetap
        self.fixed_angle = math.radians(60)  # Sudut tetap antara anchor
        self.bias = initial_bias or {'A0': 40, 'A1': 30, 'A2': 45}
        self.scale_factor = {'A0': 1.0, 'A1': 1.0, 'A2': 1.0}
    
    def apply_bias_correction(self, distances):
        """ Koreksi bias dan scaling pada pengukuran jarak """
        corrected_distances = {
            'A0': max((distances['A0'] * 100 * self.scale_factor['A0']) - self.bias['A0'], 0),
            'A1': max((distances['A1'] * 100 * self.scale_factor['A1']) - self.bias['A1'], 0),
            'A2': max((distances['A2'] * 100 * self.scale_factor['A2']) - self.bias['A2'], 0)
        }
        return corrected_distances
    
    def trilateration_cost(self, target_pos, corrected_distances, anchor_positions):
        """ Fungsi biaya untuk estimasi posisi target """
        calculated_distances = [
            np.linalg.norm(target_pos - anchor_positions['A1']),
            np.linalg.norm(target_pos - anchor_positions['A2']),
            np.linalg.norm(target_pos - anchor_positions['A0'])
        ]
        distance_errors = [
            ((calc - measured) ** 2) for calc, measured in zip(calculated_distances, 
            [corrected_distances['A1'], corrected_distances['A2'], corrected_distances['A0']])
        ]
        return np.sum(distance_errors)
    
    def estimate_target_position(self, distances, anchor_positions):
        """ Estimasi posisi target menggunakan optimasi trilateration """
        corrected_distances = self.apply_bias_correction(distances)
        initial_guess = np.mean([anchor_positions['A1'], anchor_positions['A2'], anchor_positions['A0']], axis=0)
        result = minimize(self.trilateration_cost, initial_guess, args=(corrected_distances, anchor_positions), method='Nelder-Mead', options={'maxiter': 100})
        diagnostics = {'raw_distances': distances, 'corrected_distances': corrected_distances, 'optimization_success': result.success, 'optimization_message': result.message}
        return result.x, diagnostics
    
    def calculate_angle(self, a, b, c):
        """ Hitung sudut menggunakan hukum kosinus """
        try:
            cos_angle = (a**2 + b**2 - c**2) / (2 * a * b)
            cos_angle = max(min(cos_angle, 1), -1)
            angle = math.degrees(math.acos(cos_angle))
            return angle
        except Exception as e:
            print(f"Error calculating angle: {e}")
            return None
    
    def calculate_robot_angle(self, target_pos, reference_pos):
        """ Hitung sudut robot untuk mengejar target """
        vector_to_target = target_pos - reference_pos
        angle = math.atan2(vector_to_target[1], vector_to_target[0])
        return math.degrees(angle)
    
    def print_tracking_info(self, corrected_distances, robot_angle=None, additional_angle=None):
        """ Cetak informasi tracking """
        print(f"\nA0 = {corrected_distances['A0']:.2f} cm | A1 = {corrected_distances['A1']:.2f} cm | A2 = {corrected_distances['A2']:.2f} cm")
        if robot_angle is not None:
            print(f"Sudut Robot: {robot_angle:.2f} derajat")
        if additional_angle is not None:
            print(f"Sudut Tambahan: {additional_angle:.2f} derajat")

class RobotController:
    def __init__(self, r_wheel_port, l_wheel_port):
        # Inisialisasi motor
        self.right_motor = motor.MotorControl(device=r_wheel_port)
        self.right_motor.set_drive_mode(1, 2)
        self.left_motor = motor.MotorControl(device=l_wheel_port)
        self.left_motor.set_drive_mode(1, 2)
        
        # Parameter kecepatan
        self.base_speed = 75  # Kecepatan dasar
        self.slow_speed = self.base_speed / 2  # Kecepatan rendah
        self.turn_speed = self.base_speed / 4  # Kecepatan belok
        self.stop_speed = 0
        
        # Parameter kontrol
        self.min_follow_distance = 30  # Jarak minimum (cm) untuk berhenti
        self.max_follow_distance = 300  # Jarak maksimum (cm) untuk kecepatan penuh
        self.angle_threshold = 15  # Batas sudut (derajat) untuk gerak lurus
        
        # Parameter target distance
        self.target_distance = 150  # Jarak target yang diinginkan (cm)
        self.distance_threshold = 15  # Toleransi jarak (±15 cm)
        self.target_reached = False  # Status pencapaian target
        
    def move(self, right_speed, left_speed):
        """Menggerakkan robot dengan kecepatan tertentu"""
        self.right_motor.send_rpm(1, right_speed)
        self.left_motor.send_rpm(1, left_speed)
        
    def stop(self):
        """Menghentikan robot"""
        self.move(self.stop_speed, self.stop_speed)

    def follow_target(self, angle, distance_to_target):
        """Menentukan gerakan robot berdasarkan sudut dan jarak target"""
        # PROTOKOL KEAMANAN: Jika target terlalu dekat (kurang dari 30 cm), berhenti
        if distance_to_target < self.min_follow_distance:
            print(f"PROTOKOL KEAMANAN: Target berada dalam jarak {distance_to_target:.2f} cm (< {self.min_follow_distance} cm)")
            print("Robot berhenti untuk keamanan")
            self.stop()
            return
        
        # Cek jika robot sudah mencapai jarak target
        if abs(distance_to_target - self.target_distance) <= self.distance_threshold:
            print(f"TARGET TERCAPAI: Robot berada pada jarak {distance_to_target:.2f} cm dari target")
            print(f"(Jarak target: {self.target_distance} ± {self.distance_threshold} cm)")
            self.stop()
            return
        
        # Hitung faktor kecepatan berdasarkan jarak
        # Makin jauh target, makin cepat robot bergerak
        distance_factor = min(1.0, (distance_to_target - self.min_follow_distance) / 
                            (self.max_follow_distance - self.min_follow_distance))
        adjusted_speed = self.base_speed * distance_factor
        
        # Normalisasi sudut ke rentang 0-360
        norm_angle = angle % 360
        if norm_angle > 180:
            norm_angle -= 360
            
        print(f"Sudut target: {norm_angle:.2f}°, Jarak: {distance_to_target:.2f} cm, Kecepatan: {adjusted_speed:.2f}")
        
        # Logika pergerakan berdasarkan sudut target (menggunakan referensi diagram)
        # Depan (-30 hingga 30 derajat)
        if -self.angle_threshold <= norm_angle <= self.angle_threshold:
            # Maju lurus
            self.move(-adjusted_speed, adjusted_speed)
            print("Bergerak maju")
            
        # Depan Kanan (30 hingga 60 derajat)
        elif self.angle_threshold < norm_angle <= 60:
            # Belok kanan ringan
            self.move(-self.turn_speed, adjusted_speed)
            print("Belok kanan ringan")
            
        # Kanan (60 hingga 120 derajat)
        elif 60 < norm_angle <= 120:
            # Belok kanan tajam
            self.move(self.turn_speed, adjusted_speed)
            print("Belok kanan tajam")
            
        # Belakang Kanan (120 hingga 150 derajat)
        elif 120 < norm_angle <= 150:
            # Belok kanan mundur
            self.move(adjusted_speed, self.turn_speed)
            print("Belok kanan mundur")
            
        # Belakang (150 hingga -150 derajat)
        elif 150 < norm_angle or norm_angle <= -150:
            # Mundur
            self.move(adjusted_speed, -adjusted_speed)
            print("Bergerak mundur")
            
        # Belakang Kiri (-150 hingga -120 derajat)
        elif -150 < norm_angle <= -120:
            # Belok kiri mundur
            self.move(self.turn_speed, -adjusted_speed)
            print("Belok kiri mundur")
            
        # Kiri (-120 hingga -60 derajat)
        elif -120 < norm_angle <= -60:
            # Belok kiri tajam
            self.move(-adjusted_speed, -self.turn_speed)
            print("Belok kiri tajam")
            
        # Depan Kiri (-60 hingga -30 derajat)
        elif -60 < norm_angle < -self.angle_threshold:
            # Belok kiri ringan
            self.move(-adjusted_speed, self.turn_speed)
            print("Belok kiri ringan")
    
    def close(self):
        """Menutup koneksi motor"""
        self.stop()
        self.right_motor.close()
        self.left_motor.close()
        
def main_robot_follower():
    # Gunakan konstanta port yang sudah didefinisikan
    r_wheel_port = RIGHT_WHEEL_PORT
    l_wheel_port = LEFT_WHEEL_PORT
    
    # Inisialisasi socket UDP
    UDP_IP = "192.168.80.113"  # IP Raspberry Pi 2 (penerima)
    UDP_PORT = 5005
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    
    # Inisialisasi tracker UWB
    tracker = UWBTracker({
        'A0': [0, 0],     # Posisi A0 (tengah robot)
        'A1': [-0.16, -0.3],  # Posisi A1 (kiri bawah robot)
        'A2': [0.16, -0.3]    # Posisi A2 (kanan bawah robot)
    })
    
    # Inisialisasi robot controller
    robot = RobotController(r_wheel_port, l_wheel_port)
    # Konfigurasi jarak target (TAMBAHKAN INI jika ingin mengubah nilai default)
    robot.target_distance = 150  # Jarak target dalam cm
    robot.distance_threshold = 15  # Toleransi +-15 cm
    
    print("Robot siap mengikuti target UWB T0...")
    print(f"PROTOKOL KEAMANAN: Robot akan berhenti saat jarak target < {robot.min_follow_distance} cm")
    print("Tekan Ctrl+C untuk keluar.")
    
    try:
        while True:
            try:
                # Terima data dari Raspberry Pi 1 dengan timeout
                sock.settimeout(0.1)  # Timeout pendek untuk memeriksa Ctrl+C
                try:
                    data, addr = sock.recvfrom(1024)
                    print(f"Data diterima dari {addr}: {data.decode()}")
                except socket.timeout:
                    continue
                
                # Proses data UWB
                parts = data.decode().split(",")
                if len(parts) >= 3:
                    raw_values = [
                        float(parts[0]),
                        float(parts[1]),
                        float(parts[2])
                    ]
                    
                    # Siapkan data jarak
                    uwb_data = {
                        'A0': raw_values[0],
                        'A1': raw_values[1],
                        'A2': raw_values[2]
                    }
                    
                    # Koreksi bias pada jarak
                    corrected_distances = tracker.apply_bias_correction(uwb_data)
                    
                    # Estimasi posisi target
                    anchor_positions = {
                        'A0': np.array([0.5, 0.5]),
                        'A1': np.array([0, 1]),
                        'A2': np.array([1, 0])
                    }
                    
                    target_pos, diagnostics = tracker.estimate_target_position(
                        uwb_data,
                        anchor_positions
                    )
                    
                    # Hitung sudut robot ke target
                    robot_angle = tracker.calculate_robot_angle(
                        target_pos,
                        anchor_positions['A0']
                    )
                    
                    # Hitung sudut tambahan (hukum kosinus)
                    additional_angle = tracker.calculate_angle(
                        corrected_distances['A0'],
                        corrected_distances['A1'],
                        corrected_distances['A2']
                    )
                    
                    # Cetak informasi tracking
                    tracker.print_tracking_info(
                        corrected_distances,
                        robot_angle,
                        additional_angle
                    )
                    
                    # Cetak informasi tambahan
                    print("Diagnostik:")
                    print(f"Estimasi Posisi Target: {target_pos}")
                    print(f"Status Optimasi: {diagnostics['optimization_success']}")
                    
                    # Hitung jarak ke target dari A0 (pusat robot)
                    distance_to_target = corrected_distances['A0']
                    
                    # Kontrol robot berdasarkan data tracking
                    robot.follow_target(robot_angle, distance_to_target)
                    
            except Exception as e:
                print(f"Error memproses data: {e}")
                robot.stop()  # Hentikan robot jika terjadi error
                
    except KeyboardInterrupt:
        print("Program dihentikan oleh pengguna.")
        
    finally:
        # Bersihkan koneksi
        print("Membersihkan dan menutup semua koneksi...")
        robot.close()
        sock.close()
        print("Koneksi socket dan motor ditutup.")

if __name__ == "__main__":
    main_robot_follower()
