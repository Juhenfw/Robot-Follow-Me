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
        self.stop_speed = 0
        
        # Parameter kontrol
        self.min_follow_distance = 30  # Jarak minimum (cm) untuk berhenti
        self.max_follow_distance = 300  # Jarak maksimum (cm) untuk kecepatan penuh
        
        # Parameter target distance
        self.target_distance = 150  # Jarak target yang diinginkan (cm)
        self.distance_threshold = 15  # Toleransi jarak (Â±15 cm)
        self.target_reached = False  # Status pencapaian target
        
    def move(self, right_speed, left_speed):
        """Menggerakkan robot dengan kecepatan tertentu"""
        self.right_motor.send_rpm(1, right_speed)
        self.left_motor.send_rpm(1, left_speed)
        
    def stop(self):
        """Menghentikan robot"""
        self.move(self.stop_speed, self.stop_speed)

    def follow_target(self, distance_to_target):
        """Menentukan gerakan robot berdasarkan jarak target"""
        # PROTOKOL KEAMANAN: Jika target terlalu dekat (kurang dari 30 cm), berhenti
        if distance_to_target < self.min_follow_distance:
            print(f"PROTOKOL KEAMANAN: Target berada dalam jarak {distance_to_target:.2f} cm (< {self.min_follow_distance} cm)")
            print("Robot berhenti untuk keamanan")
            self.stop()
            return
        
        # Cek jika robot sudah mencapai jarak target
        if abs(distance_to_target - self.target_distance) <= self.distance_threshold:
            print(f"TARGET TERCAPAI: Robot berada pada jarak {distance_to_target:.2f} cm dari target")
            print(f"(Jarak target: {self.target_distance} Â± {self.distance_threshold} cm)")
            self.stop()
            return
        
        # Hitung faktor kecepatan berdasarkan jarak
        # Makin jauh target, makin cepat robot bergerak
        distance_factor = min(1.0, (distance_to_target - self.min_follow_distance) / 
                            (self.max_follow_distance - self.min_follow_distance))
        adjusted_speed = self.base_speed * distance_factor
        
        # Bergerak maju
        self.move(-adjusted_speed, adjusted_speed)
        print(f"Bergerak maju dengan kecepatan: {adjusted_speed:.2f}")

    
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
                    
                    # Hitung jarak ke target dari A0 (pusat robot)
                    distance_to_target = corrected_distances['A0']
                    
                    # Kontrol robot berdasarkan data tracking
                    robot.follow_target(distance_to_target)
                    
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
