import socket
import numpy as np
from scipy.optimize import minimize
import math

class UWBTracker:
    def __init__(self, fixed_anchor_positions, initial_bias=None):
        """ Inisialisasi tracker UWB dengan posisi anchor dan manajemen bias """
        self.A1_pos = np.array(fixed_anchor_positions['A1'])
        self.A2_pos = np.array(fixed_anchor_positions['A2'])
        self.A0_pos = np.array(fixed_anchor_positions['A0'])  # Posisi A0 juga tetap
        self.fixed_angle = math.radians(60)  # Sudut tetap antara anchor
        self.bias = initial_bias or {'A0': 0, 'A1': 0, 'A2': 0}
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

def main_uwb_receiver():
    # Inisialisasi socket
    UDP_IP = "192.168.1.2"  # IP Raspberry Pi 2 (di sini sebagai penerima)
    UDP_PORT = 5005         # Port untuk komunikasi
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    # Inisialisasi tracker dengan posisi anchor
    tracker = UWBTracker({
        'A0': [0.5, 0.5],  # Posisi A0 tetap
        'A1': [0, 1],      # Posisi A1 tetap
        'A2': [1, 0]       # Posisi A2 tetap
    })

    print("Menunggu data dari Raspberry Pi 1...")

    try:
        while True:
            # Terima data dari Raspberry Pi 1
            data, addr = sock.recvfrom(1024)
            print(f"Data diterima: {data.decode()}")
            
            # Pisahkan dan konversi data
            try:
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

                    # Koreksi bias
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

                    # Hitung sudut robot
                    robot_angle = tracker.calculate_robot_angle(
                        target_pos,
                        anchor_positions['A0']
                    )

                    # Hitung sudut tambahan menggunakan hukum kosinus
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
                    print("\nDiagnostik:")
                    print("Estimasi Posisi Target:", target_pos)
                    print("Status Optimasi:", diagnostics['optimization_success'])

            except Exception as e:
                print(f"Error memproses data: {e}")

    except KeyboardInterrupt:
        print("Receiver dihentikan oleh pengguna.")
    
    finally:
        # Tutup socket
        sock.close()
        print("Koneksi socket ditutup.")

if __name__ == "__main__":
    main_uwb_receiver()
