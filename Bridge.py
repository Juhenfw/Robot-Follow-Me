import numpy as np  
import math  
import serial  
import time  
from scipy.optimize import minimize  

class UWBTracker:  
    def __init__(self, fixed_anchor_positions, initial_bias=None):  
        """  
        Inisialisasi tracker UWB dengan posisi anchor dan manajemen bias  
        
        Args:  
        fixed_anchor_positions: Koordinat anchor A1, A2 yang sudah diketahui  
        initial_bias: Bias awal untuk setiap anchor  
        """  
        # Posisi anchor tetap  
        self.A1_pos = np.array(fixed_anchor_positions['A1'])  
        self.A2_pos = np.array(fixed_anchor_positions['A2'])  
        
        # Sudut tetap antara anchor  
        self.fixed_angle = math.radians(60)  
        
        # Manajemen bias  
        self.bias = initial_bias or {  
            'A0': 0,  # Bias untuk A0  
            'A1': 0,  # Bias untuk A1  
            'A2': 0   # Bias untuk A2  
        }  
        
        # Parameter kalibrasi tambahan  
        self.scale_factor = {  
            'A0': 1.0,  
            'A1': 1.0,  
            'A2': 1.0  
        }  
    
    def apply_bias_correction(self, distances):  
        """  
        Koreksi bias dan scaling pada pengukuran jarak  
        
        Args:  
        distances: Dictionary jarak mentah dari setiap anchor  
        
        Returns:  
        Dictionary jarak terkoreksi dalam cm  
        """  
        corrected_distances = {  
            'A0': max((distances['A0'] * 100 * self.scale_factor['A0']) - self.bias['A0'], 0),  
            'A1': max((distances['A1'] * 100 * self.scale_factor['A1']) - self.bias['A1'], 0),  
            'A2': max((distances['A2'] * 100 * self.scale_factor['A2']) - self.bias['A2'], 0)  
        }  
        return corrected_distances  
    
    def trilateration_cost(self, target_pos, corrected_distances, anchor_positions):  
        """  
        Fungsi biaya untuk estimasi posisi target dengan pertimbangan geometri  
        
        Args:  
        target_pos: Posisi target yang dicoba  
        corrected_distances: Jarak terkoreksi dari setiap anchor  
        anchor_positions: Posisi anchor  
        
        Returns:  
        Total error kuadrat  
        """  
        # Hitung jarak dari target ke setiap anchor  
        calculated_distances = [  
            np.linalg.norm(target_pos - anchor_positions['A1']),  
            np.linalg.norm(target_pos - anchor_positions['A2']),  
            np.linalg.norm(target_pos - anchor_positions['A0'])  
        ]  
        
        # Hitung error kuadrat dengan bobot  
        distance_errors = [  
            ((calc - measured) ** 2)   
            for calc, measured in zip(  
                calculated_distances,   
                [corrected_distances['A1'], corrected_distances['A2'], corrected_distances['A0']]  
            )  
        ]  
        
        return np.sum(distance_errors)  
    
    def estimate_target_position(self, distances, anchor_positions):  
        """  
        Estimasi posisi target menggunakan optimasi trilateration  
        
        Args:  
        distances: Dictionary jarak dari setiap anchor  
        anchor_positions: Posisi anchor  
        
        Returns:  
        Estimasi posisi target dan informasi diagnostik  
        """  
        # Terapkan koreksi bias  
        corrected_distances = self.apply_bias_correction(distances)  
        
        # Persiapan initial guess  
        initial_guess = np.mean([  
            anchor_positions['A1'],   
            anchor_positions['A2'],   
            anchor_positions['A0']  
        ], axis=0)  
        
        # Optimasi untuk menemukan posisi target  
        result = minimize(  
            self.trilateration_cost,   
            initial_guess,   
            args=(corrected_distances, anchor_positions),  
            method='Nelder-Mead',  
            options={'maxiter': 100}  
        )  
        
        # Simpan informasi diagnostik  
        diagnostics = {  
            'raw_distances': distances,  
            'corrected_distances': corrected_distances,  
            'optimization_success': result.success,  
            'optimization_message': result.message  
        }  
        
        return result.x, diagnostics  
    
    def calculate_angle(self, a, b, c):  
        """  
        Hitung sudut menggunakan hukum kosinus  
        
        Args:  
        a, b, c: Panjang sisi-sisi segitiga  
        
        Returns:  
        Sudut dalam derajat  
        """  
        try:  
            # Pastikan tidak ada pembagian dengan nol  
            cos_angle = (a**2 + b**2 - c**2) / (2 * a * b)  
            
            # Pastikan nilai cosinus valid  
            cos_angle = max(min(cos_angle, 1), -1)  
            
            # Hitung sudut dalam derajat  
            angle = math.degrees(math.acos(cos_angle))  
            
            return angle  
        except Exception as e:  
            print(f"Error calculating angle: {e}")  
            return None  
    
    def calculate_robot_angle(self, target_pos, reference_pos):  
        """  
        Hitung sudut robot untuk mengejar target  
        
        Args:  
        target_pos: Posisi target  
        reference_pos: Posisi referensi (biasanya posisi A0)  
        
        Returns:  
        Sudut robot dalam derajat  
        """  
        # Vektor dari referensi ke target  
        vector_to_target = target_pos - reference_pos  
        
        # Gunakan arctangent2 untuk sudut absolut  
        angle = math.atan2(vector_to_target[1], vector_to_target[0])  
        
        return math.degrees(angle)  
    
    def print_tracking_info(self, corrected_distances, robot_angle=None, additional_angle=None):  
        """  
        Cetak informasi tracking dalam format yang diinginkan  
        
        Args:  
        corrected_distances: Jarak terkoreksi dari setiap anchor  
        robot_angle: Sudut robot (opsional)  
        additional_angle: Sudut tambahan (opsional)  
        """  
        print(  
            f"\nA0 = {corrected_distances['A0']:.2f} cm | "  
            f"A1 = {corrected_distances['A1']:.2f} cm | "  
            f"A2 = {corrected_distances['A2']:.2f} cm"  
        )  
        
        # Tambahkan output sudut jika tersedia  
        if robot_angle is not None:  
            print(f"Sudut Robot: {robot_angle:.2f} derajat")  
        
        if additional_angle is not None:  
            print(f"Sudut Tambahan: {additional_angle:.2f} derajat")  

def main_uwb_tracking():  
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
    
    # Inisialisasi tracker dengan posisi anchor  
    tracker = UWBTracker({  
        'A1': [0, 1],    # Koordinat A1   
        'A2': [1, 0]     # Koordinat A2  
    })  
    
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
                            
                            # Simulasi posisi anchor bergerak  
                            anchor_positions = {  
                                'A0': np.array([0.5, 0.5]),  # Posisi A0 yang bergerak  
                                'A1': np.array([0, 1]),      # Posisi A1 tetap  
                                'A2': np.array([1, 0])       # Posisi A2 tetap  
                            }  
                            
                            # Siapkan data jarak  
                            uwb_data = {  
                                'A0': raw_values[0],  
                                'A1': raw_values[1],  
                                'A2': raw_values[2]  
                            }  
                            
                            # Koreksi bias  
                            corrected_distances = tracker.apply_bias_correction(uwb_data)  
                            
                            # Estimasi posisi target  
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
                                corrected_distances['A0'],  # a  
                                corrected_distances['A1'],  # b  
                                corrected_distances['A2']   # c  
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
                        
                        # Jeda untuk mencegah overload  
                        time.sleep(0.5)  
                    
                    except Exception as e:  
                        print(f"Error memproses data: {e}")  
    
    except KeyboardInterrupt:  
        print("Tracking dihentikan oleh pengguna.")  
    
    finally:  
        # Tutup koneksi serial  
        if 'ser' in locals() and ser.is_open:  
            ser.close()  
            print("Koneksi serial ditutup.")  

# Jalankan tracking UWB  
if __name__ == "__main__":  
    main_uwb_tracking()  
