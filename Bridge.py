import serial  
import math  
import numpy as np  
from scipy.optimize import minimize  
import logging  

class UWBPositioning:  
    def __init__(self, port="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",   
                 baudrate=115200,   
                 timeout=1):  
        # Konfigurasi robot  
        self.L = 65  # Panjang robot  
        self.W = 35  # Lebar robot  

        # Konfigurasi anchor  
        self.anchors = {  
            "A0": (-self.L / 2, self.W / 2),   # Depan kiri  
            "A1": (-self.L / 2, -self.W / 2),  # Depan kanan  
            "A2": (self.L / 2, 0),             # Belakang tengah  
        }  

        # Konfigurasi serial  
        self.port = port  
        self.baudrate = baudrate  
        self.timeout = timeout  

        # Setup logging  
        logging.basicConfig(level=logging.INFO,   
                            format='%(asctime)s - %(levelname)s: %(message)s')  
        self.logger = logging.getLogger(__name__)  

    def method_weighted_centroid(self, distances):  
        """  
        Metode 1: Weighted Centroid  
        """  
        weights = []  
        x_tag, y_tag = 0, 0  
        total_weight = 0  

        for anchor, dist in zip(self.anchors.values(), distances):  
            x, y = anchor  
            weight = 1 / (dist + 0.001)  # Hindari pembagian nol  
            weights.append(weight)  
            x_tag += x * weight  
            y_tag += y * weight  
            total_weight += weight  

        x_tag /= total_weight  
        y_tag /= total_weight  

        return x_tag, y_tag  

    def method_trilateration(self, distances):  
        """  
        Metode 2: Trilaterasi menggunakan optimasi  
        """  
        def objective_function(pos):  
            return sum(  
                (np.linalg.norm(pos - np.array(anchor)) - dist)**2   
                for anchor, dist in zip(list(self.anchors.values()), distances)  
            )  

        initial_guess = [0, 0]  
        result = minimize(objective_function, initial_guess, method='Nelder-Mead')  
        
        return result.x  

    def method_angle_calculation(self, x_tag, y_tag):  
        """  
        Metode perhitungan sudut dengan variasi  
        """  
        methods = {  
            'atan2': math.degrees(math.atan2(y_tag, x_tag)),  
            'law_of_cosines': self._angle_law_of_cosines(x_tag, y_tag),  
            'dot_product': self._angle_dot_product(x_tag, y_tag)  
        }  
        return methods  

    def _angle_law_of_cosines(self, x_tag, y_tag):  
        """  
        Sudut menggunakan hukum kosinus  
        """  
        distance = math.sqrt(x_tag**2 + y_tag**2)  
        if distance == 0:  
            return 0  
        cos_theta = x_tag / distance  
        return math.degrees(math.acos(cos_theta))  

    def _angle_dot_product(self, x_tag, y_tag):  
        """  
        Sudut menggunakan dot product  
        """  
        reference_vector = [1, 0]  # Vektor referensi sumbu x  
        current_vector = [x_tag, y_tag]  
        
        dot_product = np.dot(reference_vector, current_vector)  
        magnitude_product = np.linalg.norm(reference_vector) * np.linalg.norm(current_vector)  
        
        cos_theta = dot_product / magnitude_product  
        return math.degrees(math.acos(cos_theta))  

    def process_uwb_data(self, data):  
        """  
        Proses data UWB dengan berbagai metode  
        """  
        try:  
            parts = data.split(",")  
            if len(parts) < 4 or not parts[0].startswith("$KT0"):  
                self.logger.warning("Invalid data format")  
                return None  

            # Proses jarak  
            raw_values = parts[1:4]  
            processed_values = [  
                0.0 if value.lower() == "null" else float(value)   
                for value in raw_values  
            ]  
            
            # Konversi ke cm  
            distances = [val * 100 for val in processed_values]  

            # Metode weighted centroid  
            x_tag, y_tag = self.method_weighted_centroid(distances)  

            # Kalkulasi jarak dan sudut  
            distance = math.sqrt(x_tag**2 + y_tag**2)  
            angle_methods = self.method_angle_calculation(x_tag, y_tag)  

            return {  
                "raw_distances": distances,  
                "position": (x_tag, y_tag),  
                "distance": distance,  
                "angles": angle_methods  
            }  

        except Exception as e:  
            self.logger.error(f"Error processing data: {e}")  
            return None  

    def start_reading(self):  
        """  
        Membaca data serial  
        """  
        try:  
            with serial.Serial(self.port, self.baudrate, timeout=self.timeout) as ser:  
                self.logger.info("Serial connection established")  
                
                while True:  
                    if ser.in_waiting > 0:  
                        data = ser.readline().decode("utf-8").strip()  
                        result = self.process_uwb_data(data)  
                        
                        if result:  
                            print("\n--- UWB Position Data ---")  
                            print(f"Raw Distances: {result['raw_distances']}")  
                            print(f"Position (x,y): {result['position']}")  
                            print(f"Distance: {result['distance']:.2f} cm")  
                            print("Angle Methods:")  
                            for method, angle in result['angles'].items():  
                                print(f"- {method}: {angle:.2f}°")  

        except serial.SerialException as e:  
            self.logger.error(f"Serial connection error: {e}")  

def main():  
    uwb_tracker = UWBPositioning()  
    uwb_tracker.start_reading()  

if __name__ == "__main__":  
    main()
