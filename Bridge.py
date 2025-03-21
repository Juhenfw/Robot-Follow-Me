import serial  
import math  
import numpy as np  
from scipy import optimize  
import logging  
import threading  
import time  

class UWBPositioning:  
    def __init__(self, port="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",   
                 baudrate=115200,   
                 timeout=1):  
        # Konfigurasi parameter robot  
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
        
        # Variabel untuk filter dan validasi  
        self.distance_history = []  
        self.max_history_length = 5  
        
    def trilateration(self, distances):  
        """  
        Metode trilaterasi dengan optimasi untuk posisi tag  
        """  
        def objective_function(pos):  
            return sum((np.linalg.norm(pos - np.array(anchor)) - dist)**2   
                       for anchor, dist in zip(list(self.anchors.values()), distances))  
        
        initial_guess = [0, 0]  
        result = optimize.minimize(objective_function, initial_guess, method='Nelder-Mead')  
        
        return result.x  
    
    def kalman_filter(self, new_distance):  
        """  
        Sederhana Kalman filter untuk stabilisasi pembacaan jarak  
        """  
        if len(self.distance_history) < self.max_history_length:  
            self.distance_history.append(new_distance)  
            return new_distance  
        
        # Rata-rata bergerak  
        self.distance_history.pop(0)  
        self.distance_history.append(new_distance)  
        return np.mean(self.distance_history)  
    
    def process_uwb_data(self, raw_data):  
        """  
        Proses data UWB dengan validasi dan filter  
        """  
        try:  
            parts = raw_data.split(",")  
            if len(parts) < 4 or not parts[0].startswith("$KT0"):  
                self.logger.warning("Invalid data format")  
                return None  
            
            # Konversi dan filter data  
            distances = []  
            for value in parts[1:4]:  
                if value.lower() == "null":  
                    distances.append(0.0)  
                else:  
                    dist = float(value) * 100  # Konversi ke cm  
                    distances.append(self.kalman_filter(dist))  
            
            # Trilaterasi  
            x_tag, y_tag = self.trilateration(distances)  
            
            # Perhitungan jarak dan sudut  
            distance = np.linalg.norm([x_tag, y_tag])  
            angle = np.degrees(np.arctan2(y_tag, x_tag))  
            
            return {  
                "distances": distances,  
                "position": (x_tag, y_tag),  
                "distance": distance,  
                "angle": angle  
            }  
        
        except Exception as e:  
            self.logger.error(f"Error processing data: {e}")  
            return None  
    
    def start_uwb_reading(self):  
        """  
        Pembacaan serial dengan thread terpisah  
        """  
        try:  
            with serial.Serial(self.port, self.baudrate, timeout=self.timeout) as ser:  
                self.logger.info("UWB Serial connection established")  
                
                while True:  
                    if ser.in_waiting > 0:  
                        data = ser.readline().decode("utf-8").strip()  
                        result = self.process_uwb_data(data)  
                        
                        if result:  
                            self.logger.info(  
                                f"A0: {result['distances'][0]:.2f} cm | "  
                                f"A1: {result['distances'][1]:.2f} cm | "  
                                f"A2: {result['distances'][2]:.2f} cm | "  
                                f"Distance: {result['distance']:.2f} cm | "  
                                f"Angle: {result['angle']:.2f}°"  
                            )  
                        
                        time.sleep(0.1)  # Menghindari pembacaan berlebihan  
        
        except serial.SerialException as e:  
            self.logger.error(f"Serial connection error: {e}")  
        
def main():  
    uwb_tracker = UWBPositioning()  
    uwb_tracker.start_uwb_reading()  

if __name__ == "__main__":  
    main()
