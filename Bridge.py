import serial  
import math  
import numpy as np  
from scipy import optimize  
import logging  
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
        logging.basicConfig(level=logging.DEBUG,   
                            format='%(asctime)s - %(levelname)s: %(message)s')  
        self.logger = logging.getLogger(__name__)  
        
    def process_uwb_data(self, raw_data):  
        """  
        Proses data UWB dengan debugging tambahan  
        """  
        try:  
            # Debug: Print raw data untuk melihat format aktual  
            self.logger.debug(f"Raw data received: {raw_data}")  
            
            # Coba berbagai kemungkinan pemisah  
            separators = [',', ';', '\t', ' ']  
            
            for sep in separators:  
                parts = raw_data.split(sep)  
                
                # Debug: Print parts yang dihasilkan  
                self.logger.debug(f"Parts using separator '{sep}': {parts}")  
                
                # Cek format yang mungkin  
                if len(parts) >= 4 and ('$KT0' in parts[0] or 'KT0' in parts[0]):  
                    # Ekstrak jarak  
                    try:  
                        distances = []  
                        for value in parts[1:4]:  
                            # Bersihkan dari karakter non-numerik  
                            cleaned_value = ''.join(  
                                char for char in value   
                                if char.isdigit() or char in '.-'  
                            )  
                            
                            # Konversi ke float  
                            if cleaned_value:  
                                dist = float(cleaned_value) * 100  # Konversi ke cm  
                                distances.append(dist)  
                            else:  
                                distances.append(0.0)  
                        
                        # Debug: Print jarak yang diekstrak  
                        self.logger.debug(f"Extracted distances: {distances}")  
                        
                        return distances  
                    
                    except Exception as e:  
                        self.logger.error(f"Error parsing distances: {e}")  
            
            # Jika tidak ada format yang cocok  
            self.logger.warning(f"No valid format found for data: {raw_data}")  
            return None  
        
        except Exception as e:  
            self.logger.error(f"Unexpected error processing data: {e}")  
            return None  
    
    def start_uwb_reading(self):  
        """  
        Pembacaan serial dengan debugging komprehensif  
        """  
        try:  
            # Buka serial dengan mode debugging  
            ser = serial.Serial(  
                port=self.port,   
                baudrate=self.baudrate,   
                timeout=self.timeout  
            )  
            self.logger.info("UWB Serial connection established")  
            
            while True:  
                try:  
                    # Baca data mentah  
                    raw_data = ser.readline()  
                    
                    # Dekode dengan berbagai encoding  
                    encodings = ['utf-8', 'ascii', 'latin-1']  
                    decoded_data = None  
                    
                    for encoding in encodings:  
                        try:  
                            decoded_data = raw_data.decode(encoding).strip()  
                            break  
                        except UnicodeDecodeError:  
                            continue  
                    
                    if not decoded_data:  
                        self.logger.warning(f"Cannot decode data: {raw_data}")  
                        continue  
                    
                    # Proses data  
                    distances = self.process_uwb_data(decoded_data)  
                    
                    if distances:  
                        self.logger.info(f"Distances: {distances}")  
                    
                    time.sleep(0.1)  
                
                except Exception as e:  
                    self.logger.error(f"Error in reading cycle: {e}")  
                    time.sleep(1)  
        
        except serial.SerialException as e:  
            self.logger.error(f"Serial connection error: {e}")  
        finally:  
            if 'ser' in locals():  
                ser.close()  

def main():  
    uwb_tracker = UWBPositioning()  
    uwb_tracker.start_uwb_reading()  

if __name__ == "__main__":  
    main()
