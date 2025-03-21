import serial  
import logging  
import sys  
import traceback  

class UWBDebugger:  
    def __init__(self, port="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",   
                 baudrate=115200,   
                 timeout=1):  
        # Setup logging  
        logging.basicConfig(  
            level=logging.DEBUG,   
            format='%(asctime)s - %(levelname)s: %(message)s',  
            handlers=[  
                logging.StreamHandler(sys.stdout),  
                logging.FileHandler('uwb_debug.log')  
            ]  
        )  
        self.logger = logging.getLogger(__name__)  
        
        # Konfigurasi serial  
        self.port = port  
        self.baudrate = baudrate  
        self.timeout = timeout  
    
    def raw_serial_capture(self):  
        """  
        Capture dan logging data mentah dari serial  
        """  
        try:  
            # Coba berbagai encoding  
            encodings = ['utf-8', 'ascii', 'latin-1', 'utf-16']  
            
            with serial.Serial(self.port, self.baudrate, timeout=self.timeout) as ser:  
                self.logger.info(f"Serial connection established on {self.port}")  
                
                while True:  
                    try:  
                        # Baca data mentah  
                        raw_data = ser.readline()  
                        
                        # Log bytes mentah  
                        self.logger.debug(f"Raw bytes: {raw_data}")  
                        
                        # Coba dekode dengan berbagai encoding  
                        for encoding in encodings:  
                            try:  
                                decoded_data = raw_data.decode(encoding).strip()  
                                
                                # Log setiap hasil dekode  
                                self.logger.info(f"Decoded ({encoding}): {decoded_data}")  
                                
                                # Analisis detail  
                                self.analyze_data(decoded_data)  
                                
                                break  
                            except UnicodeDecodeError:  
                                continue  
                        
                    except Exception as e:  
                        self.logger.error(f"Error reading serial: {e}")  
                        traceback.print_exc()  
        
        except serial.SerialException as e:  
            self.logger.error(f"Serial connection error: {e}")  
            traceback.print_exc()  
    
    def analyze_data(self, data):  
        """  
        Analisis mendetail terhadap data yang diterima  
        """  
        try:  
            # Cek panjang data  
            self.logger.debug(f"Data length: {len(data)}")  
            
            # Cek karakter yang ada  
            char_types = {  
                'digits': sum(c.isdigit() for c in data),  
                'letters': sum(c.isalpha() for c in data),  
                'special': sum(not c.isalnum() for c in data)  
            }  
            self.logger.debug(f"Character types: {char_types}")  
            
            # Coba berbagai pemisah  
            separators = [',', ';', '\t', ' ']  
            for sep in separators:  
                parts = data.split(sep)  
                self.logger.debug(f"Split by '{sep}': {parts}")  
        
        except Exception as e:  
            self.logger.error(f"Analysis error: {e}")  
            traceback.print_exc()  

def main():  
    debugger = UWBDebugger()  
    debugger.raw_serial_capture()  

if __name__ == "__main__":  
    main()
