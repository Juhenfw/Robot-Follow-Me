import socket
import serial
import time
import numpy as np
from collections import deque
import threading
import netifaces  # Modul untuk mendapatkan IP dari jaringan Wi-Fi

def get_ip_from_wifi(interface='wlan0'):
    """Mendapatkan IP dari interface Wi-Fi"""
    try:
        ip = netifaces.ifaddresses(interface)[netifaces.AF_INET][0]['addr']
        return ip
    except (KeyError, ValueError):
        print(f"Failed to get IP address for interface: {interface}")
        return None
    
def get_ip_from_subnet(ip, target_last_digit):
    """Mengambil IP dengan perbedaan 3 digit terakhir"""
    ip_parts = ip.split(".")
    ip_parts[-1] = str(target_last_digit)  # Ganti digit terakhir dengan target_last_digit
    return ".".join(ip_parts)

class UWBTransmitter:
    def __init__(self, serial_port, udp_ip, udp_port):
        self.serial_port = serial_port
        self.udp_ip = udp_ip  # IP yang dinamis berdasarkan jaringan Wi-Fi
        self.udp_port = udp_port
        self.ser = None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Data storage, filters, validators, etc.
        # (same as before)

    def connect(self):
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=115200,
                timeout=0.1  # Reduced timeout for better responsiveness
            )
            print(f"Connected to {self.serial_port}")
            return True
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            return False
    
    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial connection closed.")
        self.sock.close()
        print("Socket connection closed.")

    def process_data(self, line):
        if not line.startswith("$KT0"):
            return
        try:
            parts = line.split(",")
            if len(parts) < 4:
                return
            # Parse raw values (same as before)
        except Exception as e:
            print(f"Error parsing data: {e}")
    
    def process_buffer(self):
        # Your existing processing logic
        # For simplicity, returning placeholder values
        return {'A0': 1.0, 'A1': 1.0, 'A2': 1.0}, {'A0': 1.0, 'A1': 1.0, 'A2': 1.0}, False
    
    def send_data(self, filtered_values):
        # Format values with 3 decimal places
        formatted_values = [f"{filtered_values[key]:.4f}" for key in ['A0', 'A1', 'A2']]
        uwb_data = ",".join(formatted_values)
        self.sock.sendto(uwb_data.encode(), (self.udp_ip, self.udp_port))
        return uwb_data

    def read_serial_thread(self):
        """Thread to continuously read from serial port"""
        buffer = ""
        while True:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting).decode("utf-8", errors="ignore")
                    buffer += data
                    lines = buffer.split('\n')
                    buffer = lines.pop()  # Keep incomplete line
                    for line in lines:
                        line = line.strip()
                        if line:
                            self.process_data(line)
                else:
                    time.sleep(0.01)
            except Exception as e:
                print(f"Error in serial thread: {e}")
                time.sleep(0.001)

    def process_thread(self):
        """Thread to process data and send at controlled rate"""
        while True:
            try:
                # Process data and send it at controlled rate
                filtered_values, raw_values, movement_detected = self.process_buffer()
                self.send_data(filtered_values)
                # For now, just simulate printing the data
                print(f"Filtered: {filtered_values} | Raw: {raw_values} | Movement Detected: {movement_detected}")
                time.sleep(0.1)  # Simulate data processing rate
            except Exception as e:
                print(f"Error in process thread: {e}")
                time.sleep(0.001)
    
    def run(self):
        """Start threads for reading and processing"""
        if not self.connect():
            return

        # Start reading thread
        read_thread = threading.Thread(target=self.read_serial_thread)
        read_thread.daemon = True
        read_thread.start()

        # Start processing thread
        process_thread = threading.Thread(target=self.process_thread)
        process_thread.daemon = True
        process_thread.start()

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("Transmitter stopped by user.")
        finally:
            self.disconnect()

def main_uwb_transmitter():
    # Configuration
    serial_port = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
    
    # Ambil IP Raspberry Pi yang terhubung ke Wi-Fi dan pastikan IP yang diambil adalah IP yang sesuai
    current_ip = get_ip_from_wifi()  # Mengambil IP dari Wi-Fi
    if current_ip:
        print(f"Raspberry Pi current IP: {current_ip}")
        
        # Mengambil tiga digit terakhir yang berbeda
        current_last_digit = int(current_ip.split('.')[-1])
        target_last_digit = 128  # Misalnya ingin mendapatkan IP yang berakhiran .128
        
        if current_last_digit != target_last_digit:
            udp_ip = get_ip_from_subnet(current_ip, target_last_digit)  # Sesuaikan IP
            print(f"Target IP with last digit {target_last_digit}: {udp_ip}")
        else:
            print(f"Raspberry Pi IP is already {target_last_digit}. No change needed.")
            udp_ip = current_ip  # Gunakan IP Raspberry Pi saat ini

        udp_port = 5005           # Port untuk komunikasi
        
        # Create and run transmitter
        transmitter = UWBTransmitter(serial_port, udp_ip, udp_port)
        transmitter.run()
    else:
        print("Unable to retrieve Raspberry Pi IP")

if __name__ == "__main__":
    main_uwb_transmitter()