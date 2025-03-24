#!/usr/bin/env python3

import serial
import time
import signal
import sys
from threading import Thread

class HaoruTechReceiver:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        """
        Initialize receiver connection with HR-RTLS1 on Raspberry Pi
        
        Args:
            port: Serial port connected to HR-RTLS1 device
            baudrate: Communication speed (default 115200, check manual)
        """
        self.ser = None
        self.port = port
        self.baudrate = baudrate
        self.running = False
        self.rx_thread = None
    
    def connect(self):
        """Establish serial connection with the HR-RTLS1"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1
            )
            if self.ser.is_open:
                print(f"Receiver connected to port {self.port}")
                return True
            else:
                print(f"Failed to open serial port {self.port}")
                return False
        except Exception as e:
            print(f"Connection Error: {str(e)}")
            return False
    
    def disconnect(self):
        """Close the serial connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Receiver disconnected")
    
    def start_receiving(self):
        """Start receiving data from the HR-RTLS1"""
        if not self.ser or not self.ser.is_open:
            print("Error: Device not connected")
            return
        
        print("Starting data reception...")
        self.running = True
        self.rx_thread = Thread(target=self._receive_data)
        self.rx_thread.start()
    
    def stop_receiving(self):
        """Stop receiving data"""
        print("Stopping data reception...")
        self.running = False
        if self.rx_thread and self.rx_thread.is_alive():
            self.rx_thread.join()

    def _receive_data(self):
        """Continuously read data from the HR-RTLS1 and process it"""
        while self.running:
            try:
                if self.ser.in_waiting > 0:
                    # Read data from serial port
                    raw_data = self.ser.readline().decode('utf-8').strip()
                    
                    # Process the received data
                    self.process_data(raw_data)
                else:
                    time.sleep(0.1)
            except Exception as e:
                print(f"Error receiving data: {str(e)}")
    
    def process_data(self, data):
        """
        Process incoming data from HR-RTLS1
        
        Args:
            data: Raw string data received from HR-RTLS1
        """
        try:
            # Assume data is in JSON format (check documentation for actual format)
            print(f"Received Data: {data}")
            
            # Parse the data to extract positioning, e.g., coordinates
            # Adjust this based on the actual data structure received
            if "POS" in data:
                # Example of parsing data with "POS:x,y,z" format
                parts = data.split(":")[1].split(",")
                if len(parts) == 3:
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    print(f"Position: x={x:.2f}m, y={y:.2f}m, z={z:.2f}m")
            
        except Exception as e:
            print(f"Error processing data: {str(e)}. Raw Data: {data}")

def main():
    receiver = HaoruTechReceiver(port="/dev/ttyUSB0", baudrate=115200)

    try:
        # Connect the receiver
        if not receiver.connect():
            print("Failed to connect to HR-RTLS1 receiver!")
            return
        
        # Start receiving data
        receiver.start_receiving()
        
        # Prevent program exit
        print("Receiving data. Press Ctrl+C to stop.")
        while True:
            time.sleep(1)
    
    except KeyboardInterrupt:
        print("\nStopping receiver...")
    finally:
        receiver.stop_receiving()
        receiver.disconnect()

if __name__ == "__main__":
    # Handle signal for clean exit
    def signal_handler(sig, frame):
        print("\nCtrl+C detected. Exiting...")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    main()
