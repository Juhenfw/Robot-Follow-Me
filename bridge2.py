#!/usr/bin/env python3

import serial
import time
import json
import signal
import sys
from threading import Thread

class HaoruTechRTLS:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        """
        Initialize connection to HR-RTLS1 device
        
        Args:
            port: Serial port where the device is connected
            baudrate: Communication speed (check manual for correct setting)
        """
        self.ser = None
        self.port = port
        self.baudrate = baudrate
        self.running = False
        self.data_thread = None
        
    def connect(self):
        """Establish connection to the HR-RTLS1 device"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1
            )
            
            if self.ser.isOpen():
                print(f"Connected to HR-RTLS1 on {self.port}")
                return True
            else:
                print(f"Failed to open connection on {self.port}")
                return False
                
        except Exception as e:
            print(f"Connection error: {str(e)}")
            return False
    
    def disconnect(self):
        """Close connection to device"""
        if self.ser and self.ser.isOpen():
            self.ser.close()
            print("Disconnected from HR-RTLS1")
    
    def send_command(self, command):
        """
        Send command to the device
        
        Args:
            command: Command string according to HR-RTLS1 protocol
        """
        if not self.ser or not self.ser.isOpen():
            print("Error: Device not connected")
            return False
            
        try:
            # Add termination character if needed (check manual)
            if not command.endswith('\r\n'):
                command += '\r\n'
                
            self.ser.write(command.encode())
            return True
        except Exception as e:
            print(f"Error sending command: {str(e)}")
            return False
    
    def read_response(self, timeout=1.0):
        """
        Read response from the device
        
        Args:
            timeout: Maximum time to wait for response in seconds
        
        Returns:
            Response string or None if timeout
        """
        if not self.ser or not self.ser.isOpen():
            print("Error: Device not connected")
            return None
            
        try:
            # Set timeout for this read operation
            self.ser.timeout = timeout
            
            # Read until encounter newline
            response = self.ser.readline().decode().strip()
            return response
        except Exception as e:
            print(f"Error reading response: {str(e)}")
            return None
    
    def configure_system(self):
        """Configure the HR-RTLS1 system with initial settings"""
        # Send configuration commands according to manual
        # Example (modify based on actual commands from manual):
        self.send_command("AT+CONFIG")
        response = self.read_response()
        print(f"Config response: {response}")
        
        # Set mode to tag or anchor as needed
        self.send_command("AT+MODE=TAG")  # or ANCHOR
        response = self.read_response()
        print(f"Mode response: {response}")
        
        # Additional configuration commands
        # ...
    
    def start_ranging(self):
        """Start the ranging/positioning process"""
        self.send_command("AT+START")
        response = self.read_response()
        print(f"Start ranging response: {response}")
        
        # Start background thread to continuously receive positioning data
        self.running = True
        self.data_thread = Thread(target=self._data_receiver)
        self.data_thread.daemon = True
        self.data_thread.start()
    
    def stop_ranging(self):
        """Stop the ranging/positioning process"""
        self.running = False
        if self.data_thread:
            self.data_thread.join(timeout=2.0)
            
        self.send_command("AT+STOP")
        response = self.read_response()
        print(f"Stop ranging response: {response}")
    
    def _data_receiver(self):
        """Background thread to continuously receive positioning data"""
        while self.running:
            try:
                data = self.read_response(timeout=0.1)
                if data:
                    self._process_data(data)
            except Exception as e:
                print(f"Error in data receiver: {str(e)}")
                time.sleep(0.1)
    
    def _process_data(self, data):
        """
        Process received positioning data
        
        Args:
            data: Data string received from device
        """
        try:
            # Attempt to parse as JSON if the system outputs JSON format
            # Modify according to actual data format from HR-RTLS1
            try:
                json_data = json.loads(data)
                print(f"Position: x={json_data.get('x', 0):.2f}m, y={json_data.get('y', 0):.2f}m, z={json_data.get('z', 0):.2f}m")
            except json.JSONDecodeError:
                # If not JSON, process according to actual format
                if data.startswith("POS:"):
                    # Example format: "POS:1.23,4.56,0.78"
                    parts = data.split(":")[1].split(",")
                    if len(parts) >= 3:
                        x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                        print(f"Position: x={x:.2f}m, y={y:.2f}m, z={z:.2f}m")
                else:
                    # Raw output
                    print(f"Data: {data}")
                    
        except Exception as e:
            print(f"Error processing data: {str(e)}")
            print(f"Raw data: {data}")

def main():
    # Create instance of HaoruTech RTLS handler
    rtls = HaoruTechRTLS(port="/dev/ttyUSB0", baudrate=115200)
    
    try:
        # Connect to device
        if not rtls.connect():
            print("Failed to connect to HR-RTLS1 device")
            return
        
        # Configure the system
        rtls.configure_system()
        
        # Start ranging/positioning
        rtls.start_ranging()
        
        # Keep main thread running
        print("System running. Press Ctrl+C to exit.")
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        # Clean shutdown
        rtls.stop_ranging()
        rtls.disconnect()
        print("System stopped")

if __name__ == "__main__":
    # Set up signal handler for clean exit
    def signal_handler(sig, frame):
        print("Ctrl+C pressed. Exiting...")
        sys.exit(0)
        
    signal.signal(signal.SIGINT, signal_handler)
    main()
