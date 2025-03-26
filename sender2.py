import socket
import time
import serial
import struct
import numpy as np
from collections import deque

# Serial configuration for UWB sensor
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
TIMEOUT = 1

# Server configuration
HOST = '0.0.0.0'
PORT = 5000
RECONNECT_TIMEOUT = 5

# Stabilization parameters
WINDOW_SIZE = 5        # Number of samples for moving average
MAX_JUMP_THRESHOLD = 50  # Maximum allowable jump between readings (cm)
MIN_VALID_VALUE = 5    # Minimum valid value (cm)
MAX_VALID_VALUE = 1000  # Maximum valid value (cm)

class UWBStabilizer:
    def __init__(self, window_size=WINDOW_SIZE):
        # Create buffers for each channel
        self.buffers = [deque(maxlen=window_size) for _ in range(3)]
        self.last_valid_readings = [None, None, None]
        
    def add_reading(self, readings):
        stable_readings = []
        
        for i, reading in enumerate(readings):
            # Apply validity check
            if not self.is_valid_reading(reading, i):
                # Use last stable reading if current one is invalid
                if self.last_valid_readings[i] is not None:
                    reading = self.last_valid_readings[i]
                else:
                    # If no valid reading yet, use this one but mark as uncertain
                    self.last_valid_readings[i] = reading
                    self.buffers[i].append(reading)
                    stable_readings.append(reading)
                    continue
            
            # Reading passed validation
            self.last_valid_readings[i] = reading
            self.buffers[i].append(reading)
            
            # Calculate filtered reading
            if len(self.buffers[i]) > 0:
                # Use median filter to be robust against outliers
                sorted_values = sorted(self.buffers[i])
                if len(sorted_values) % 2 == 1:
                    stable_reading = sorted_values[len(sorted_values) // 2]
                else:
                    # Average the two middle values for even number of samples
                    mid1 = sorted_values[len(sorted_values) // 2 - 1]
                    mid2 = sorted_values[len(sorted_values) // 2]
                    stable_reading = (mid1 + mid2) / 2
            else:
                stable_reading = reading
                
            stable_readings.append(stable_reading)
            
        return stable_readings
    
    def is_valid_reading(self, reading, channel_idx):
        # Check for NaN or infinity
        if np.isnan(reading) or np.isinf(reading):
            return False
            
        # Check for range validity
        if reading < MIN_VALID_VALUE or reading > MAX_VALID_VALUE:
            return False
            
        # Check for sudden jumps (if we have previous readings)
        if self.last_valid_readings[channel_idx] is not None:
            jump = abs(reading - self.last_valid_readings[channel_idx])
            if jump > MAX_JUMP_THRESHOLD:
                return False
                
        return True

def create_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    server_socket.settimeout(0.1)
    return server_socket

def correct_bias(raw_distance, bias):
    return raw_distance - bias

# Initialize stabilizer
stabilizer = UWBStabilizer()

print(f"Sender running on {HOST}:{PORT}")
print(f"Opening UWB sensor on {SERIAL_PORT}")

try:
    # Try different baudrates if connection fails
    connected = False
    baudrates_to_try = [115200, 9600, 57600, 38400]
    
    for baud in baudrates_to_try:
        try:
            ser = serial.Serial(SERIAL_PORT, baud, timeout=TIMEOUT)
            print(f"Connected to UWB sensor at {SERIAL_PORT} at {baud} baud.")
            connected = True
            break
        except serial.SerialException as e:
            print(f"Failed with baudrate {baud}: {e}")
    
    if not connected:
        raise Exception("Could not connect to UWB sensor with any baudrate")
    
    # Create TCP server
    server_socket = create_server()
    client_socket = None
    
    # Buffer for incomplete serial data
    serial_buffer = ""
    
    # Flush initial data
    ser.flushInput()
    time.sleep(1)
    
    while True:
        # Try to accept connection if not connected
        if client_socket is None:
            try:
                print("Waiting for receiver to connect...")
                client_socket, client_address = server_socket.accept()
                print(f"Connected to receiver at {client_address}")
            except socket.timeout:
                pass
        
        # Read data from UWB sensor
        if ser.in_waiting > 0:
            try:
                byte_data = ser.readline()
                data = byte_data.decode("utf-8", errors='replace').strip()
                
                # Add to buffer and check for complete message
                serial_buffer += data
                
                # Look for complete message that starts with $KT0
                if "$KT0" in serial_buffer:
                    # Extract messages (there might be multiple)
                    lines = serial_buffer.split('\n')
                    
                    # Keep the last incomplete line in the buffer
                    serial_buffer = lines[-1] if not lines[-1].endswith('\n') else ""
                    
                    # Process complete messages
                    for line in lines[:-1] if not lines[-1].endswith('\n') else lines:
                        if line.startswith("$KT0"):
                            try:
                                parts = line.split(",")
                                if len(parts) >= 4:
                                    raw_values = parts[1:4]
                                    processed_values = []
                                    
                                    for i, value in enumerate(raw_values):
                                        if value.lower() == "null":
                                            processed_values.append(0.0)
                                        else:
                                            try:
                                                processed_values.append(float(value))
                                            except ValueError:
                                                processed_values.append(0.0)
                                    
                                    # Apply stabilization
                                    A0, A1, A2 = processed_values
                                    A0_cm, A1_cm, A2_cm = A0*100, A1*100, A2*100
                                    
                                    # Get stabilized readings
                                    stable_readings = stabilizer.add_reading([A0_cm, A1_cm, A2_cm])
                                    stable_A0, stable_A1, stable_A2 = stable_readings
                                    
                                    # Apply bias correction
                                    cal_A0 = correct_bias(stable_A0, 30)
                                    cal_A1 = correct_bias(stable_A1, 0)
                                    cal_A2 = correct_bias(stable_A2, 0)
                                    
                                    # Create message with stabilized data
                                    stabilized_message = f"$KT0,{cal_A0/100:.2f},{cal_A1/100:.2f},{cal_A2/100:.2f}"
                                    
                                    print(f"Raw: ${A0_cm:.2f},{A1_cm:.2f},{A2_cm:.2f} -> Stabilized: {cal_A0:.2f},{cal_A1:.2f},{cal_A2:.2f}")
                                    
                                    # If client is connected, send the data
                                    if client_socket:
                                        try:
                                            # Add newline for complete message framing
                                            message = stabilized_message + "\n"
                                            message_bytes = message.encode('utf-8')
                                            
                                            # Send message length first, then the message
                                            message_length = len(message_bytes)
                                            client_socket.sendall(struct.pack('!I', message_length))
                                            client_socket.sendall(message_bytes)
                                            
                                        except (socket.error, BrokenPipeError) as e:
                                            print(f"Connection error: {e}")
                                            client_socket.close()
                                            client_socket = None
                            except Exception as e:
                                print(f"Error processing data: {e}")
            except UnicodeDecodeError:
                print("Unicode decode error - skipping corrupted data")
                ser.flushInput()
                serial_buffer = ""
        
        # Small delay to prevent CPU hogging
        time.sleep(0.01)
        
except serial.SerialException as e:
    print(f"Serial error: {e}")
except KeyboardInterrupt:
    print("Exiting program.")
finally:
    # Close all connections
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial connection closed.")
    if 'client_socket' in locals() and client_socket:
        client_socket.close()
    if 'server_socket' in locals():
        server_socket.close()
    print("All connections closed.")

