import serial
import socket
import time
import json

# Serial Configuration
SERIAL_PORT = '/dev/ttyACM0'  # Change to match your device
BAUDRATE = 115200
TIMEOUT = 1

# UDP Configuration
UDP_IP = "192.168.1.100"  # IP address of the receiver (robot)
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def initialize_serial():
    """Initialize and return serial connection with retry logic"""
    max_attempts = 5
    for attempt in range(max_attempts):
        try:
            ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT)
            print(f"Connected to {SERIAL_PORT}")
            return ser
        except serial.SerialException as e:
            print(f"Attempt {attempt+1}/{max_attempts}: Failed to connect to {SERIAL_PORT}: {e}")
            if attempt < max_attempts - 1:
                print(f"Retrying in 2 seconds...")
                time.sleep(2)
    
    raise Exception(f"Failed to connect to {SERIAL_PORT} after {max_attempts} attempts")

def parse_position_data(data_line):
    """Parse position data from UWB device"""
    try:
        # Example data format: "POS,T0,1234.56,789.01,23.45"
        # Adjust parsing according to your actual data format
        parts = data_line.split(',')
        if parts[0] == "POS" and parts[1] == "T0":
            x = float(parts[2])
            y = float(parts[3])
            z = float(parts[4])
            return x, y, z
        return None
    except (IndexError, ValueError) as e:
        print(f"Error parsing data: {e}")
        return None

# Main execution
try:
    ser = initialize_serial()
    
    while True:
        try:
            if ser.in_waiting > 0:
                data_line = ser.readline().decode("utf-8").strip()
                position_data = parse_position_data(data_line)
                
                if position_data:
                    x, y, z = position_data
                    # Add timestamp for data freshness validation
                    timestamp = time.time()
                    # Format: timestamp, x, y, z
                    message = f"{timestamp},{x},{y},{z}"
                    
                    # Send data via UDP
                    try:
                        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))
                        print(f"Sent position: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
                    except socket.error as e:
                        print(f"UDP send error: {e}")
                        
        except serial.SerialException as e:
            print(f"Serial error: {e}")
            # Try to reconnect
            try:
                ser.close()
                print("Disconnected from serial port")
                time.sleep(1)
                ser = initialize_serial()
            except Exception as e:
                print(f"Failed to reconnect: {e}")
                time.sleep(5)
                
        except Exception as e:
            print(f"Unexpected error: {e}")
            time.sleep(1)
            
        # Small delay to prevent CPU overload
        time.sleep(0.01)
            
except KeyboardInterrupt:
    print("Program stopped by user")
    # Clean shutdown
    if 'ser' in locals() and ser.is_open:
        ser.close()
    sock.close()
