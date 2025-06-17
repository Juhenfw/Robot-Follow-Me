# SENDER

import socket
import time
import serial
import traceback

# UDP Configuration
UDP_IP = "192.168.99.113"  # Change to the IP address of the receiver Raspberry Pi
UDP_PORT = 5005
print(f"Configured to send to {UDP_IP}:{UDP_PORT}")

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Serial port configuration for UWB sensor
SERIAL_PORT = "/dev/ttyUSB0"  # Change to your actual UWB sensor port
BAUD_RATE = 115200

def initialize_serial():
    """Initialize and return serial connection to UWB sensor"""
    print(f"Attempting to connect to UWB sensor on {SERIAL_PORT}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print("Successfully connected to UWB sensor!")
        return ser
    except serial.SerialException as e:
        print(f"Failed to connect to UWB sensor: {e}")
        return None

def parse_position_data(data_line):
    """Parse position data from UWB sensor output"""
    try:
        # Modify this parser according to your UWB sensor's actual data format
        # Example format: "POS: x=1.23, y=4.56, z=7.89"
        if "POS:" in data_line:
            parts = data_line.split("=")
            x = float(parts[1].split(",")[0])
            y = float(parts[2].split(",")[0])
            z = float(parts[3])
            return (x, y, z)
    except Exception as e:
        print(f"Error parsing position data: {e}")
    return None

# Main program
print("UWB Data Sender Starting...")
serial_reconnect_attempts = 0
MAX_RECONNECT_ATTEMPTS = 10

try:
    ser = initialize_serial()
    
    while True:
        # Check if serial is connected
        if ser is None or not ser.is_open:
            serial_reconnect_attempts += 1
            print(f"Serial not connected. Attempt {serial_reconnect_attempts}/{MAX_RECONNECT_ATTEMPTS}")
            
            if serial_reconnect_attempts > MAX_RECONNECT_ATTEMPTS:
                print("Max reconnection attempts reached. Waiting 30 seconds before retrying...")
                time.sleep(30)
                serial_reconnect_attempts = 0
                
            ser = initialize_serial()
            time.sleep(1)
            continue
            
        # Reset counter when connected
        serial_reconnect_attempts = 0
            
        try:
            # Read data from UWB sensor
            if ser.in_waiting > 0:
                data_line = ser.readline().decode("utf-8").strip()
                print(f"Raw data: {data_line}")
                
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
            # Close the serial connection to force reconnect in next iteration
            try:
                ser.close()
                print("Disconnected from serial port")
            except:
                pass
            ser = None
            time.sleep(1)
                
        except Exception as e:
            print(f"Unexpected error: {e}")
            traceback.print_exc()
            time.sleep(1)
            
        # Small delay to prevent CPU overload
        time.sleep(0.01)
            
except KeyboardInterrupt:
    print("\nProgram stopped by user")
    # Clean shutdown
    if 'ser' in locals() and ser is not None and ser.is_open:
        ser.close()
    sock.close()
except Exception as e:
    print(f"Fatal error: {e}")
    traceback.print_exc()
finally:
    print("Sender script terminated")
