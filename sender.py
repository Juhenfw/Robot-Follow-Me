import socket
import time
import serial
import struct

# Serial configuration for UWB sensor
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
TIMEOUT = 1

# Server configuration
HOST = '0.0.0.0'  # Listen on all interfaces
PORT = 5000       # Port to listen on
RECONNECT_TIMEOUT = 5  # Seconds to wait before accepting new connection

def create_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    server_socket.settimeout(0.1)  # Non-blocking socket with short timeout
    return server_socket

print(f"Sender running on {HOST}:{PORT}")
print(f"Opening UWB sensor on {SERIAL_PORT}")

try:
    # Open serial connection to UWB sensor
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT)
    print(f"Connected to UWB sensor at {SERIAL_PORT} at {BAUDRATE} baud.")
    
    # Create TCP server
    server_socket = create_server()
    client_socket = None
    
    while True:
        # Try to accept connection if not connected
        if client_socket is None:
            try:
                print("Waiting for receiver to connect...")
                client_socket, client_address = server_socket.accept()
                print(f"Connected to receiver at {client_address}")
            except socket.timeout:
                # No connection request yet, continue reading from sensor
                pass
        
        # Read data from UWB sensor
        if ser.in_waiting > 0:
            data = ser.readline().decode("utf-8").strip()
            
            if data.startswith("$KT0"):
                print(f"Read from sensor: {data}")
                
                # If client is connected, send the data
                if client_socket:
                    try:
                        # Add newline for complete message framing
                        message = data + "\n"
                        message_bytes = message.encode('utf-8')
                        
                        # Send message length first, then the message
                        message_length = len(message_bytes)
                        client_socket.sendall(struct.pack('!I', message_length))
                        client_socket.sendall(message_bytes)
                        print(f"Sent to receiver: {data}")
                        
                    except (socket.error, BrokenPipeError) as e:
                        print(f"Connection error: {e}")
                        client_socket.close()
                        client_socket = None
        
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
