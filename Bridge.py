#!/usr/bin/env python3
import socket
import json
import time

# Configuration
HOST = ''  # Listen on all available interfaces
PORT = 65432

# Function to process UWB data and control robot
def process_uwb_data(distance, angle):
    """
    This function processes the UWB data and controls the robot
    Replace with your actual robot control code
    
    Args:
        distance: Distance in cm
        angle: Angle in degrees
    """
    print(f"Robot position: Distance={distance}cm, Angle={angle}°")
    
    # Add your robot control logic here
    # For example:
    if distance < 5:
        print("Action: Too close, backing up")
    elif angle > 180:
        print("Action: Turning right")
    elif angle < 160:
        print("Action: Turning left")
    else:
        print("Action: Moving forward")

def main():
    # Create a TCP/IP socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    # Bind to address and port
    server_socket.bind((HOST, PORT))
    
    # Listen for incoming connections
    server_socket.listen(1)
    print(f"UWB Receiver started on port {PORT}")
    print("Waiting for connection...")
    
    # Accept connection
    conn, addr = server_socket.accept()
    print(f"Connected to sender at {addr}")
    
    # Buffer for accumulating incoming data
    buffer = b''
    
    try:
        while True:
            # Receive data
            chunk = conn.recv(1024)
            
            # Check if connection is closed
            if not chunk:
                print("Connection closed by sender")
                break
                
            # Add received data to buffer
            buffer += chunk
            
            # Process complete messages (delimited by newlines)
            while b'\n' in buffer:
                # Split at first newline
                line, buffer = buffer.split(b'\n', 1)
                
                try:
                    # Decode and parse JSON
                    data = json.loads(line.decode('utf-8'))
                    
                    # Process the data
                    process_uwb_data(data['distance'], data['angle'])
                    
                except (UnicodeDecodeError, json.JSONDecodeError) as e:
                    print(f"Error processing data: {e}")
                
    except KeyboardInterrupt:
        print("\nStopping receiver...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Clean up
        if 'conn' in locals():
            conn.close()
        server_socket.close()
        print("Receiver closed")

if __name__ == "__main__":
    main()
