#!/usr/bin/env python3
import socket
import json
import math

# Configuration
HOST = ''  # Listen on all available interfaces
PORT = 65432

# Robot dimensions and anchor positions (replace with your actual values)
L = 65  # Length of robot in cm
W = 35  # Width of robot in cm

# Anchor positions (replace with your actual anchor positions)
anchors = {
    "A0": (-L/2, W/2),   # Front left
    "A1": (-L/2, -W/2),  # Front right
    "A2": (L/2, 0)       # Back center
}

def calculate_trilateration(A0, A1, A2):
    """
    Calculate position using trilateration algorithm
    
    Args:
        A0, A1, A2: Distances to the respective anchors in centimeters
    
    Returns:
        distance: Distance from robot center to tag (cm)
        angle: Angle to tag (degrees)
    """
    weights = []
    x_tag, y_tag = 0, 0
    
    # Get anchor positions and distances
    distances = [A0, A1, A2]
    
    for i, (anchor_name, anchor_pos) in enumerate(anchors.items()):
        x, y = anchor_pos
        dist = distances[i]
        
        # Weight is inversely proportional to distance
        weight = 1 / dist if dist > 0 else 0
        weights.append(weight)
        
        # Weighted sum of positions
        x_tag += x * weight
        y_tag += y * weight
    
    # Calculate weighted average position
    total_weight = sum(weights)
    if total_weight > 0:
        x_tag /= total_weight
        y_tag /= total_weight
    
    # Calculate distance and angle from robot center to tag
    distance = math.sqrt(x_tag**2 + y_tag**2)
    angle = math.degrees(math.atan2(y_tag, x_tag))
    
    return distance, angle

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
                    
                    # Extract anchor distances
                    A0 = data['A0']
                    A1 = data['A1']
                    A2 = data['A2']
                    
                    # Calculate position using trilateration
                    distance, angle = calculate_trilateration(A0, A1, A2)
                    
                    # Print results - proper handling of degree symbol
                    print(f"A0 = {A0:.2f} centimeter | A1 = {A1:.2f} centimeter | "
                          f"A2 = {A2:.2f} centimeter | T0 to Robot = {distance:.2f} cm | "
                          f"Sudut = {angle:.2f}°")
                    
                    # Add your robot control logic here based on distance and angle
                    
                except (UnicodeDecodeError, json.JSONDecodeError) as e:
                    print(f"Error processing data: {e}")
                except KeyError as e:
                    print(f"Missing data field: {e}")
                
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
