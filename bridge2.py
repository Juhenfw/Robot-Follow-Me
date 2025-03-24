#!/usr/bin/env python3
import socket
import json
import time
import math

# Configuration
ROBOT_IP = '192.168.101.113'  # Replace with the IP address of your robot's Raspberry Pi
PORT = 65432
SEND_INTERVAL = 0.1  # seconds between transmissions

# Create a TCP/IP socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

def send_uwb_data(A0, A1, A2):
    """
    Send UWB anchor distances to the robot
    
    Args:
        A0, A1, A2: Distances to the respective anchors in centimeters
    """
    try:
        # Create data packet with raw anchor distances
        data = {
            "A0": round(A0, 2),
            "A1": round(A1, 2),
            "A2": round(A2, 2),
            "timestamp": time.time()
        }
        
        # Convert to JSON string with newline delimiter
        json_data = json.dumps(data) + "\n"
        
        # Send data
        client_socket.sendall(json_data.encode('utf-8'))
        
        # Display locally what was sent
        print(f"A0 = {A0:.2f} cm | A1 = {A1:.2f} cm | A2 = {A2:.2f} cm")
        
    except Exception as e:
        print(f"Error sending data: {e}")
        raise

def main():
    try:
        # Connect to the robot's Raspberry Pi
        print(f"Connecting to {ROBOT_IP}:{PORT}...")
        client_socket.connect((ROBOT_IP, PORT))
        print("Connected!")
        
        while True:
            # Replace this with your actual UWB sensor readings
            A0 = float(input("Enter A0 distance (cm): "))
            A1 = float(input("Enter A1 distance (cm): "))
            A2 = float(input("Enter A2 distance (cm): "))
            
            # Send the UWB data to the robot
            send_uwb_data(A0, A1, A2)
            
            # Optional: add a delay between readings
            # time.sleep(SEND_INTERVAL)
            
    except KeyboardInterrupt:
        print("\nStopping sender...")
    except ConnectionRefusedError:
        print(f"Connection refused. Is the receiver running at {ROBOT_IP}:{PORT}?")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        client_socket.close()
        print("Sender closed")

if __name__ == "__main__":
    main()
