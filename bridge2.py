#!/usr/bin/env python3
import socket
import json
import time
import math
import random  # For simulation - replace with actual UWB data reading

# Configuration
ROBOT_IP = '192.168.101.113'  # Replace with the IP address of your robot's Raspberry Pi
PORT = 65432
SEND_INTERVAL = 0.1  # seconds between transmissions

# Function to simulate UWB data (replace with your actual UWB sensor code)
def get_uwb_data():
    # This is just a simulation - implement your actual UWB data acquisition here
    # For example, reading from serial port connected to UWB module
    
    # Simulated data - random values around a point
    distance = 10 + random.uniform(-2, 2)
    angle = 170 + random.uniform(-10, 10)
    
    return distance, angle

def main():
    # Create a TCP/IP socket
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    try:
        # Connect to the robot's Raspberry Pi
        print(f"Connecting to {ROBOT_IP}:{PORT}...")
        client_socket.connect((ROBOT_IP, PORT))
        print("Connected!")
        
        while True:
            # Get UWB data
            distance, angle = get_uwb_data()
            
            # Create JSON data packet with newline as delimiter
            data = {
                "distance": round(distance, 2),
                "angle": round(angle, 2),
                "timestamp": time.time()
            }
            
            # Convert to JSON string with newline delimiter
            json_data = json.dumps(data) + "\n"
            
            # Send data
            client_socket.sendall(json_data.encode('utf-8'))
            print(f"Sent: Distance={data['distance']}cm, Angle={data['angle']}°")
            
            # Wait before sending next data
            time.sleep(SEND_INTERVAL)
            
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
