import socket
import time
import random

# Server configuration
HOST = '0.0.0.0'  # Listen on all interfaces
PORT = 5000       # Port to listen on

# Create socket server
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind((HOST, PORT))
server_socket.listen(1)

print(f"Sender running on {HOST}:{PORT}")
print("Waiting for receiver to connect...")

try:
    # Accept connection
    client_socket, client_address = server_socket.accept()
    print(f"Connected to receiver at {client_address}")
    
    while True:
        # Generate random sensor data
        A0 = random.uniform(0.5, 5.0)
        A1 = random.uniform(0.5, 5.0)
        A2 = random.uniform(0.5, 5.0)
        
        # Format data to match expected format
        data = f"$KT0,{A0:.2f},{A1:.2f},{A2:.2f}\n"
        
        # Send data
        client_socket.sendall(data.encode('utf-8'))
        print(f"Sent: {data.strip()}")
        
        # Wait before sending next data
        time.sleep(1)
        
except socket.error as e:
    print(f"Socket error: {e}")
except KeyboardInterrupt:
    print("Exiting program.")
finally:
    # Close connections
    if 'client_socket' in locals():
        client_socket.close()
    server_socket.close()
    print("All connections closed.")
