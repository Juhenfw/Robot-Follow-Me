import socket
import struct
import time

# Configure connection
SENDER_IP = '192.168.1.X'  # Replace with sender Raspberry Pi's IP address
PORT = 5000
BUFFER_SIZE = 4096
RECONNECT_TIMEOUT = 5  # Seconds to wait before reconnection attempt

# Fungsi Koreksi Bias
def correct_bias(raw_distance, bias):
    return raw_distance - bias

def connect_to_sender():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.settimeout(10.0)  # 10 second timeout for connection
    print(f"Connecting to sender at {SENDER_IP}:{PORT}...")
    client_socket.connect((SENDER_IP, PORT))
    client_socket.settimeout(2.0)  # 2 second timeout for data reception
    print("Connected!")
    return client_socket

# Main program loop
while True:
    client_socket = None
    try:
        # Create socket and connect to server
        client_socket = connect_to_sender()
        
        # Buffer for incomplete messages
        data_buffer = b''
        
        while True:
            try:
                # First receive the message length (4 bytes)
                length_data = b''
                while len(length_data) < 4:
                    chunk = client_socket.recv(4 - len(length_data))
                    if not chunk:
                        raise socket.error("Connection closed by sender during length reception")
                    length_data += chunk
                
                # Unpack the message length
                message_length = struct.unpack('!I', length_data)[0]
                
                # Then receive the exact message
                data = b''
                while len(data) < message_length:
                    chunk = client_socket.recv(min(BUFFER_SIZE, message_length - len(data)))
                    if not chunk:
                        raise socket.error("Connection closed by sender during data reception")
                    data += chunk
                
                # Decode and process the message
                data_str = data.decode('utf-8').strip()
                
                if data_str.startswith("$KT0"):
                    try:
                        parts = data_str.split(",")
                        if len(parts) >= 4:
                            raw_values = parts[1:4]
                            processed_values = []
                            
                            for i, value in enumerate(raw_values):
                                if value.lower() == "null":
                                    processed_values.append(0.0)
                                else:
                                    processed_values.append(float(value))
                            
                            A0, A1, A2 = processed_values
                            cal_A0 = correct_bias(A0*100, 30)
                            cal_A1 = correct_bias(A1*100, 0)
                            cal_A2 = correct_bias(A2*100, 0)
                            
                            print(
                                f"A0 = {cal_A0:.2f} centimeter | A1 = {cal_A1:.2f} centimeter | A2 = {cal_A2:.2f} centimeter"
                            )
                        else:
                            print("Error: Data tidak lengkap.")
                    except ValueError as e:
                        print(f"Error processing data: {e}")
            
            except socket.timeout:
                print("Waiting for data...")
                # Continue loop to try again
                
    except socket.error as e:
        print(f"Socket error: {e}")
        if client_socket:
            client_socket.close()
            client_socket = None
        print(f"Waiting {RECONNECT_TIMEOUT} seconds before reconnecting...")
        time.sleep(RECONNECT_TIMEOUT)
        
    except KeyboardInterrupt:
        print("Exiting program.")
        if client_socket:
            client_socket.close()
        break
