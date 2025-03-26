import socket

# Configure connection
SENDER_IP = '192.168.1.X'  # Replace with the sender Raspberry Pi's IP address
PORT = 5000
BUFFER_SIZE = 1024

# Fungsi Koreksi Bias
def correct_bias(raw_distance, bias):
    return raw_distance - bias

try:
    # Create socket and connect to server
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((SENDER_IP, PORT))
    print(f"Connected to sender at {SENDER_IP}:{PORT}")
    
    # Create buffer for data that might be split across packets
    data_buffer = ""
    
    while True:
        # Receive data
        data_chunk = client_socket.recv(BUFFER_SIZE).decode('utf-8')
        if not data_chunk:
            print("Connection closed by sender")
            break
            
        # Add received data to buffer
        data_buffer += data_chunk
        
        # Process complete lines in buffer
        while '\n' in data_buffer:
            # Extract a line and remove it from buffer
            line, data_buffer = data_buffer.split('\n', 1)
            data = line.strip()
            
            if data.startswith("$KT0"):
                try:
                    parts = data.split(",")
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

except socket.error as e:
    print(f"Socket error: {e}")
except KeyboardInterrupt:
    print("Exiting program.")
finally:
    if 'client_socket' in locals():
        client_socket.close()
        print("Connection closed.")
