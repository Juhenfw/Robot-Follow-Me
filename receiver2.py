import socket
import struct
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading

# Configuration
HOST = '127.0.0.1'  # Server IP address (change to sender's IP)
PORT = 5000         # Server port (must match sender)
RECONNECT_TIMEOUT = 5
BUFFER_SIZE = 100   # Size of history buffer for plotting

class UWBReceiver:
    def __init__(self, host=HOST, port=PORT, buffer_size=BUFFER_SIZE):
        # Connection settings
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False
        
        # Data buffers for visualization (for each channel)
        self.timestamps = deque(maxlen=buffer_size)
        self.channel_data = [deque(maxlen=buffer_size) for _ in range(3)]
        self.raw_data = [0, 0, 0]  # Latest raw values
        
        # Lock for thread safety
        self.data_lock = threading.Lock()
        
        # Start connection thread
        self.running = True
        self.connection_thread = threading.Thread(target=self.connection_manager)
        self.connection_thread.daemon = True
        self.connection_thread.start()
    
    def connect(self):
        """Establish connection to the sender"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            self.connected = True
            print(f"Connected to {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Close the connection"""
        if self.socket:
            self.socket.close()
            self.socket = None
            self.connected = False
            print("Disconnected from sender")
    
    def connection_manager(self):
        """Manages connection and data reception"""
        while self.running:
            if not self.connected:
                if self.connect():
                    self.receive_data()
                else:
                    print(f"Reconnecting in {RECONNECT_TIMEOUT} seconds...")
                    time.sleep(RECONNECT_TIMEOUT)
            else:
                time.sleep(0.1)  # Small delay when connected
    
    def receive_data(self):
        """Receive and process data from the sender"""
        try:
            while self.connected and self.running:
                # First receive message length (4 bytes)
                length_data = self.socket.recv(4)
                if not length_data:
                    raise Exception("Connection closed by sender")
                
                message_length = struct.unpack('!I', length_data)[0]
                
                # Receive the actual message
                chunks = []
                bytes_received = 0
                while bytes_received < message_length:
                    chunk = self.socket.recv(min(message_length - bytes_received, 1024))
                    if not chunk:
                        raise Exception("Connection closed during data transmission")
                    chunks.append(chunk)
                    bytes_received += len(chunk)
                
                message = b''.join(chunks).decode('utf-8')
                self.process_message(message)
                
        except Exception as e:
            print(f"Error in data reception: {e}")
            self.disconnect()
    
    def process_message(self, message):
        """Process received UWB data"""
        try:
            # Parse message format: $KT0,x.xx,y.yy,z.zz
            if message.startswith("$KT0"):
                parts = message.strip().split(',')
                if len(parts) >= 4:
                    # Extract distance values
                    distances = [float(parts[i]) for i in range(1, 4)]
                    
                    # Update data buffers with thread safety
                    with self.data_lock:
                        self.raw_data = distances.copy()
                        current_time = time.time()
                        self.timestamps.append(current_time)
                        
                        for i, value in enumerate(distances):
                            self.channel_data[i].append(value)
                    
                    # Print received data
                    print(f"Received: CH0={distances[0]:.2f}m, CH1={distances[1]:.2f}m, CH2={distances[2]:.2f}m")
        except Exception as e:
            print(f"Error processing message: {e}")
    
    def get_latest_data(self):
        """Get the latest data (thread-safe)"""
        with self.data_lock:
            return self.raw_data.copy()
    
    def get_history_data(self):
        """Get historical data for plotting (thread-safe)"""
        with self.data_lock:
            times = list(self.timestamps)
            channels = [list(ch) for ch in self.channel_data]
            return times, channels
    
    def shutdown(self):
        """Shutdown the receiver"""
        self.running = False
        self.disconnect()
        if self.connection_thread.is_alive():
            self.connection_thread.join(timeout=1.0)


class UWBVisualizer:
    def __init__(self, receiver):
        self.receiver = receiver
        
        # Create plot
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.fig.canvas.manager.set_window_title('UWB Sensor Data Visualization')
        
        # Initialize lines for each channel
        self.lines = []
        colors = ['r', 'g', 'b']
        labels = ['Channel 0', 'Channel 1', 'Channel 2']
        
        for i in range(3):
            line, = self.ax.plot([], [], colors[i], label=labels[i])
            self.lines.append(line)
        
        # Set up plot
        self.ax.set_title('UWB Distance Measurements')
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Distance (m)')
        self.ax.set_ylim(0, 10)  # Adjust based on expected range
        self.ax.grid(True)
        self.ax.legend(loc='upper right')
        
        # Text for real-time values
        self.text = self.ax.text(0.02, 0.95, '', transform=self.ax.transAxes)
        
        # Set up animation
        self.ani = FuncAnimation(self.fig, self.update, interval=100, blit=True)
    
    def update(self, frame):
        """Update function for animation"""
        times, channels = self.receiver.get_history_data()
        
        if not times:  # No data yet
            return self.lines + [self.text]
        
        # Convert timestamps to relative time in seconds
        relative_times = [t - times[0] for t in times]
        
        # Update each channel's line
        for i, line in enumerate(self.lines):
            if len(relative_times) == len(channels[i]):
                line.set_data(relative_times, channels[i])
        
        # Update axis limits if needed
        if relative_times:
            self.ax.set_xlim(max(0, relative_times[-1] - 30), max(30, relative_times[-1]))
        
        # Update real-time value text
        latest = self.receiver.get_latest_data()
        status_text = f"Latest: CH0={latest[0]:.2f}m, CH1={latest[1]:.2f}m, CH2={latest[2]:.2f}m"
        self.text.set_text(status_text)
        
        return self.lines + [self.text]
    
    def show(self):
        """Show the visualization"""
        plt.tight_layout()
        plt.show()


# Additional data processing functions
def calculate_position(distances):
    """
    Calculate position based on UWB distances (trilateration)
    This is a simplified example - real implementation would need anchor positions
    """
    # Placeholder for trilateration algorithm
    # In a real implementation, this would use the distances from anchors
    # to calculate the 2D or 3D position
    
    if len(distances) >= 3:
        # Simplified position estimation
        x = (distances[0]**2 - distances[1]**2) / 2
        y = (distances[0]**2 - distances[2]**2) / 2
        return (x, y)
    return (0, 0)


# Example usage
if __name__ == "__main__":
    try:
        # Create receiver
        receiver = UWBReceiver()
        
        # Optional: Set up data logging
        log_file = open("uwb_data.csv", "w")
        log_file.write("timestamp,channel0,channel1,channel2,x_pos,y_pos\n")
        
        # Create and start visualizer
        visualizer = UWBVisualizer(receiver)
        
        # Start a separate thread for data logging
        def log_data():
            while receiver.running:
                data = receiver.get_latest_data()
                x, y = calculate_position(data)
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                log_file.write(f"{timestamp},{data[0]},{data[1]},{data[2]},{x},{y}\n")
                log_file.flush()
                time.sleep(1)
        
        log_thread = threading.Thread(target=log_data)
        log_thread.daemon = True
        log_thread.start()
        
        # Show visualization (this will block until window is closed)
        visualizer.show()
        
    except KeyboardInterrupt:
        print("Program interrupted by user")
    finally:
        # Clean shutdown
        if 'receiver' in locals():
            receiver.shutdown()
        if 'log_file' in locals():
            log_file.close()
            print("Data logging completed")
        print("Program terminated")
