# placeholder/n

# Visualization utilities

import socket
import json
import threading
import time

class VisualizationClient:
    """Client for sending visualization data to external visualization tools"""
    
    def __init__(self, host='localhost', port=5555):
        """
        Initialize the visualization client
        
        Args:
            host (str): Host to connect to
            port (int): Port to connect to
        """
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False
        self._lock = threading.Lock()
        self._last_data = None
        self._update_thread = None
        self._stop_event = threading.Event()
    
    def connect(self):
        """
        Connect to the visualization server
        
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            if self.connected:
                return True
                
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.host, self.port))
            self.connected = True
            
            # Start update thread
            self._stop_event.clear()
            self._update_thread = threading.Thread(target=self._update_loop)
            self._update_thread.daemon = True
            self._update_thread.start()
            
            return True
            
        except socket.error as e:
            print(f"Visualization connection error: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """
        Disconnect from the visualization server
        
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            self._stop_event.set()
            if self._update_thread:
                self._update_thread.join(timeout=1.0)
                
            if self.socket:
                self.socket.close()
                
            self.connected = False
            return True
            
        except Exception as e:
            print(f"Visualization disconnect error: {e}")
            return False
    
    def send_data(self, data):
        """
        Send data to visualization server
        
        Args:
            data (dict): Data to send
            
        Returns:
            bool: True if successful, False otherwise
        """
        with self._lock:
            self._last_data = data
            
        if not self.connected:
            try:
                self.connect()
            except:
                return False
                
        try:
            if self.connected:
                data_json = json.dumps(data)
                self.socket.sendall((data_json + '\n').encode())
                return True
            return False
            
        except socket.error as e:
            print(f"Visualization send error: {e}")
            self.connected = False
            return False
    
    def _update_loop(self):
        """Thread that periodically sends data updates"""
        while not self._stop_event.is_set():
            try:
                # If we have data and are connected, send an update
                if self._last_data is not None and self.connected:
                    with self._lock:
                        data = self._last_data
                    
                    data_json = json.dumps(data)
                    self.socket.sendall((data_json + '\n').encode())
            except socket.error:
                self.connected = False
                try:
                    # Try to reconnect
                    time.sleep(1.0)
                    self.connect()
                except:
                    pass
            
            # Sleep between updates
            time.sleep(0.1)
