from rplidar import RPLidar
import serial
import time
import threading
import numpy as np
import os
import struct

class LidarA2M12Handler:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.lidar = None
        self.serial_port = None
        self.scan_data = np.full(360, float('inf'))
        self.running = False
        self.lock = threading.Lock()
        self.scan_thread = None
        
        # Obstacle detection parameters
        self.obstacle_threshold = 400  # mm
        self.front_angles = list(range(340, 360)) + list(range(0, 20))
        self.left_angles = list(range(20, 90))
        self.right_angles = list(range(270, 340))
        
        # Obstacle status
        self.obstacles = {
            'front': False, 
            'left': False, 
            'right': False,
            'front_dist': float('inf'),
            'left_dist': float('inf'),
            'right_dist': float('inf')
        }

    def verify_device(self):
        """Verify if the device exists and is accessible"""
        if not os.path.exists(self.port):
            print(f"Device {self.port} does not exist.")
            return False
        
        try:
            # Check if we can open the port
            test_serial = serial.Serial(self.port, self.baudrate, timeout=1)
            test_serial.close()
            return True
        except Exception as e:
            print(f"Cannot access {self.port}: {e}")
            return False

    def direct_connect(self):
        """Connect directly using serial communication bypassing rplidar library"""
        try:
            print(f"Attempting direct serial connection to {self.port}...")
            
            # Open serial connection
            self.serial_port = serial.Serial(
                self.port,
                self.baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=2
            )
            
            # Reset the device by toggling DTR
            self.serial_port.setDTR(False)
            time.sleep(0.5)
            self.serial_port.setDTR(True)
            time.sleep(0.5)
            
            # Clear any pending data
            self.serial_port.reset_input_buffer()
            
            # Send stop command (binary)
            self.serial_port.write(b'\xA5\x25')
            time.sleep(0.5)
            
            # Send reset command
            self.serial_port.write(b'\xA5\x40')
            time.sleep(2)  # Wait for reset
            self.serial_port.reset_input_buffer()
            
            # Start motor by sending motor start command
            self.serial_port.write(b'\xA5\xF0')
            time.sleep(1)
            
            # Send scan command
            self.serial_port.write(b'\xA5\x20')
            time.sleep(0.5)
            
            # Read some data to verify communication
            data = self.serial_port.read(10)
            if len(data) > 0:
                print(f"Received data from LIDAR: {data.hex()}")
                return True
            else:
                print("No data received from LIDAR")
                self.serial_port.close()
                self.serial_port = None
                return False
                
        except Exception as e:
            print(f"Direct serial connection failed: {e}")
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            self.serial_port = None
            return False

    def connect_with_library(self):
        """Attempt to connect using the rplidar library"""
        try:
            print(f"Attempting to connect using rplidar library...")
            self.lidar = RPLidar(self.port, baudrate=self.baudrate, timeout=3)
            
            # First stop motor if running
            try:
                self.lidar.stop_motor()
                time.sleep(0.5)
            except:
                pass
                
            # Start motor
            self.lidar.start_motor()
            time.sleep(1)
            
            # Try to get info - this will throw the exception if incompatible
            try:
                info = self.lidar.get_info()
                health = self.lidar.get_health()
                print(f"LIDAR connected. Model: {info['model']}, Firmware: {info['firmware']}")
                print(f"LIDAR health: {health}")
                return True
            except Exception as e:
                print(f"Could not get LIDAR info: {e}")
                # Continue to fallback method
                self.lidar.stop_motor()
                self.lidar.disconnect()
                self.lidar = None
                return False
                
        except Exception as e:
            print(f"Library connection failed: {e}")
            if self.lidar:
                try:
                    self.lidar.stop_motor()
                    self.lidar.disconnect()
                except:
                    pass
                self.lidar = None
            return False

    def connect(self):
        """Try multiple connection methods"""
        if not self.verify_device():
            # Try alternative device paths
            alternative_ports = ['/dev/ttyUSB1', '/dev/ttyACM0', '/dev/ttyACM1']
            print(f"Trying alternative ports: {alternative_ports}")
            
            for alt_port in alternative_ports:
                self.port = alt_port
                if self.verify_device():
                    print(f"Found device at {alt_port}")
                    break
            else:
                print("No LIDAR device found at any standard port")
                return False
        
        # Try rplidar library first
        if self.connect_with_library():
            print("Connected using rplidar library")
            return True
            
        # If library fails, try direct connection
        if self.direct_connect():
            print("Connected using direct serial communication")
            return True
            
        print("All connection attempts failed")
        return False

    def _parse_scan_data(self, data):
        """Parse raw scan data from direct serial connection"""
        if len(data) < 5:
            return None, None
            
        # Check if this is a valid data packet
        quality = data[0] & 0xFC
        angle_q6 = ((data[1] & 0xFF) | ((data[2] & 0x3F) << 8)) >> 1
        angle = angle_q6 / 64.0
        distance = ((data[3] & 0xFF) | ((data[4] & 0xFF) << 8)) / 4.0  # mm
        
        if distance == 0:
            return None, None
            
        return angle, distance

    def _scan_process_library(self):
        """Process scan data using the rplidar library"""
        try:
            print("Starting scan using rplidar library...")
            for _, angle, distance in self.lidar.iter_measures(max_buf_meas=3000):
                if not self.running:
                    break
                
                if distance > 0:
                    angle_int = int(angle)
                    if 0 <= angle_int < 360:
                        with self.lock:
                            self.scan_data[angle_int] = min(distance, self.scan_data[angle_int])
                            
                # Periodically process obstacles
                self._detect_obstacles()
                time.sleep(0.001)  # Small delay to prevent CPU hogging
                
        except Exception as e:
            print(f"Scan error (library): {e}")
            return False
            
        return True

    def _scan_process_direct(self):
        """Process scan data using direct serial communication"""
        try:
            print("Starting scan using direct serial connection...")
            buffer = bytearray()
            
            while self.running:
                if self.serial_port.in_waiting:
                    # Read available data
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    buffer.extend(data)
                    
                    # Process complete packets
                    while len(buffer) >= 5:
                        # Check for start flag
                        if (buffer[0] & 0x01) == 1:
                            # Parse the measurement
                            angle, distance = self._parse_scan_data(buffer[:5])
                            
                            if angle is not None and distance is not None:
                                angle_int = int(angle)
                                if 0 <= angle_int < 360:
                                    with self.lock:
                                        self.scan_data[angle_int] = min(distance, self.scan_data[angle_int])
                            
                            # Remove processed bytes
                            buffer = buffer[5:]
                        else:
                            # Not a valid start, skip one byte
                            buffer = buffer[1:]
                    
                    # Process obstacles periodically
                    self._detect_obstacles()
                
                time.sleep(0.001)  # Small delay
                
        except Exception as e:
            print(f"Scan error (direct): {e}")
            return False
            
        return True

    def start(self):
        """Start LIDAR scanning"""
        if not self.lidar and not self.serial_port:
            if not self.connect():
                return False
        
        self.running = True
        
        # Determine which scan method to use
        if self.lidar:
            self.scan_thread = threading.Thread(target=self._scan_process_library)
        else:
            self.scan_thread = threading.Thread(target=self._scan_process_direct)
            
        self.scan_thread.daemon = True
        self.scan_thread.start()
        
        print("LIDAR scanning started")
        return True

    def _detect_obstacles(self):
        """Detect obstacles in front, left, and right sectors"""
        with self.lock:
            # Get valid distance readings (ignore infinity values)
            front_valid = [self.scan_data[a] for a in self.front_angles if self.scan_data[a] < float('inf')]
            left_valid = [self.scan_data[a] for a in self.left_angles if self.scan_data[a] < float('inf')]
            right_valid = [self.scan_data[a] for a in self.right_angles if self.scan_data[a] < float('inf')]
            
            # Calculate minimum distances with fallbacks
            front_min = min(front_valid) if front_valid else float('inf')
            left_min = min(left_valid) if left_valid else float('inf')
            right_min = min(right_valid) if right_valid else float('inf')
            
            # Update obstacle status
            self.obstacles = {
                'front': front_min < self.obstacle_threshold,
                'left': left_min < self.obstacle_threshold,
                'right': right_min < self.obstacle_threshold,
                'front_dist': front_min,
                'left_dist': left_min,
                'right_dist': right_min
            }

    def get_obstacle_status(self):
        """Get current obstacle status (thread-safe)"""
        with self.lock:
            return self.obstacles.copy()

    def stop(self):
        """Stop LIDAR scanning and clean up resources"""
        self.running = False
        
        if self.scan_thread and self.scan_thread.is_alive():
            self.scan_thread.join(timeout=2)
        
        if self.lidar:
            try:
                self.lidar.stop_motor()
                self.lidar.disconnect()
            except Exception as e:
                print(f"Error stopping LIDAR: {e}")
            self.lidar = None
            
        if self.serial_port:
            try:
                # Send stop scan command
                self.serial_port.write(b'\xA5\x25')
                time.sleep(0.5)
                # Stop motor
                self.serial_port.write(b'\xA5\xF5')
                time.sleep(0.5)
                self.serial_port.close()
            except Exception as e:
                print(f"Error closing serial port: {e}")
            self.serial_port = None
            
        print("LIDAR stopped")

# Demo of usage
if __name__ == "__main__":
    lidar = LidarA2M12Handler(port="/dev/ttyUSB0")
    
    if lidar.start():
        try:
            print("Obstacle detection running. Press Ctrl+C to stop.")
            while True:
                obstacles = lidar.get_obstacle_status()
                print(f"Obstacles: Front={obstacles['front']} ({obstacles['front_dist']:.0f}mm), " +
                      f"Left={obstacles['left']} ({obstacles['left_dist']:.0f}mm), " +
                      f"Right={obstacles['right']} ({obstacles['right_dist']:.0f}mm)")
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("Stopping...")
        finally:
            lidar.stop()
    else:
        print("Failed to start LIDAR")
