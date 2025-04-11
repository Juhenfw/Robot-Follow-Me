from rplidar import RPLidar
import time
import threading
import numpy as np

class LidarObstacleAvoidance:
    def __init__(self, port='/dev/ttyUSB0'):
        self.port = port
        self.lidar = None
        self.scan_data = np.full(360, float('inf'))  # Initialize with infinity
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
    
    def connect(self):
        """Connect to LIDAR device with error handling"""
        max_attempts = 3
        for attempt in range(max_attempts):
            try:
                print(f"Connecting to LIDAR (attempt {attempt+1}/{max_attempts})...")
                self.lidar = RPLidar(self.port, timeout=3)
                
                # First stop motor if it's running
                try:
                    self.lidar.stop_motor()
                    time.sleep(0.5)
                except:
                    pass
                
                # Start motor and verify connection
                self.lidar.start_motor()
                time.sleep(1)  # Allow motor to stabilize
                
                # Get info to verify connection
                info = self.lidar.get_info()
                health = self.lidar.get_health()
                print(f"LIDAR connected. Info: {info}")
                print(f"LIDAR health: {health}")
                
                if health[0] == 'Error':
                    raise Exception(f"LIDAR reports error: {health}")
                
                return True
            except Exception as e:
                print(f"Failed to connect to LIDAR: {e}")
                if self.lidar:
                    try:
                        self.lidar.stop_motor()
                        self.lidar.disconnect()
                    except:
                        pass
                    self.lidar = None
                
                if attempt < max_attempts - 1:
                    print(f"Retrying in 2 seconds...")
                    time.sleep(2)
                else:
                    print("Failed to connect to LIDAR after multiple attempts")
                    return False
    
    def start(self):
        """Start LIDAR scanning"""
        if not self.lidar and not self.connect():
            return False
        
        self.running = True
        self.scan_thread = threading.Thread(target=self._scan_process)
        self.scan_thread.daemon = True
        self.scan_thread.start()
        print("LIDAR scanning started")
        return True
    
    def _scan_process(self):
        """Process LIDAR scan data in a separate thread"""
        # Reset scan data before starting
        with self.lock:
            self.scan_data = np.full(360, float('inf'))
        
        scan_count = 0
        error_count = 0
        
        while self.running:
            try:
                # Use iter_measures which is more reliable for A2M12
                for _, angle, distance in self.lidar.iter_measures(max_buf_meas=3000):
                    if not self.running:
                        break
                    
                    # Store valid measurements only
                    if distance > 0:
                        angle_int = int(angle)
                        if 0 <= angle_int < 360:
                            with self.lock:
                                self.scan_data[angle_int] = min(distance, self.scan_data[angle_int])
                    
                    scan_count += 1
                    # Process obstacles after accumulating enough measurements
                    if scan_count >= 90:
                        self._detect_obstacles()
                        scan_count = 0
                
                # Reset error count on successful iteration
                error_count = 0
                
            except Exception as e:
                error_count += 1
                print(f"LIDAR scan error: {e}")
                
                # If too many consecutive errors, try to reconnect
                if error_count >= 3:
                    print("Multiple errors detected. Attempting to reconnect LIDAR...")
                    try:
                        if self.lidar:
                            self.lidar.stop_motor()
                            self.lidar.disconnect()
                    except:
                        pass
                    
                    time.sleep(2)
                    if self.connect():
                        error_count = 0
                    else:
                        # If reconnection fails, stop scanning
                        self.running = False
                        break
                
                # Short delay before retrying
                time.sleep(1)
    
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
                time.sleep(0.5)
                self.lidar.disconnect()
            except Exception as e:
                print(f"Error stopping LIDAR: {e}")
            
            self.lidar = None
        
        print("LIDAR stopped")

# Example implementation of obstacle avoidance logic
def avoid_obstacles(lidar_processor, motor_controller):
    """Basic obstacle avoidance logic"""
    obstacles = lidar_processor.get_obstacle_status()
    
    print(f"Obstacle status: Front={obstacles['front']} ({obstacles['front_dist']:.0f}mm), "
          f"Left={obstacles['left']} ({obstacles['left_dist']:.0f}mm), "
          f"Right={obstacles['right']} ({obstacles['right_dist']:.0f}mm)")
    
    if obstacles['front']:
        if not obstacles['right'] and (obstacles['left'] or obstacles['right_dist'] < obstacles['left_dist']):
            print("Obstacle in front - turning right")
            motor_controller.turn_right()
        elif not obstacles['left']:
            print("Obstacle in front - turning left")
            motor_controller.turn_left()
        else:
            print("Obstacles all around - reversing")
            motor_controller.reverse()
    elif obstacles['left'] and not obstacles['right']:
        print("Obstacle on left - adjusting right")
        motor_controller.adjust_right()
    elif obstacles['right'] and not obstacles['left']:
        print("Obstacle on right - adjusting left")
        motor_controller.adjust_left()
    else:
        print("No obstacles - moving forward")
        motor_controller.forward()

# Example usage
if __name__ == "__main__":
    # Create mock motor controller
    class MockMotorController:
        def forward(self): print("Moving forward")
        def turn_left(self): print("Turning left")
        def turn_right(self): print("Turning right")
        def reverse(self): print("Moving backward")
        def adjust_left(self): print("Slight left adjustment")
        def adjust_right(self): print("Slight right adjustment")
        def stop(self): print("Stopping")
    
    motor_controller = MockMotorController()
    
    # Initialize and start LIDAR
    lidar = LidarObstacleAvoidance(port="/dev/ttyUSB0")
    if lidar.start():
        try:
            print("Obstacle avoidance running. Press Ctrl+C to stop.")
            while True:
                avoid_obstacles(lidar, motor_controller)
                time.sleep(0.2)  # Run obstacle avoidance at 5Hz
        except KeyboardInterrupt:
            print("Stopping...")
        finally:
            lidar.stop()
            motor_controller.stop()
