# Robot Controller Implementation

import time
import numpy as np
import threading
from controllers.pid_controller import PIDController
from utils.safety import SafetyMonitor

class RobotController:
    """Main controller for the robot follower system"""
    
    def __init__(self, lidar, uwb_tracker, config=None, simulation_mode=False):
        """
        Initialize the robot controller
        
        Args:
            lidar: LidarSensor instance for obstacle detection
            uwb_tracker: UWBTracker instance for target tracking
            config (dict): Configuration parameters
            simulation_mode (bool): Whether to run in simulation mode
        """
        self.lidar = lidar
        self.uwb_tracker = uwb_tracker
        self.config = config or {}
        self.simulation_mode = simulation_mode
        
        # Initialize safety monitor
        safety_config = self.config.get('SAFETY_CONFIG', {})
        self.safety = SafetyMonitor(safety_config)
        
        # Initialize PID controllers
        self.distance_pid = PIDController(
            kp=0.5, ki=0.1, kd=0.2,
            setpoint=self.config.get('ROBOT_CONFIG', {}).get('min_follow_distance', 100),
            output_limits=(-100, 100)
        )
        
        self.angle_pid = PIDController(
            kp=1.0, ki=0.0, kd=0.5,
            setpoint=0.0,  # We want to keep the target centered (angle = 0)
            output_limits=(-100, 100)
        )
        
        # Robot state
        self.left_speed = 0
        self.right_speed = 0
        self._position = np.zeros(2)  # [x, y]
        self._orientation = 0.0  # radians
        
        # Control loop
        self._stop_event = threading.Event()
        self._control_thread = None
        self._lock = threading.Lock()
        self.is_running = False
        self._last_time = time.time()
    
    def start(self):
        """
        Start the robot controller
        
        Returns:
            bool: True if successful, False otherwise
        """
        if self.is_running:
            return True
            
        try:
            # Start sensors if they're not already running
            if not self.lidar.is_running:
                self.lidar.start()
            
            if not self.uwb_tracker.is_running:
                self.uwb_tracker.start()
            
            # Start control loop
            self._stop_event.clear()
            self._control_thread = threading.Thread(target=self._control_loop)
            self._control_thread.daemon = True
            self._control_thread.start()
            
            self.is_running = True
            return True
            
        except Exception as e:
            print(f"Error starting robot controller: {e}")
            return False
    
    def stop(self):
        """
        Stop the robot controller
        
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.is_running:
            return True
            
        try:
            # Stop control loop
            self._stop_event.set()
            self.set_motor_speeds(0, 0)
            
            if self._control_thread:
                self._control_thread.join(timeout=1.0)
            
            self.is_running = False
            return True
            
        except Exception as e:
            print(f"Error stopping robot controller: {e}")
            return False
    
    def set_motor_speeds(self, left, right):
        """
        Set motor speeds
        
        Args:
            left (float): Left motor speed (-100 to 100)
            right (float): Right motor speed (-100 to 100)
            
        Returns:
            bool: True if successful, False otherwise
        """
        with self._lock:
            # Ensure speeds are within limits
            self.left_speed = max(-100, min(100, left))
            self.right_speed = max(-100, min(100, right))
            
            if not self.simulation_mode:
                # Here we would send commands to the actual motors
                # Example: self._send_motor_command(self.left_speed, self.right_speed)
                pass
            
            return True
    
    def get_current_speed(self):
        """
        Get current motor speeds
        
        Returns:
            tuple: (left_speed, right_speed)
        """
        with self._lock:
            return (self.left_speed, self.right_speed)
    
    def get_current_position(self):
        """
        Get current robot position and orientation
        
        Returns:
            tuple: (position, orientation)
                position: numpy.ndarray - 2D position [x, y]
                orientation: float - orientation in radians
        """
        with self._lock:
            return (self._position.copy(), self._orientation)
    
    def set_target_distance(self, distance):
        """
        Set desired target distance
        
        Args:
            distance (float): Desired distance in cm
            
        Returns:
            bool: True if successful, False otherwise
        """
        self.distance_pid.set_setpoint(distance)
        return True
    
    def _control_loop(self):
        """Internal thread for robot control"""
        last_status_time = time.time()
        status_interval = 1.0  # seconds
        
        while not self._stop_event.is_set():
            try:
                current_time = time.time()
                dt = current_time - self._last_time
                self._last_time = current_time
                
                # Skip if dt is too small
                if dt < 0.01:
                    time.sleep(0.01)
                    continue
                
                # Check safety status
                if self.safety.is_emergency_stop():
                    self.set_motor_speeds(0, 0)
                    time.sleep(0.1)
                    continue
                
                # Get sensor data
                lidar_data = self.lidar.get_data()
                target_pos, target_vel, target_diag = self.uwb_tracker.get_data()
                
                # Check safety conditions
                lidar_safe = self.safety.check_lidar_safety(lidar_data)
                target_reliable = self.safety.check_target_tracking(target_pos, target_diag)
                
                # If unsafe, stop
                if not lidar_safe:
                    print("Obstacle detected! Stopping.")
                    self.set_motor_speeds(0, 0)
                    time.sleep(0.1)
                    continue
                
                if not target_reliable:
                    print("Target tracking unreliable! Stopping.")
                    self.set_motor_speeds(0, 0)
                    time.sleep(0.1)
                    continue
                
                # Calculate distance and angle to target
                target_distance = np.linalg.norm(target_pos)
                target_angle = np.arctan2(target_pos[1], target_pos[0])
                
                # Update PID controllers
                distance_output = self.distance_pid.update(target_distance, current_time)
                angle_output = self.angle_pid.update(target_angle, current_time)
                
                # Convert PID outputs to motor speeds
                base_speed = self.config.get('ROBOT_CONFIG', {}).get('base_speed', 50)
                
                # Simple differential drive control
                left_speed = base_speed - angle_output
                right_speed = base_speed + angle_output
                
                # Adjust overall speed based on distance
                if distance_output < 0:  # We're too close, slow down or back up
                    speed_factor = max(-1.0, distance_output / 100.0)
                    left_speed *= speed_factor
                    right_speed *= speed_factor
                elif distance_output > 0:  # We're too far, speed up
                    speed_factor = min(2.0, 1.0 + distance_output / 100.0)
                    left_speed *= speed_factor
                    right_speed *= speed_factor
                
                # Apply motor speeds
                self.set_motor_speeds(left_speed, right_speed)
                
                # Update simulated position if in simulation mode
                if self.simulation_mode:
                    self._update_simulation(dt)
                
                # Print status occasionally
                if current_time - last_status_time >= status_interval:
                    status = {
                        'target_pos': target_pos.tolist(),
                        'target_distance': target_distance,
                        'target_angle': target_angle,
                        'left_speed': self.left_speed,
                        'right_speed': self.right_speed,
                        'safety': self.safety.get_safety_status()
                    }
                    print(f"Status: {status}")
                    last_status_time = current_time
                
            except Exception as e:
                print(f"Error in control loop: {e}")
            
            # Sleep a bit to prevent CPU overuse
            time.sleep(0.05)
    
    def _update_simulation(self, dt):
        """
        Update simulation state
        
        Args:
            dt (float): Time delta in seconds
        """
        if not self.simulation_mode:
            return
            
        with self._lock:
            # Calculate linear and angular velocity based on differential drive model
            # This is a simplified model
            wheel_base = 0.2  # meters between wheels
            speed_scale = 0.001  # scale from -100,100 to m/s
            
            left_vel = self.left_speed * speed_scale
            right_vel = self.right_speed * speed_scale
            
            linear_vel = (left_vel + right_vel) / 2
            angular_vel = (right_vel - left_vel) / wheel_base
            
            # Update orientation
            self._orientation += angular_vel * dt
            
            # Update position based on linear velocity and orientation
            self._position[0] += linear_vel * dt * np.cos(self._orientation)
            self._position[1] += linear_vel * dt * np.sin(self._orientation)
