# Safety features

import numpy as np
import threading

class SafetyMonitor:
    """Monitor for ensuring robot safety"""
    
    def __init__(self, config=None):
        """
        Initialize the safety monitor
        
        Args:
            config (dict): Configuration for safety parameters
        """
        self.config = config or {}
        self.min_obstacle_distance = self.config.get('min_obstacle_distance', 0.5)
        self.max_target_error = self.config.get('max_target_error', 0.5)
        self.min_confidence = self.config.get('min_confidence', 0.6)
        
        self._emergency_stop = False
        self._lock = threading.Lock()
        self._obstacle_detected = False
        self._target_lost = False
        
    def check_lidar_safety(self, lidar_data):
        """
        Check LIDAR data for safety concerns
        
        Args:
            lidar_data (numpy.ndarray): Array of distance measurements
            
        Returns:
            bool: True if safe, False if unsafe
        """
        with self._lock:
            # Check for obstacles in front of the robot
            # Typically we care about obstacles in front of the robot (e.g. -45 to +45 degrees)
            front_indices = list(range(315, 360)) + list(range(0, 46))
            front_distances = lidar_data[front_indices]
            
            # Check if any distance is below safety threshold
            unsafe = np.any(front_distances < self.min_obstacle_distance)
            
            if unsafe:
                self._obstacle_detected = True
                return False
            else:
                self._obstacle_detected = False
                return True
    
    def check_target_tracking(self, position, diagnostics):
        """
        Check if target tracking is reliable
        
        Args:
            position (numpy.ndarray): Target position
            diagnostics (dict): Target tracking diagnostics
            
        Returns:
            bool: True if target tracking is reliable, False otherwise
        """
        with self._lock:
            error = diagnostics.get('error_estimate', 0.0)
            confidence = diagnostics.get('confidence', 1.0)
            
            # Check if position is valid
            if np.isnan(position).any() or np.isinf(position).any():
                self._target_lost = True
                return False
            
            # Check error and confidence
            if error > self.max_target_error or confidence < self.min_confidence:
                self._target_lost = True
                return False
            
            self._target_lost = False
            return True
    
    def emergency_stop(self):
        """
        Trigger emergency stop
        
        Returns:
            bool: Always True
        """
        with self._lock:
            self._emergency_stop = True
        return True
    
    def clear_emergency_stop(self):
        """
        Clear emergency stop
        
        Returns:
            bool: Always True
        """
        with self._lock:
            self._emergency_stop = False
        return True
    
    def is_emergency_stop(self):
        """
        Check if emergency stop is active
        
        Returns:
            bool: True if emergency stop is active, False otherwise
        """
        with self._lock:
            return self._emergency_stop
    
    def get_safety_status(self):
        """
        Get current safety status
        
        Returns:
            dict: Safety status
        """
        with self._lock:
            return {
                'emergency_stop': self._emergency_stop,
                'obstacle_detected': self._obstacle_detected,
                'target_lost': self._target_lost
            }
