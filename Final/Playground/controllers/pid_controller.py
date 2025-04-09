# placeholder/n

# PID Controller implementation

import time

class PIDController:
    """PID controller for smooth control"""
    
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, setpoint=0.0, output_limits=None):
        """
        Initialize PID controller with given parameters
        
        Args:
            kp (float): Proportional gain
            ki (float): Integral gain
            kd (float): Derivative gain
            setpoint (float): Desired setpoint
            output_limits (tuple): (min, max) output limits, None for no limits
        """
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.setpoint = setpoint  # Desired setpoint
        self.output_limits = output_limits
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_update_time = time.time()
    
    def update(self, current_value, current_time=None):
        """
        Update the PID controller and get new output
        
        Args:
            current_value (float): Current process value
            current_time (float): Current time, defaults to time.time()
            
        Returns:
            float: Controller output
        """
        # Get current time if not provided
        if current_time is None:
            current_time = time.time()
            
        # Calculate dt
        dt = current_time - self.last_update_time
        
        # Avoid division by zero or negative dt
        if dt <= 0:
            dt = 0.001
            
        # Calculate error
        error = self.setpoint - current_value
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative term (avoid divide by zero)
        if dt > 0:
            d_term = self.kd * (error - self.prev_error) / dt
        else:
            d_term = 0
        
        # Save error and time for next iteration
        self.prev_error = error
        self.last_update_time = current_time
        
        # Calculate output
        output = p_term + i_term + d_term
        
        # Apply output limits if specified
        if self.output_limits is not None:
            output = max(self.output_limits[0], min(self.output_limits[1], output))
        
        return output
    
    def set_gains(self, kp=None, ki=None, kd=None):
        """
        Update the controller gains
        
        Args:
            kp (float): New proportional gain, or None to keep current
            ki (float): New integral gain, or None to keep current
            kd (float): New derivative gain, or None to keep current
        """
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
    
    def set_setpoint(self, setpoint):
        """
        Update the controller setpoint
        
        Args:
            setpoint (float): New setpoint
        """
        self.setpoint = setpoint
    
    def reset(self):
        """Reset the PID controller internal state"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_update_time = time.time()
