# placeholder/n

# Base class for all sensors

from abc import ABC, abstractmethod

class Sensor(ABC):
    """Base abstract class for all sensors"""
    
    def __init__(self, simulation_mode=False, config=None):
        """
        Initialize the sensor
        
        Args:
            simulation_mode (bool): Whether to run in simulation mode
            config (dict): Configuration parameters
        """
        self.simulation_mode = simulation_mode
        self.config = config or {}
        self.is_running = False
    
    @abstractmethod
    def start(self):
        """
        Start the sensor
        
        Returns:
            bool: True if successful, False otherwise
        """
        pass
    
    @abstractmethod
    def stop(self):
        """
        Stop the sensor
        
        Returns:
            bool: True if successful, False otherwise
        """
        pass
    
    @abstractmethod
    def get_data(self):
        """
        Get data from the sensor
        
        Returns:
            Any: Sensor-specific data
        """
        pass
