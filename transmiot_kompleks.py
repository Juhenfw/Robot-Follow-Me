import socket
import serial
import time
import numpy as np
from collections import deque
import threading

class AdaptiveKalmanFilter:
    def __init__(self, process_variance=0.125, measurement_variance=0.5, initial_value=0):
        self.base_process_variance = process_variance
        self.base_measurement_variance = measurement_variance
        self.estimate = initial_value
        self.estimate_error = 1.0
        self.last_measurement = initial_value
        self.movement_detected = False
        self.quality_factor = 1.0  # Signal quality factor
        
    def update(self, measurement, signal_quality=1.0, movement_scale=1.0):
        # Save quality factor for diagnostics
        self.quality_factor = signal_quality
        
        # Calculate dynamic variances based on movement and signal quality
        if signal_quality < 0.5:
            # Poor signal quality - trust the prediction more
            proc_variance = self.base_process_variance * 0.5
            meas_variance = self.base_measurement_variance * (2.0/signal_quality)
        else:
            # Adjust variances based on movement detection
            proc_variance = self.base_process_variance * movement_scale
            meas_variance = self.base_measurement_variance / signal_quality
            
        # Calculate rate of change
        delta = abs(measurement - self.last_measurement)
        self.movement_detected = delta > 0.1
        
        # Prediction update
        prediction_error = self.estimate_error + proc_variance
        
        # Measurement update
        kalman_gain = prediction_error / (prediction_error + meas_variance)
        self.estimate = self.estimate + kalman_gain * (measurement - self.estimate)
        self.estimate_error = (1 - kalman_gain) * prediction_error
        
        self.last_measurement = measurement
        return self.estimate

class SensorDataValidator:
    def __init__(self, max_rate_of_change=0.5, min_value=0.0, max_value=20.0):
        self.max_rate_of_change = max_rate_of_change  # Maximum allowed change per reading
        self.min_value = min_value                    # Minimum valid value
        self.max_value = max_value                    # Maximum valid value
        self.last_valid_value = None
        self.confidence = 1.0
        
    def validate(self, value, last_values=None):
        # Initialize if first reading
        if self.last_valid_value is None:
            if self.min_value <= value <= self.max_value:
                self.last_valid_value = value
                self.confidence = 1.0
                return value, self.confidence
            else:
                self.last_valid_value = max(min(value, self.max_value), self.min_value)
                self.confidence = 0.5
                return self.last_valid_value, self.confidence
        
        # Calculate rate of change if we have past values
        if last_values and len(last_values) > 0:
            avg_rate = np.mean([abs(value - v) for v in last_values[-3:]])
            sudden_change = avg_rate > self.max_rate_of_change
        else:
            sudden_change = abs(value - self.last_valid_value) > self.max_rate_of_change
        
        # Value range check
        value_in_range = self.min_value <= value <= self.max_value
        
        # Determine confidence and validated value
        if value_in_range and not sudden_change:
            # Valid value - high confidence
            self.confidence = 1.0
            self.last_valid_value = value
        elif value_in_range and sudden_change:
            # Suspicious change - medium confidence
            # We'll accept it but with reduced confidence
            self.confidence = 0.7
            self.last_valid_value = value
        else:
            # Invalid value - low confidence
            # For invalid values outside range, we clamp to range
            if not value_in_range:
                clamped_value = max(min(value, self.max_value), self.min_value)
                if abs(clamped_value - self.last_valid_value) <= self.max_rate_of_change:
                    # Clamped value is reasonable
                    self.confidence = 0.3
                    self.last_valid_value = clamped_value
                else:
                    # Keep last valid value with very low confidence
                    self.confidence = 0.1
            else:
                # For sudden changes, reduce confidence but accept partially
                weight = 0.3  # How much we trust the new value
                self.confidence = 0.5
                self.last_valid_value = (1-weight) * self.last_valid_value + weight * value
        
        return self.last_valid_value, self.confidence

class UWBTransmitter:
    def __init__(self, serial_port, udp_ip, udp_port):
        self.serial_port = serial_port
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        
        # Serial and socket setup
        self.ser = None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Data storage
        self.window_size = 5
        self.history = {
            'A0': deque(maxlen=self.window_size),
            'A1': deque(maxlen=self.window_size),
            'A2': deque(maxlen=self.window_size)
        }
        
        # Filters and validators
        self.kalman_filters = {
            'A0': AdaptiveKalmanFilter(process_variance=0.15, measurement_variance=0.4),
            'A1': AdaptiveKalmanFilter(process_variance=0.15, measurement_variance=0.4),
            'A2': AdaptiveKalmanFilter(process_variance=0.15, measurement_variance=0.4)
        }
        
        self.validators = {
            'A0': SensorDataValidator(max_rate_of_change=0.5, min_value=0.0, max_value=20.0),
            'A1': SensorDataValidator(max_rate_of_change=0.5, min_value=0.0, max_value=20.0),
            'A2': SensorDataValidator(max_rate_of_change=0.5, min_value=0.0, max_value=20.0)
        }
        
        # State tracking
        self.last_values = {'A0': None, 'A1': None, 'A2': None}
        self.last_sent_values = {'A0': None, 'A1': None, 'A2': None}
        self.stability_counter = {'A0': 0, 'A1': 0, 'A2': 0}
        self.last_send_time = 0
        self.running = True
        self.data_buffer = []
        self.buffer_lock = threading.Lock()
        
        # Settings
        self.movement_threshold = 0.08
        self.min_send_interval = 0.02  # 50Hz max send rate
        self.stuck_threshold = 15      # Readings before force reset
        
    def connect(self):
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=115200,
                timeout=0.1  # Reduced timeout for better responsiveness
            )
            print(f"Connected to {self.serial_port}")
            return True
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            return False
    
    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial connection closed.")
        self.sock.close()
        print("Socket connection closed.")
            
    def process_data(self, line):
        if not line.startswith("$KT0"):
            return
            
        try:
            parts = line.split(",")
            if len(parts) < 4:
                return
                
            # Parse raw values
            raw_values = {}
            for i, key in enumerate(['A0', 'A1', 'A2']):
                idx = i + 1
                if idx < len(parts) and parts[idx].lower() != "null":
                    try:
                        value = float(parts[idx])
                        raw_values[key] = value
                    except ValueError:
                        raw_values[key] = None
                else:
                    raw_values[key] = None
            
            with self.buffer_lock:
                self.data_buffer.append(raw_values)
        except Exception as e:
            print(f"Error parsing data: {e}")
    
    def process_buffer(self):
        filtered_values = {'A0': None, 'A1': None, 'A2': None}
        movement_detected = False
        
        # Process all buffered data
        with self.buffer_lock:
            if not self.data_buffer:
                return None
                
            # Process all buffered data to get latest state
            for raw_values in self.data_buffer:
                for key in raw_values:
                    if raw_values[key] is None:
                        continue
                        
                    # Add to history
                    self.history[key].append(raw_values[key])
                    
                    # Detect movement
                    if self.last_values[key] is not None:
                        delta = abs(raw_values[key] - self.last_values[key])
                        if delta > self.movement_threshold:
                            movement_detected = True
                            self.stability_counter[key] = 0
                        else:
                            self.stability_counter[key] += 1
                    
                    # Update last value
                    self.last_values[key] = raw_values[key]
            
            # Clear buffer after processing
            latest_values = self.data_buffer[-1]
            self.data_buffer.clear()
        
        # Compute filtered values from the latest data
        for key in ['A0', 'A1', 'A2']:
            # Skip if no value
            if latest_values.get(key) is None:
                filtered_values[key] = self.last_sent_values.get(key, 0.0)
                continue
                
            # Force reset if stuck
            force_reset = self.stability_counter[key] > self.stuck_threshold
            
            # Validate the data
            validated_value, confidence = self.validators[key].validate(
                latest_values[key], 
                list(self.history[key])
            )
            
            # Calculate dynamic movement scale (more responsive during movement)
            if movement_detected or force_reset:
                movement_scale = 3.0  # More responsive during movement
            else:
                movement_scale = 1.0
            
            # Apply Kalman filter
            kalman_value = self.kalman_filters[key].update(
                validated_value, 
                signal_quality=confidence,
                movement_scale=movement_scale
            )
            
            # Compute final value based on filter strategy
            if movement_detected or len(self.history[key]) < 3 or force_reset:
                # During movement or sparse data, prioritize responsiveness
                median_value = np.median(list(self.history[key])[-3:]) if self.history[key] else validated_value
                # Blend with emphasis on latest data for responsiveness
                filtered_values[key] = median_value * 0.7 + kalman_value * 0.3
            else:
                # During stability, prioritize smoothness
                filtered_values[key] = kalman_value * 0.8 + validated_value * 0.2
            
            # Reset if stuck
            if force_reset:
                self.kalman_filters[key].estimate = validated_value
                filtered_values[key] = validated_value
                self.stability_counter[key] = 0
            
            # Update last sent value
            self.last_sent_values[key] = filtered_values[key]
        
        return filtered_values, latest_values, movement_detected
            
    def send_data(self, filtered_values):
        # Format values with 3 decimal places
        formatted_values = [f"{filtered_values[key]:.3f}" for key in ['A0', 'A1', 'A2']]
        uwb_data = ",".join(formatted_values)
        
        # Send data
        self.sock.sendto(uwb_data.encode(), (self.udp_ip, self.udp_port))
        self.last_send_time = time.time()
        
        return uwb_data
        
    def read_serial_thread(self):
        """Thread to continuously read from serial port"""
        buffer = ""
        while self.running:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting).decode("utf-8", errors="ignore")
                    buffer += data
                    
                    # Process complete lines
                    lines = buffer.split('\n')
                    buffer = lines.pop()  # Keep incomplete line
                    
                    for line in lines:
                        line = line.strip()
                        if line:
                            self.process_data(line)
                else:
                    # Small pause to prevent CPU overuse
                    time.sleep(0.001)
            except Exception as e:
                print(f"Error in serial thread: {e}")
                time.sleep(0.1)
                
    def process_thread(self):
        """Thread to process data and send at controlled rate"""
        while self.running:
            try:
                # Check if we have data to process
                result = self.process_buffer()
                if result:
                    filtered_values, raw_values, movement_detected = result
                    
                    # Determine if it's time to send
                    now = time.time()
                    time_since_last = now - self.last_send_time
                    
                    # Send at higher rate during movement, otherwise throttle
                    min_interval = self.min_send_interval / 2 if movement_detected else self.min_send_interval
                    
                    if time_since_last >= min_interval:
                        # Send data
                        uwb_data = self.send_data(filtered_values)
                        
                        # Format raw values for display
                        raw_formatted = [
                            f"{raw_values[key]:.3f}" if raw_values.get(key) is not None else "null"
                            for key in ['A0', 'A1', 'A2']
                        ]
                        raw_data = ",".join(raw_formatted)
                        
                        # Display status
                        status = "MOVING" if movement_detected else "STABLE"
                        print(f"Raw: {raw_data} | Filtered: {uwb_data} | Status: {status}")
                
                # Small pause to prevent CPU overuse
                time.sleep(0.005)
            except Exception as e:
                print(f"Error in process thread: {e}")
                time.sleep(0.1)
    
    def run(self):
        if not self.connect():
            return
            
        # Start reading thread
        self.running = True
        read_thread = threading.Thread(target=self.read_serial_thread)
        read_thread.daemon = True
        read_thread.start()
        
        # Start processing thread
        process_thread = threading.Thread(target=self.process_thread)
        process_thread.daemon = True
        process_thread.start()
        
        try:
            # Keep main thread alive
            while self.running:
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("Transmitter stopped by user.")
        finally:
            self.running = False
            time.sleep(0.2)  # Give threads time to close
            self.disconnect()

def main_uwb_transmitter():
    # Configuration
    serial_port = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
    udp_ip = "192.168.80.113"  # IP Raspberry Pi 2
    udp_port = 5005           # Port untuk komunikasi
    
    # Create and run transmitter
    transmitter = UWBTransmitter(serial_port, udp_ip, udp_port)
    transmitter.run()

if __name__ == "__main__":
    main_uwb_transmitter()
