
import socket
import numpy as np
import math
import time
from ddsm115 import MotorControl
import threading

# Konfigurasi parameter
speed = 80
rotasi = 2
batas = 60

# Mode debugging - set ke False untuk mengurangi delay dari printing
DEBUG = False

# Parameter penghalusan gerakan
SMOOTHING_FACTOR = 0.3  # Faktor pembobotan untuk kecepatan baru (0-1)
MIN_SPEED_CHANGE = 5    # Perubahan kecepatan minimal untuk memulai transisi

class UWBTracker:
    """Handles UWB data processing and position estimation"""
    
    def __init__(self):  # Fixed: Changed init to __init__
        # Default bias correction values
        self.bias = {
            'A0': 50.0,  # Example bias value in cm
            'A1': 50.0,  # Example bias value in cm
            'A2': 50.0   # Example bias value in cm
        }
        
        # Default scale factor values
        self.scale_factor = {
            'A0': 1.0,  # Example scale factor
            'A1': 1.0,  # Example scale factor
            'A2': 1.06  # Example scale factor
        }
        
        # Untuk penghalusan pembacaan sensor
        self.prev_distances = {'A0': 0, 'A1': 0, 'A2': 0}
        self.smoothing_weight = 0.7  # Bobot untuk pembacaan saat ini (0-1)
    
    def apply_bias_correction(self, distances):
        """Koreksi bias dan scaling pada pengukuran jarak dengan penghalusan"""
        # Hitung jarak terkoreksi
        corrected_distances = {
            'A0': max((distances['A0'] * 100 * self.scale_factor['A0']) - self.bias['A0'], 0),
            'A1': max((distances['A1'] * 100 * self.scale_factor['A1']) - self.bias['A1'], 0),
            'A2': max((distances['A2'] * 100 * self.scale_factor['A2']) - self.bias['A2'], 0)
        }
        
        # Terapkan penghalusan untuk mengurangi loncatan nilai
        smoothed_distances = {}
        for sensor, value in corrected_distances.items():
            # Weighted average dengan pembacaan sebelumnya
            smoothed_value = (self.smoothing_weight * value) + ((1 - self.smoothing_weight) * self.prev_distances[sensor])
            smoothed_distances[sensor] = smoothed_value
            self.prev_distances[sensor] = smoothed_value
            
        return smoothed_distances

class RobotController:
    """Controls the robot's movement based on UWB data with smooth transitions"""
    
    def __init__(self, r_wheel_port, l_wheel_port):  # Fixed: Changed init to __init__
        self.right_motor = MotorControl(device=r_wheel_port)
        self.left_motor = MotorControl(device=l_wheel_port)
        self.right_motor.set_drive_mode(1, 2)
        self.left_motor.set_drive_mode(1, 2)
        
        # Untuk penghalusan pergerakan
        self.current_right_speed = 0
        self.current_left_speed = 0
        self.target_right_speed = 0
        self.target_left_speed = 0
        
        # Status terakhir untuk mengurangi perintah berulang
        self.last_status = ""
        
        # Hysteresis untuk mengurangi osilasi
        self.hysteresis = 5  # cm
        
        # Flag untuk indikasi jika robot sedang bergerak
        self.is_moving = False
    
    def smooth_move(self, right_speed, left_speed):
        """Set target speeds for smooth acceleration/deceleration"""
        # Hanya update target jika berbeda secara signifikan
        if (abs(right_speed - self.target_right_speed) > MIN_SPEED_CHANGE or
            abs(left_speed - self.target_left_speed) > MIN_SPEED_CHANGE):
            self.target_right_speed = right_speed
            self.target_left_speed = left_speed
    
    def update_speed(self):
        """Gradually update current speed towards target speed"""
        # Hitung selisih kecepatan saat ini dengan target
        right_diff = self.target_right_speed - self.current_right_speed
        left_diff = self.target_left_speed - self.current_left_speed
        
        # Terapkan perubahan kecepatan secara bertahap
        self.current_right_speed += right_diff * SMOOTHING_FACTOR
        self.current_left_speed += left_diff * SMOOTHING_FACTOR
        
        # Kirim perintah kecepatan ke motor
        self.right_motor.send_rpm(1, int(self.current_right_speed))
        self.left_motor.send_rpm(1, int(self.current_left_speed))
        
        # Indikasi gerakan
        self.is_moving = abs(self.current_right_speed) > 1 or abs(self.current_left_speed) > 1
    
    def move(self, right_speed, left_speed):
        """Move the robot with immediate response"""
        # Untuk kasus yang membutuhkan respons segera, gunakan direct movement
        self.right_motor.send_rpm(1, right_speed)
        self.left_motor.send_rpm(1, left_speed)
        self.current_right_speed = right_speed
        self.current_left_speed = left_speed
        self.target_right_speed = right_speed
        self.target_left_speed = left_speed
    
    def stop(self):
        """Stop the robot movement with minimal deceleration"""
        # Untuk penghentian darurat, hentikan langsung
        self.smooth_move(0, 0)  # Set target speeds to zero for smooth stop
    
    def analyze_and_act(self, distances):
        """
        Decide the robot's action based on UWB sensor distances.
        Enhanced for smoother transitions and reduced oscillation.
        """
        A0, A1, A2 = distances['A0'], distances['A1'], distances['A2']
        
        # Hitung perbedaan A1 dan A2 dengan hysteresis untuk mengurangi osilasi
        a1_a2_diff = A1 - A2
        
        # Print distance values only in debug mode
        if DEBUG:
            print("UWB Corrected Distances (cm):")
            print(f"A0: {A0:.2f} | A1: {A1:.2f} | A2: {A2:.2f}")
        
        # Penentuan status untuk debugging
        current_status = ""
        
        # Stop when target reached
        if A0 <= batas:
            current_status = f"TARGET_REACHED (A0 <= {batas} cm)"
            self.stop()
            
        # Forward motion - target directly ahead
        elif A0 < A1 and A0 < A2 and abs(a1_a2_diff) < 13:
            current_status = "FORWARD"
            self.smooth_move(-speed, speed)  # Move forward with equal speed
            
        # Target slightly to the left
        elif (A1 < A2 and A0 < A2):
            current_status = "SERONG_KIRI"
            # Gunakan faktor kecepatan yang lebih halus berdasarkan perbedaan A1-A2
            turn_factor = max(0.3, min(1.0, abs(a1_a2_diff) / 50))
            left_speed = speed * (1 - (turn_factor/rotasi))
            self.smooth_move(-speed, left_speed)
            
        # Target slightly to the right
        elif (A2 < A1 and A0 < A1):
            current_status = "SERONG_KANAN"
            # Gunakan faktor kecepatan yang lebih halus berdasarkan perbedaan A1-A2
            turn_factor = max(0.3, min(1.0, abs(a1_a2_diff) / 50))
            right_speed = speed * (1 - (turn_factor/rotasi))
            self.smooth_move(-right_speed, speed)
            
        # Sharp right turn needed
        elif (A2 < A1 - self.hysteresis):
            current_status = "ROTATE_RIGHT"
            # Kecepatan rotasi yang lebih halus
            rotation_speed = min(speed, max(20, abs(a1_a2_diff)))
            self.smooth_move(0, rotation_speed/rotasi)
            
        # Sharp left turn needed
        elif (A1 < A2 - self.hysteresis):
            current_status = "ROTATE_LEFT"
            # Kecepatan rotasi yang lebih halus
            rotation_speed = min(speed, max(20, abs(a1_a2_diff)))
            self.smooth_move(-rotation_speed/rotasi, 0)
            
        # We're facing away from the target - need to rotate
        elif (A0 > A1 or A0 > A2):
            current_status = "REORIENT"
            rotation_speed = min(speed, max(20, A0/10))
            self.smooth_move(rotation_speed/rotasi, rotation_speed/rotasi)
            
        # Default - maintain current motion
        else:
            current_status = "MAINTAIN"
            # Tidak perlu melakukan apa-apa, biarkan robot mengikuti target saat ini
        
        # Print status hanya jika berubah untuk mengurangi output
        if current_status != self.last_status and DEBUG:
            print(current_status)
            self.last_status = current_status

def main_robot_loop():
    """Main loop with optimized processing and minimal delay"""
    # Communication setup
    r_wheel_port = "/dev/ttyRS485-1"
    l_wheel_port = "/dev/ttyRS485-2"
    UDP_IP = "192.168.102.128"
    UDP_PORT = 5005
    
    # Socket optimization - use larger buffer and non-blocking
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setblocking(0)  # Non-blocking socket operations
    
    # Initialize UWB distances
    raw_uwb_distances = {'A0': 1000, 'A1': 1000, 'A2': 1000}
    if DEBUG:
        print("Initial UWB distances:", raw_uwb_distances)
    
    # Initialize UWB tracker with bias correction
    uwb_tracker = UWBTracker()
    
    # Initialize robot controller
    robot = RobotController(r_wheel_port, l_wheel_port)
    
    # Timing variables
    last_message_time = time.time()
    last_movement_update = time.time()
    update_interval = 0.01  # 10ms interval for movement updates
    
    try:
        print("Starting optimized robot control system...")
        
        while True:
            current_time = time.time()
            
            # Non-blocking UDP receive
            try:
                data, addr = sock.recvfrom(1024)
                last_message_time = current_time
                
                # Decode data
                parts = data.decode().split(",")
                
                # Update raw distances
                raw_uwb_distances['A0'] = float(parts[0])
                raw_uwb_distances['A1'] = float(parts[1])
                raw_uwb_distances['A2'] = float(parts[2])
                
                if DEBUG:
                    print("\n--- New UWB Data ---")
                    print(f"Raw: A0={raw_uwb_distances['A0']}, A1={raw_uwb_distances['A1']}, A2={raw_uwb_distances['A2']}")
                
                # Apply bias correction with smoothing
                corrected_distances = uwb_tracker.apply_bias_correction(raw_uwb_distances)
                
                # Analyze and set new target speeds
                robot.analyze_and_act(corrected_distances)
                
            except BlockingIOError:
                # No data available, continue with movement updates
                pass
            except Exception as e:
                if DEBUG:
                    print(f"Error receiving data: {e}")
            
            # Update movement at regular intervals for smooth acceleration
            if current_time - last_movement_update >= update_interval:
                robot.update_speed()
                last_movement_update = current_time
            
            # Safety check - stop if no data received for 2 seconds
            if current_time - last_message_time > 2.0 and robot.is_moving:
                print("WARNING: No UWB data received for 2 seconds. Stopping for safety.")
                robot.stop()
            
            # Ultra-minimal sleep to prevent CPU overload but maintain responsiveness
            time.sleep(0.001)  # 1ms delay
            
    except KeyboardInterrupt:
        print("Terminating robot process.")
    finally:
        robot.stop()
        sock.close()
        print("All systems shut down.")

if __name__ == "__main__":
    main_robot_loop()
