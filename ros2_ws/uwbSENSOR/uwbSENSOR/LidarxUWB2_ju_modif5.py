#!/usr/bin/env python3

import socket
import numpy as np
import math
import time
import threading
import sys
import os
import select
import netifaces

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from ddsm115 import MotorControl
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# Import gpiozero for controlling GPIO pins
from gpiozero import LED

# ==================== KONFIGURASI YANG DIPERBAIKI ====================
SCAN_THRESHOLD = 800  # 80cm
DANGER_THRESHOLD = 400  # 40cm
MIN_VALID_DISTANCE = 100
CRITICAL_DANGER_THRESHOLD = 200  # 20cm

# PERBAIKAN: Speed dan timing yang lebih terkontrol
DEFAULT_SPEED = 60
ROTATION_FACTOR = 1.8
STOP_THRESHOLD = 80  # cm

# PERBAIKAN: Timing untuk escape mechanism
TURN_DURATION = 1.5  # Durasi maksimal untuk satu putaran (detik)
BACKUP_DURATION = 1.0  # Durasi backup (detik)
FORWARD_CHECK_DURATION = 0.5  # Durasi cek forward setelah turn

CONTROL_FREQUENCY = 30  # Hz - lebih stabil
LIDAR_SKIP_FRAMES = 1
UWB_TIMEOUT = 0.1
UWB_DATA_TIMEOUT = 2.0
MAX_LOOP_TIME = 0.03

LIDAR_BUFFER_SIZE = 1
UWB_BUFFER_SIZE = 1024

# Definisi sudut untuk deteksi rintangan
FRONT_REGION = [(330, 360), (0, 30)]
RIGHT_REGION = (31, 140)
LEFT_REGION = (220, 329)
TARGET_EXCLUSION_ANGLE = 10

# GPIO Pins Initialization
gpio_pin_17 = LED(17)
gpio_pin_27 = LED(27)

# Dynamic IP functions
def get_ip_from_wifi(interface='wlan0'):
    try:
        ip = netifaces.ifaddresses(interface)[netifaces.AF_INET][0]['addr']
        return ip
    except (KeyError, ValueError):
        print(f"Failed to get IP address for interface: {interface}")
        return None
    
def get_ip_from_subnet(ip, target_last_digit):
    ip_parts = ip.split(".")
    ip_parts[-1] = str(target_last_digit)
    return ".".join(ip_parts)

class UWBTracker:
    def __init__(self):
        self.bias = {
            'A0': 50.0,
            'A1': 50.0,
            'A2': 50.0
        }
        
        self.scale_factor = {
            'A0': 1.0,
            'A1': 1.1,
            'A2': 1.1
        }
        
        self.target_direction = None
        self.target_distance = None
    
    def apply_bias_correction(self, distances):
        corrected_distances = {
            'A0': max((distances['A0'] * 100 * self.scale_factor['A0']) - self.bias['A0'], 0),
            'A1': max((distances['A1'] * 100 * self.scale_factor['A1']) - self.bias['A1'], 0),
            'A2': max((distances['A2'] * 100 * self.scale_factor['A2']) - self.bias['A2'], 0)
        }
        return corrected_distances
    
    def estimate_target_direction(self, distances):
        A0, A1, A2 = distances['A0'], distances['A1'], distances['A2']
        target_distance = A0 * 10
        
        diff = A2 - A1
        if abs(diff) < 20:
            target_direction = 270
        elif diff < 0:
            angle_offset = min(45, abs(diff) * 1.5)
            target_direction = 270 + angle_offset
        else:
            angle_offset = min(45, abs(diff) * 1.5)
            target_direction = 270 - angle_offset
        
        self.target_direction = target_direction
        self.target_distance = target_distance
        
        return target_direction, target_distance

class LidarProcessor:
    def __init__(self):
        self.scan_data = {}
        self.lock = threading.Lock()
        
        # Status rintangan
        self.front_obstacle = False
        self.left_obstacle = False
        self.right_obstacle = False
        self.danger_zone = False
        self.critical_danger = False
        
        # Jarak minimum di setiap region
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        
        self.last_scan_time = 0
        self.last_scan_msg = None
        self.target_direction = None
        self.target_distance = None
        
        # PERBAIKAN: Enhanced clearance tracking
        self.clearance_history = {'LEFT': [], 'RIGHT': [], 'FRONT': []}
        self.history_length = 5
    
    def set_target_info(self, direction, distance):
        self.target_direction = direction
        self.target_distance = distance
    
    def process_scan(self, scan_msg):
        with self.lock:
            self.last_scan_msg = scan_msg
            self.last_scan_time = time.time()
            self.scan_data.clear()
            
            ranges = scan_msg.ranges
            angle_increment = scan_msg.angle_increment
            angle_min = scan_msg.angle_min
            
            step_size = 5
            
            for i in range(0, len(ranges), step_size):
                distance = ranges[i]
                
                if distance < 0.01 or distance > 10.0 or math.isinf(distance):
                    continue
                
                angle_rad = angle_min + (i * angle_increment)
                angle_deg = int(math.degrees(angle_rad) % 360)
                distance_mm = int(distance * 1000)
                
                self.scan_data[angle_deg] = distance_mm

            self._analyze_obstacles_with_clearance_tracking()
    
    def _is_angle_in_region(self, angle, region):
        if isinstance(region, tuple):
            start, end = region
            if start <= end:
                return start <= angle <= end
            else:
                return angle >= start or angle <= end
        else:
            for r in region:
                start, end = r
                if start <= end:
                    if start <= angle <= end:
                        return True
                else:
                    if angle >= start or angle <= end:
                        return True
            return False
    
    def _analyze_obstacles_with_clearance_tracking(self):
        """PERBAIKAN: Analisis obstacle dengan tracking clearance untuk mencegah infinite loop"""
        if not self.scan_data:
            return
        
        # Reset status
        self.front_obstacle = self.left_obstacle = self.right_obstacle = False
        self.danger_zone = self.critical_danger = False
        self.front_distance = self.left_distance = self.right_distance = float('inf')
        
        # Pre-compute region checks
        front_angles = set(range(330, 360)) | set(range(0, 30))
        right_angles = set(range(30, 151))
        left_angles = set(range(210, 330))
        
        for angle, distance in self.scan_data.items():
            # Skip target exclusion
            if (self.target_direction is not None and 
                abs(angle - self.target_direction) < TARGET_EXCLUSION_ANGLE):
                continue
            
            if angle in front_angles:
                if distance < self.front_distance:
                    self.front_distance = distance
                if distance < SCAN_THRESHOLD:
                    self.front_obstacle = True
                if distance < DANGER_THRESHOLD:
                    self.danger_zone = True
                if distance < CRITICAL_DANGER_THRESHOLD:
                    self.critical_danger = True
                    
            elif angle in right_angles:
                if distance < self.right_distance:
                    self.right_distance = distance
                if distance < SCAN_THRESHOLD:
                    self.right_obstacle = True
                    
            elif angle in left_angles:
                if distance < self.left_distance:
                    self.left_distance = distance
                if distance < SCAN_THRESHOLD:
                    self.left_obstacle = True
        
        # PERBAIKAN: Update clearance history untuk tracking
        self._update_clearance_history()
    
    def _update_clearance_history(self):
        """Update riwayat clearance untuk setiap arah"""
        current_clearances = {
            'FRONT': self.front_distance,
            'LEFT': self.left_distance,
            'RIGHT': self.right_distance
        }
        
        for direction, clearance in current_clearances.items():
            self.clearance_history[direction].append(clearance)
            if len(self.clearance_history[direction]) > self.history_length:
                self.clearance_history[direction].pop(0)
    
    def get_average_clearance(self, direction):
        """Dapatkan rata-rata clearance untuk arah tertentu"""
        if direction in self.clearance_history and self.clearance_history[direction]:
            return sum(self.clearance_history[direction]) / len(self.clearance_history[direction])
        return float('inf')
    
    def is_clearance_improving(self, direction):
        """Cek apakah clearance arah tertentu sedang membaik"""
        history = self.clearance_history.get(direction, [])
        if len(history) < 3:
            return False
        
        # Cek apakah 3 measurement terakhir menunjukkan peningkatan
        recent = history[-3:]
        return recent[-1] > recent[0]
    
    def get_obstacle_status(self):
        with self.lock:
            scan_age = time.time() - self.last_scan_time
            data_valid = scan_age < 1.0
            
            status = {
                'front': {
                    'obstacle': self.front_obstacle if data_valid else False,
                    'distance': self.front_distance if data_valid else float('inf'),
                    'avg_clearance': self.get_average_clearance('FRONT'),
                    'improving': self.is_clearance_improving('FRONT')
                },
                'left': {
                    'obstacle': self.left_obstacle if data_valid else False,
                    'distance': self.left_distance if data_valid else float('inf'),
                    'avg_clearance': self.get_average_clearance('LEFT'),
                    'improving': self.is_clearance_improving('LEFT')
                },
                'right': {
                    'obstacle': self.right_obstacle if data_valid else False,
                    'distance': self.right_distance if data_valid else float('inf'),
                    'avg_clearance': self.get_average_clearance('RIGHT'),
                    'improving': self.is_clearance_improving('RIGHT')
                },
                'danger_zone': self.danger_zone if data_valid else False,
                'critical_danger': self.critical_danger if data_valid else False,
                'data_valid': data_valid,
                'scan_points': len(self.scan_data) if data_valid else 0,
                'scan_age': scan_age
            }
            
            return status
    
    def get_best_direction_with_clearance_analysis(self):
        """PERBAIKAN: Pilih arah terbaik berdasarkan analisis clearance yang komprehensif"""
        status = self.get_obstacle_status()
        
        if not status['data_valid']:
            print("Warning: LIDAR data not valid - ALLOWING FORWARD")
            return "FORWARD"
        
        if status['critical_danger']:
            return "STOP"
        
        # Analisis clearance untuk setiap arah
        directions = ['FRONT', 'LEFT', 'RIGHT']
        direction_scores = {}
        
        for direction in directions:
            dir_key = direction.lower()
            
            # Score berdasarkan jarak saat ini
            current_distance = status[dir_key]['distance']
            distance_score = min(current_distance / 1000.0, 1.0)  # Normalize ke 0-1
            
            # Score berdasarkan rata-rata clearance
            avg_clearance = status[dir_key]['avg_clearance']
            avg_score = min(avg_clearance / 1000.0, 1.0)
            
            # Bonus jika clearance sedang membaik
            improvement_bonus = 0.2 if status[dir_key]['improving'] else 0
            
            # Penalty jika ada obstacle
            obstacle_penalty = -0.5 if status[dir_key]['obstacle'] else 0
            
            # Total score
            total_score = distance_score + avg_score + improvement_bonus + obstacle_penalty
            direction_scores[direction] = total_score
            
            print(f"{direction}: dist={current_distance:.0f}mm, avg={avg_clearance:.0f}mm, "
                  f"improving={status[dir_key]['improving']}, obstacle={status[dir_key]['obstacle']}, "
                  f"score={total_score:.2f}")
        
        # Pilih arah dengan score tertinggi
        best_direction = max(direction_scores, key=direction_scores.get)
        best_score = direction_scores[best_direction]
        
        print(f"Best direction: {best_direction} (score: {best_score:.2f})")
        
        # Jika score terbaik masih rendah, pertimbangkan backup
        if best_score < 0.3:
            print("All directions have low scores - considering BACKUP")
            return "BACKUP"
        
        return best_direction

class RobotController:
    def __init__(self, r_wheel_port, l_wheel_port):
        self.right_motor = MotorControl(device=r_wheel_port)
        self.left_motor = MotorControl(device=l_wheel_port)
        self.right_motor.set_drive_mode(1, 2)
        self.left_motor.set_drive_mode(1, 2)
        
        self.speed = DEFAULT_SPEED
        self.rotation_factor = ROTATION_FACTOR
        self.stop_threshold = STOP_THRESHOLD
        
        self.obstacle_avoidance_active = False
        self.last_command_time = time.time()
        self.current_direction = "STOP"
        self.emergency_stop = False
        
        # Command batching
        self.last_command = (0, 0)
        self.command_threshold = 3
        self.last_command_time = 0
        self.min_command_interval = 0.05
        
        # PERBAIKAN: State machine untuk mencegah infinite loop
        self.avoidance_state = "IDLE"  # IDLE, TURNING, BACKING, CHECKING, STUCK
        self.state_start_time = time.time()
        self.turn_start_time = 0
        self.last_turn_direction = None
        self.consecutive_same_turns = 0
        self.stuck_counter = 0
        self.max_stuck_count = 5
        
        # PERBAIKAN: Tracking untuk escape mechanism
        self.total_commands_sent = 0
        self.last_movement_time = time.time()
        self.last_successful_forward = time.time()
    
    def move(self, right_speed, left_speed):
        current_time = time.time()
        
        if (abs(right_speed - self.last_command[0]) < self.command_threshold and
            abs(left_speed - self.last_command[1]) < self.command_threshold and
            current_time - self.last_command_time < self.min_command_interval):
            return
        
        # Send command
        self.right_motor.send_rpm(1, int(right_speed))
        self.left_motor.send_rpm(1, int(left_speed))
        
        self.last_command = (right_speed, left_speed)
        self.last_command_time = current_time
        self.total_commands_sent += 1
        
        if abs(right_speed) > 0 or abs(left_speed) > 0:
            self.last_movement_time = current_time
        
        print(f"MOTOR: R={right_speed:.1f}, L={left_speed:.1f}, State={self.avoidance_state}")
    
    def stop(self):
        self.move(0, 0)
        self.current_direction = "STOP"
        print("ROBOT STOPPED")
    
    def check_critical_obstacles(self, lidar):
        status = lidar.get_obstacle_status()
        
        if status['data_valid'] and status['critical_danger']:
            print("CRITICAL DANGER DETECTED - EMERGENCY STOP")
            self.stop()
            self.emergency_stop = True
            self.avoidance_state = "STUCK"
            self.state_start_time = time.time()
            return True
        
        self.emergency_stop = False
        return False
    
    def execute_state_machine_avoidance(self, lidar, target_in_view=False):
        """PERBAIKAN: State machine untuk obstacle avoidance yang mencegah infinite loop"""
        status = lidar.get_obstacle_status()
        current_time = time.time()
        state_duration = current_time - self.state_start_time
        
        print(f"\n=== AVOIDANCE STATE MACHINE ===")
        print(f"State: {self.avoidance_state}, Duration: {state_duration:.1f}s")
        print(f"Stuck counter: {self.stuck_counter}/{self.max_stuck_count}")
        
        # State: IDLE - Normal operation
        if self.avoidance_state == "IDLE":
            if not status['data_valid']:
                return False
            
            # Check if we need to start avoidance
            if (status['danger_zone'] or 
                (status['front']['obstacle'] and status['front']['distance'] < DANGER_THRESHOLD * 1.5)):
                
                print("Entering avoidance mode")
                self.avoidance_state = "TURNING"
                self.state_start_time = current_time
                self.turn_start_time = current_time
                return True
            
            return False
        
        # State: TURNING - Robot is turning to avoid obstacle
        elif self.avoidance_state == "TURNING":
            turn_duration = current_time - self.turn_start_time
            
            # Timeout check for turning
            if turn_duration > TURN_DURATION:
                print("Turn timeout - checking result")
                self.avoidance_state = "CHECKING"
                self.state_start_time = current_time
                return True
            
            # Get best direction and execute turn
            best_direction = lidar.get_best_direction_with_clearance_analysis()
            
            if best_direction == "STOP":
                print("All directions blocked - backing up")
                self.avoidance_state = "BACKING"
                self.state_start_time = current_time
                self.move(self.speed//2, -self.speed//2)  # Backup
                return True
            
            elif best_direction == "BACKUP":
                print("Low clearance everywhere - backing up")
                self.avoidance_state = "BACKING"
                self.state_start_time = current_time
                self.move(self.speed//2, -self.speed//2)  # Backup
                return True
            
            elif best_direction == "LEFT":
                print("Turning LEFT")
                self.move(-self.speed, self.speed//self.rotation_factor)
                self.current_direction = "LEFT"
                self._track_turn_direction("LEFT")
                return True
                
            elif best_direction == "RIGHT":
                print("Turning RIGHT")
                self.move(-self.speed//self.rotation_factor, self.speed)
                self.current_direction = "RIGHT"
                self._track_turn_direction("RIGHT")
                return True
                
            elif best_direction == "FORWARD":
                print("Path cleared - resuming forward")
                self.avoidance_state = "IDLE"
                self.state_start_time = current_time
                self.stuck_counter = 0
                return False
            
            return True
        
        # State: BACKING - Robot is backing up
        elif self.avoidance_state == "BACKING":
            if state_duration > BACKUP_DURATION:
                print("Backup complete - trying turn again")
                self.avoidance_state = "TURNING"
                self.state_start_time = current_time
                self.turn_start_time = current_time
                return True
            
            # Continue backing up
            self.move(self.speed//2, -self.speed//2)
            return True
        
        # State: CHECKING - Check if turn was successful
        elif self.avoidance_state == "CHECKING":
            if state_duration > FORWARD_CHECK_DURATION:
                # Check if path is now clear
                if not status['front']['obstacle'] and not status['danger_zone']:
                    print("Turn successful - path clear")
                    self.avoidance_state = "IDLE"
                    self.state_start_time = current_time
                    self.stuck_counter = 0
                    self.last_successful_forward = current_time
                    return False
                else:
                    print("Turn unsuccessful - trying again")
                    self.stuck_counter += 1
                    if self.stuck_counter >= self.max_stuck_count:
                        print("Max stuck count reached - entering stuck recovery")
                        self.avoidance_state = "STUCK"
                        self.state_start_time = current_time
                    else:
                        self.avoidance_state = "TURNING"
                        self.turn_start_time = current_time
                    self.state_start_time = current_time
                    return True
            
            # Brief forward movement to test clearance
            self.move(-self.speed//3, self.speed//3)
            return True
        
        # State: STUCK - Robot is stuck, try recovery
        elif self.avoidance_state == "STUCK":
            if state_duration > 3.0:  # 3 second recovery
                print("Stuck recovery complete - resetting")
                self.avoidance_state = "IDLE"
                self.state_start_time = current_time
                self.stuck_counter = 0
                return False
            
            # Recovery maneuver: alternate backing and turning
            if int(state_duration * 2) % 2 == 0:
                print("Stuck recovery: backing")
                self.move(self.speed//2, -self.speed//2)
            else:
                print("Stuck recovery: turning")
                # Alternate turn direction for recovery
                if self.stuck_counter % 2 == 0:
                    self.move(-self.speed//2, self.speed//2)  # Turn left
                else:
                    self.move(-self.speed//2, self.speed//2)  # Turn right
            return True
        
        return False
    
    def _track_turn_direction(self, direction):
        """Track consecutive turns in same direction"""
        if direction == self.last_turn_direction:
            self.consecutive_same_turns += 1
        else:
            self.consecutive_same_turns = 1
        
        self.last_turn_direction = direction
        
        # Warning if too many consecutive same turns
        if self.consecutive_same_turns >= 4:
            print(f"WARNING: {self.consecutive_same_turns} consecutive {direction} turns!")
            self.stuck_counter += 1
    
    def handle_obstacle_avoidance(self, lidar, target_in_view=False):
        """PERBAIKAN: Main obstacle avoidance dengan state machine"""
        status = lidar.get_obstacle_status()
        
        if not status['data_valid']:
            print("LIDAR data invalid - allowing movement")
            return False
        
        # If target is very close, ignore non-critical obstacles
        if target_in_view and lidar.target_distance and lidar.target_distance < STOP_THRESHOLD * 10:
            if not status['critical_danger']:
                print("Target very close - ignoring obstacles")
                return False
        
        # Execute state machine
        avoidance_needed = self.execute_state_machine_avoidance(lidar, target_in_view)
        
        if avoidance_needed:
            self.obstacle_avoidance_active = True
        else:
            self.obstacle_avoidance_active = False
        
        return avoidance_needed
    
    def process_uwb_control(self, uwb_distances):
        """Process UWB data to control robot movement"""
        A0, A1, A2 = uwb_distances['A0'], uwb_distances['A1'], uwb_distances['A2']
        
        print("\n--- UWB Distances (cm) ---")
        print(f"A0: {A0:.2f} | A1: {A1:.2f} | A2: {A2:.2f}")
        
        if A0 <= self.stop_threshold:
            print(f"Target reached (A0 <= {self.stop_threshold} cm). Stopping.")
            self.stop()
            return
        
        diff = A2 - A1
        if abs(diff) < 15:
            print("Move Forward - Target centered")
            self.move(-self.speed, self.speed)
            self.current_direction = "FORWARD"
            self.last_successful_forward = time.time()
        elif A0 < 150:
            if diff < 0:
                turn_ratio = min(1.0, abs(diff) / 40.0)
                right_speed = -self.speed * (1.0 - turn_ratio * 0.8)
                left_speed = self.speed
                print(f"Close target right turn (ratio: {turn_ratio:.2f})")
                self.move(right_speed, left_speed)
                self.current_direction = "RIGHT"
            else:
                turn_ratio = min(1.0, abs(diff) / 40.0)
                right_speed = -self.speed
                left_speed = self.speed * (1.0 - turn_ratio * 0.8)
                print(f"Close target left turn (ratio: {turn_ratio:.2f})")
                self.move(right_speed, left_speed)
                self.current_direction = "LEFT"
        elif (A0 > A1 or A0 > A2):
            print("Rotation")
            self.move(self.speed/self.rotation_factor, self.speed/self.rotation_factor)
        else:
            if diff < 0:
                print("Turn right - target to right")
                self.move(-self.speed/self.rotation_factor, self.speed)
                self.current_direction = "RIGHT"
            else:
                print("Turn left - target to left")
                self.move(-self.speed, self.speed/self.rotation_factor)
                self.current_direction = "LEFT"
    
    def process_control(self, uwb_distances, lidar):
        """Main control logic"""
        if self.check_critical_obstacles(lidar):
            return
        
        target_direction, target_distance = uwb_tracker.estimate_target_direction(uwb_distances)
        lidar.set_target_info(target_direction, target_distance)
        
        target_in_view = (
            uwb_distances['A0'] < 200 and
            abs(uwb_distances['A1'] - uwb_distances['A2']) < 30
        )
        
        obstacle_action_needed = self.handle_obstacle_avoidance(lidar, target_in_view)
        
        if not obstacle_action_needed and not self.emergency_stop:
            # Reset avoidance state when resuming UWB control
            if self.avoidance_state != "IDLE":
                print("Resuming UWB control - resetting avoidance state")
                self.avoidance_state = "IDLE"
                self.stuck_counter = 0
            
            self.process_uwb_control(uwb_distances)

class FollowingRobotNode(Node):
    def __init__(self):
        super().__init__('following_robot_node')

        self.udp_ip = get_ip_from_wifi()
        if not self.udp_ip:
            raise ValueError("Unable to retrieve IP address from Wi-Fi interface.")

        target_last_digit = 128
        self.udp_ip = get_ip_from_subnet(self.udp_ip, target_last_digit)

        print(f"Robot will use dynamic IP: {self.udp_ip}")
        print(f"ANTI-INFINITE-LOOP: Enhanced obstacle avoidance with state machine")
        
        self.udp_port = 5005
        
        self.r_wheel_port = "/dev/ttyRS485-1"
        self.l_wheel_port = "/dev/ttyRS485-2"
        
        self.control_frequency = CONTROL_FREQUENCY
        self.lidar_skip_frames = LIDAR_SKIP_FRAMES
        self.uwb_timeout = UWB_TIMEOUT
        self.max_loop_time = MAX_LOOP_TIME
        
        self.lidar_frame_counter = 0
        self.last_control_update = 0
        self.last_loop_time = 0
        self._processing_lidar = False
        
        self.uwb_tracker = UWBTracker()
        self.lidar = LidarProcessor()
        self.controller = RobotController(self.r_wheel_port, self.l_wheel_port)

        self.lidar_cb_group = ReentrantCallbackGroup()
        
        lidar_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=LIDAR_BUFFER_SIZE
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback_optimized,
            qos_profile=lidar_qos,
            callback_group=self.lidar_cb_group
        )
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, UWB_BUFFER_SIZE)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.settimeout(self.uwb_timeout)
        
        self.raw_uwb_distances = {'A0': 1000, 'A1': 1000, 'A2': 1000}
        self.corrected_uwb_distances = {'A0': 1000, 'A1': 1000, 'A2': 1000}
        
        self.running = True
        self.last_uwb_update = 0
        
        self.control_timer = self.create_timer(
            1.0/self.control_frequency,
            self.control_loop
        )
        
        self.last_lidar_time = time.time()
        self.lidar_update_count = 0
        self.lidar_update_rate = 0
        
        self.stats_timer = self.create_timer(5.0, self.log_statistics)
        
        self.get_logger().info(f'Following Robot Node started with {self.control_frequency}Hz control frequency')

        gpio_pin_17.on()

    def scan_callback_optimized(self, msg):
        if hasattr(self, '_processing_lidar') and self._processing_lidar:
            return
        
        self._processing_lidar = True
        try:
            if self.lidar_frame_counter % self.lidar_skip_frames == 0:
                self.lidar.process_scan(msg)
            self.lidar_frame_counter += 1
            self.lidar_update_count += 1
        finally:
            self._processing_lidar = False

    def check_critical_obstacles_fast(self):
        if (time.time() - self.lidar.last_scan_time > 0.1 or 
            not hasattr(self.lidar, 'critical_danger')):
            return False
        
        if self.lidar.critical_danger:
            self.controller.stop()
            return True
        return False

    def process_uwb_data_fast(self):
        try:
            self.sock.setblocking(False)
            ready = select.select([self.sock], [], [], self.uwb_timeout)
            if ready[0]:
                data, addr = self.sock.recvfrom(UWB_BUFFER_SIZE)
                parts = data.decode().split(",")
                
                if len(parts) >= 3:
                    self.raw_uwb_distances = {
                        'A0': float(parts[0]),
                        'A1': float(parts[1]),
                        'A2': float(parts[2])
                    }
                    
                    self.corrected_uwb_distances = self.uwb_tracker.apply_bias_correction(
                        self.raw_uwb_distances
                    )
                    self.last_uwb_update = time.time()
                    return True
                    
        except (socket.error, ValueError, IndexError):
            pass
        
        return False

    def execute_control_decision(self):
        uwb_data_age = time.time() - self.last_uwb_update
        uwb_data_valid = uwb_data_age < UWB_DATA_TIMEOUT
        
        print(f"\n=== CONTROL DECISION ===")
        print(f"UWB data age: {uwb_data_age:.2f}s, Valid: {uwb_data_valid}")
        print(f"Avoidance state: {self.controller.avoidance_state}")
        print(f"Stuck counter: {self.controller.stuck_counter}")
        
        if uwb_data_valid:
            print("Using UWB control")
            self.controller.process_control(self.corrected_uwb_distances, self.lidar)
        else:
            print("UWB data invalid - basic obstacle avoidance")
            obstacle_detected = self.controller.handle_obstacle_avoidance(self.lidar)
            if not obstacle_detected and self.controller.avoidance_state == "IDLE":
                print("No obstacles - slow forward movement")
                self.controller.move(-25, 25)
    
    def log_statistics(self):
        current_time = time.time()
        update_interval = current_time - self.last_lidar_time
        if update_interval > 0:
            self.lidar_update_rate = self.lidar_update_count / update_interval
            self.last_lidar_time = current_time
            self.lidar_update_count = 0
            
            self.get_logger().info(f"=== ROBOT STATUS ===")
            self.get_logger().info(f"LIDAR update rate: {self.lidar_update_rate:.2f} Hz")
            self.get_logger().info(f"UWB data age: {time.time() - self.last_uwb_update:.2f}s")
            self.get_logger().info(f"Avoidance state: {self.controller.avoidance_state}")
            self.get_logger().info(f"Stuck counter: {self.controller.stuck_counter}")
            self.get_logger().info(f"Last successful forward: {time.time() - self.controller.last_successful_forward:.1f}s ago")
            
            # Warning untuk infinite loop
            if (self.controller.avoidance_state == "TURNING" and 
                time.time() - self.controller.state_start_time > 10):
                self.get_logger().warn("POSSIBLE INFINITE TURN LOOP DETECTED!")
    
    def control_loop(self):
        if not self.running:
            return
        start_time = time.time()
        try:
            if self.check_critical_obstacles_fast():
                return
            uwb_updated = self.process_uwb_data_fast()
            if uwb_updated or time.time() - self.last_control_update > 0.1:
                self.execute_control_decision()
                self.last_control_update = time.time()
        except Exception as e:
            gpio_pin_27.on()
            self.get_logger().error(f"Control loop error: {e}")
        
        loop_time = time.time() - start_time
        if loop_time > self.max_loop_time:
            self.get_logger().warn(f"Slow control loop: {loop_time*1000:.1f}ms")

    def stop(self):
        self.running = False
        self.controller.stop()
        self.sock.close()
        gpio_pin_17.off()
        gpio_pin_27.off()
        self.get_logger().info("Robot systems shut down.")

def main(args=None):
    rclpy.init(args=args)
    
    global uwb_tracker
    uwb_tracker = UWBTracker()
    
    node = FollowingRobotNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("\nUser interrupted. Shutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
