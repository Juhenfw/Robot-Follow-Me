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
from collections import deque
from scipy.linalg import inv

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

# Konfigurasi Global yang Dapat Diadaptasi
class AdaptiveConfig:
    def __init__(self):
        # Base thresholds
        self.BASE_SCAN_THRESHOLD = 1500
        self.BASE_DANGER_THRESHOLD = 600
        self.BASE_CRITICAL_DANGER_THRESHOLD = 250
        
        # Current adaptive thresholds
        self.SCAN_THRESHOLD = self.BASE_SCAN_THRESHOLD
        self.DANGER_THRESHOLD = self.BASE_DANGER_THRESHOLD
        self.CRITICAL_DANGER_THRESHOLD = self.BASE_CRITICAL_DANGER_THRESHOLD
        
        # Other configurations
        self.MIN_VALID_DISTANCE = 200
        self.DEFAULT_SPEED = 75
        self.ROTATION_FACTOR = 2.0
        self.STOP_THRESHOLD = 80
        
        # Performance configurations
        self.CONTROL_FREQUENCY = 50
        self.LIDAR_SKIP_FRAMES = 1
        self.UWB_TIMEOUT = 0.01
        self.MAX_LOOP_TIME = 0.02
        
        # Buffer sizes
        self.LIDAR_BUFFER_SIZE = 5
        self.UWB_BUFFER_SIZE = 1024
        
        # Region definitions
        self.FRONT_REGION = [(330, 360), (0, 30)]
        self.RIGHT_REGION = (31, 140)
        self.LEFT_REGION = (220, 329)
        self.BACK_REGION = (150, 210)
        self.TARGET_EXCLUSION_ANGLE = 10

# Global configuration instance
config = AdaptiveConfig()

# GPIO Pins Initialization
gpio_pin_17 = LED(17)
gpio_pin_27 = LED(27)

def get_ip_from_wifi(interface='wlan0'):
    """Get the IP address of the Raspberry Pi from the Wi-Fi interface"""
    try:
        ip = netifaces.ifaddresses(interface)[netifaces.AF_INET][0]['addr']
        return ip
    except (KeyError, ValueError):
        print(f"Failed to get IP address for interface: {interface}")
        return None
    
def get_ip_from_subnet(ip, target_last_digit):
    """Modify the last digit of the IP address to match the target_last_digit"""
    ip_parts = ip.split(".")
    ip_parts[-1] = str(target_last_digit)
    return ".".join(ip_parts)

# Adaptive Threshold Manager
class AdaptiveThresholdManager:
    def __init__(self):
        self.performance_history = deque(maxlen=50)
        self.collision_count = 0
        self.false_positive_count = 0
        self.last_update_time = time.time()
        
    def update_thresholds(self, robot_speed, obstacle_density, recent_performance):
        """Update threshold berdasarkan kondisi real-time"""
        current_time = time.time()
        
        # Adaptasi berdasarkan kecepatan robot
        speed_factor = 1.0 + (robot_speed / 100.0) * 0.3
        
        # Adaptasi berdasarkan kepadatan obstacle
        density_factor = 1.0 + (obstacle_density / 10.0) * 0.2
        
        # Adaptasi berdasarkan performa historis
        if len(self.performance_history) > 10:
            avg_performance = sum(self.performance_history) / len(self.performance_history)
            performance_factor = 1.0 + (1.0 - avg_performance) * 0.4
        else:
            performance_factor = 1.0
            
        # Update threshold dengan smoothing
        alpha = 0.1  # Smoothing factor
        new_scan_threshold = int(config.BASE_SCAN_THRESHOLD * speed_factor * density_factor)
        new_danger_threshold = int(config.BASE_DANGER_THRESHOLD * speed_factor * performance_factor)
        new_critical_threshold = int(config.BASE_CRITICAL_DANGER_THRESHOLD * performance_factor)
        
        # Apply smoothing
        config.SCAN_THRESHOLD = int(alpha * new_scan_threshold + (1 - alpha) * config.SCAN_THRESHOLD)
        config.DANGER_THRESHOLD = int(alpha * new_danger_threshold + (1 - alpha) * config.DANGER_THRESHOLD)
        config.CRITICAL_DANGER_THRESHOLD = int(alpha * new_critical_threshold + (1 - alpha) * config.CRITICAL_DANGER_THRESHOLD)
        
        self.last_update_time = current_time
        
        return {
            'scan': config.SCAN_THRESHOLD,
            'danger': config.DANGER_THRESHOLD,
            'critical': config.CRITICAL_DANGER_THRESHOLD
        }
    
    def record_performance(self, collision_occurred, false_positive):
        """Record performance metrics"""
        if collision_occurred:
            self.collision_count += 1
            performance_score = 0.0
        elif false_positive:
            self.false_positive_count += 1
            performance_score = 0.3
        else:
            performance_score = 1.0
            
        self.performance_history.append(performance_score)

# Enhanced Kalman Filter for Sensor Fusion
class RobotKalmanFilter:
    def __init__(self):
        # State vector: [x, y, theta, vx, vy, omega]
        self.state = np.zeros(6)
        self.P = np.eye(6) * 1000  # Covariance matrix
        
        # Process noise
        self.Q = np.diag([0.1, 0.1, 0.05, 0.2, 0.2, 0.1])
        
        # Measurement noise untuk UWB
        self.R_uwb = np.diag([50, 50, 50])
        
        # Measurement noise untuk LIDAR
        self.R_lidar = np.diag([10, 10])
        
        # Anchor positions (sesuaikan dengan setup)
        self.anchor_positions = [(0, 0), (1000, 0), (500, 1000)]  # mm
        
    def predict(self, dt, control_input=None):
        """Prediction step dengan model motion"""
        F = np.array([
            [1, 0, 0, dt, 0, 0],
            [0, 1, 0, 0, dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 0.9, 0, 0],  # Velocity damping
            [0, 0, 0, 0, 0.9, 0],
            [0, 0, 0, 0, 0, 0.9]
        ])
        
        # Predict state
        self.state = F @ self.state
        
        # Add control input if available
        if control_input is not None:
            self.state[3] += control_input[0] * dt  # vx
            self.state[4] += control_input[1] * dt  # vy
            self.state[5] += control_input[2] * dt  # omega
        
        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q
        
    def update_uwb(self, uwb_distances):
        """Update dengan measurement UWB"""
        try:
            # Measurement function untuk UWB (trilateration)
            def h_uwb(state):
                x, y = state[0], state[1]
                distances = []
                for anchor in self.anchor_positions:
                    dist = np.sqrt((x - anchor[0])**2 + (y - anchor[1])**2)
                    distances.append(dist)
                return np.array(distances)
            
            # Jacobian matrix
            def jacobian_uwb(state):
                x, y = state[0], state[1]
                H = np.zeros((3, 6))
                for i, anchor in enumerate(self.anchor_positions):
                    dx = x - anchor[0]
                    dy = y - anchor[1]
                    dist = np.sqrt(dx**2 + dy**2)
                    if dist > 0:
                        H[i, 0] = dx / dist
                        H[i, 1] = dy / dist
                return H
            
            # Kalman update
            z = np.array([uwb_distances['A0'] * 10, uwb_distances['A1'] * 10, uwb_distances['A2'] * 10])  # cm to mm
            h = h_uwb(self.state)
            H = jacobian_uwb(self.state)
            
            y = z - h  # Innovation
            S = H @ self.P @ H.T + self.R_uwb  # Innovation covariance
            K = self.P @ H.T @ np.linalg.inv(S)  # Kalman gain
            
            # Update state dan covariance
            self.state = self.state + K @ y
            self.P = (np.eye(6) - K @ H) @ self.P
            
        except Exception as e:
            print(f"Kalman UWB update error: {e}")
        
    def get_position(self):
        return self.state[0], self.state[1], self.state[2]
    
    def get_velocity(self):
        return self.state[3], self.state[4], self.state[5]

# Parameter Optimizer
class ParameterOptimizer:
    def __init__(self):
        self.parameter_history = {}
        self.performance_metrics = deque(maxlen=100)
        self.optimization_interval = 50
        self.iteration_count = 0
        
        # Parameter yang akan dioptimasi
        self.tunable_params = {
            'rotation_factor': {'min': 1.5, 'max': 3.0, 'current': 2.0},
            'lidar_skip_frames': {'min': 1, 'max': 3, 'current': 1},
            'movement_threshold': {'min': 100, 'max': 200, 'current': 150},
            'static_frames_required': {'min': 5, 'max': 15, 'current': 8}
        }
        
    def evaluate_performance(self, metrics):
        """Evaluasi performa berdasarkan metrics"""
        score = (
            (1.0 - metrics.get('collision_rate', 0)) * 0.4 +
            (1.0 / max(metrics.get('target_reach_time', 1), 1)) * 0.3 +
            metrics.get('path_efficiency', 0.5) * 0.2 +
            (1.0 - metrics.get('false_positive_rate', 0)) * 0.1
        )
        
        self.performance_metrics.append(score)
        return score
        
    def optimize_parameters(self):
        """Optimasi parameter menggunakan simple hill climbing"""
        if len(self.performance_metrics) < 20:
            return self.tunable_params
            
        current_performance = np.mean(list(self.performance_metrics)[-10:])
        
        for param_name, param_info in self.tunable_params.items():
            original_value = param_info['current']
            test_values = [
                max(param_info['min'], original_value * 0.95),
                min(param_info['max'], original_value * 1.05)
            ]
            
            best_value = original_value
            best_score = current_performance
            
            for test_val in test_values:
                estimated_score = self._estimate_performance_change(param_name, test_val)
                if estimated_score > best_score:
                    best_score = estimated_score
                    best_value = test_val
                    
            param_info['current'] = best_value
            
        return self.tunable_params
        
    def _estimate_performance_change(self, param_name, new_value):
        """Estimasi perubahan performa dengan parameter baru"""
        base_performance = np.mean(list(self.performance_metrics)[-5:])
        
        if param_name == 'rotation_factor':
            if new_value < self.tunable_params[param_name]['current']:
                return base_performance * 1.02
            else:
                return base_performance * 0.98
        
        return base_performance

# Enhanced Dynamic Object Detection
class DynamicObjectDetector:
    def __init__(self):
        self.position_history = {}
        self.history_window = 15  # Increased window
        self.movement_threshold = 100  # Reduced threshold for better sensitivity
        self.static_frames_required = 10  # Increased requirement
        self.dynamic_timeout = 3.0  # Reduced timeout
        
        self.current_objects = {}
        self.dynamic_objects = set()
        self.static_objects = set()
        
        self.dynamic_object_detected = False
        self.dynamic_object_last_seen = 0
        self.waiting_for_dynamic_object = False
        
        # Enhanced clustering parameters
        self.cluster_distance_threshold = 250  # mm
        self.min_points_per_cluster = 2
        
    def _cluster_scan_points(self, scan_data):
        """Enhanced object detection using improved clustering"""
        objects = {}
        points = []
        
        # Convert to cartesian coordinates
        for angle, distance in scan_data.items():
            if config.MIN_VALID_DISTANCE < distance < 8000:
                x = distance * math.cos(math.radians(angle))
                y = distance * math.sin(math.radians(angle))
                points.append((x, y, angle, distance))
        
        if not points:
            return objects
        
        # Sort points by angle for sequential processing
        points.sort(key=lambda p: p[2])
        
        clusters = []
        current_cluster = [points[0]]
        
        for i in range(1, len(points)):
            x, y, angle, distance = points[i]
            
            # Check distance to last point in current cluster
            last_x, last_y = current_cluster[-1][0], current_cluster[-1][1]
            dist_to_last = math.sqrt((x - last_x)**2 + (y - last_y)**2)
            
            if dist_to_last < self.cluster_distance_threshold:
                current_cluster.append(points[i])
            else:
                if len(current_cluster) >= self.min_points_per_cluster:
                    clusters.append(current_cluster)
                current_cluster = [points[i]]
        
        # Add last cluster
        if len(current_cluster) >= self.min_points_per_cluster:
            clusters.append(current_cluster)
        
        # Calculate cluster centers
        for i, cluster in enumerate(clusters):
            center_x = sum(p[0] for p in cluster) / len(cluster)
            center_y = sum(p[1] for p in cluster) / len(cluster)
            avg_distance = sum(p[3] for p in cluster) / len(cluster)
            objects[i] = (center_x, center_y, len(cluster), avg_distance)
        
        return objects
    
    def update_object_positions(self, scan_data):
        """Enhanced object position tracking"""
        current_time = time.time()
        objects = self._cluster_scan_points(scan_data)
        
        # Match objects with previous detections using Hungarian algorithm approximation
        matched_objects = {}
        used_prev_ids = set()
        
        for obj_id, (x, y, point_count, distance) in objects.items():
            best_match = None
            min_distance = float('inf')
            
            for prev_id, history in self.position_history.items():
                if prev_id in used_prev_ids or not history:
                    continue
                    
                last_pos = history[-1]['position']
                match_distance = math.sqrt((x - last_pos[0])**2 + (y - last_pos[1])**2)
                
                if match_distance < min_distance and match_distance < 400:  # Increased threshold
                    min_distance = match_distance
                    best_match = prev_id
            
            if best_match is not None:
                matched_objects[best_match] = (x, y, point_count, distance)
                used_prev_ids.add(best_match)
            else:
                new_id = max(self.position_history.keys(), default=-1) + 1
                matched_objects[new_id] = (x, y, point_count, distance)
        
        # Update position history
        for obj_id, (x, y, point_count, distance) in matched_objects.items():
            if obj_id not in self.position_history:
                self.position_history[obj_id] = []
            
            self.position_history[obj_id].append({
                'position': (x, y),
                'timestamp': current_time,
                'point_count': point_count,
                'distance': distance
            })
            
            if len(self.position_history[obj_id]) > self.history_window:
                self.position_history[obj_id].pop(0)
        
        # Clean up old objects
        objects_to_remove = []
        for obj_id, history in self.position_history.items():
            if not history or current_time - history[-1]['timestamp'] > 3.0:
                objects_to_remove.append(obj_id)
        
        for obj_id in objects_to_remove:
            del self.position_history[obj_id]
            self.dynamic_objects.discard(obj_id)
            self.static_objects.discard(obj_id)
        
        self._classify_objects(current_time)
        return matched_objects
    
    def _classify_objects(self, current_time):
        """Enhanced object classification"""
        self.dynamic_objects.clear()
        self.static_objects.clear()
        
        for obj_id, history in self.position_history.items():
            if len(history) < 5:
                continue
            
            # Calculate movement statistics
            movements = []
            velocities = []
            
            for i in range(1, len(history)):
                prev_pos = history[i-1]['position']
                curr_pos = history[i]['position']
                dt = history[i]['timestamp'] - history[i-1]['timestamp']
                
                movement = math.sqrt((curr_pos[0] - prev_pos[0])**2 + (curr_pos[1] - prev_pos[1])**2)
                movements.append(movement)
                
                if dt > 0:
                    velocity = movement / dt
                    velocities.append(velocity)
            
            if not movements:
                continue
            
            # Enhanced classification criteria
            avg_movement = np.mean(movements)
            max_movement = max(movements)
            movement_std = np.std(movements)
            avg_velocity = np.mean(velocities) if velocities else 0
            
            # Dynamic if: high average movement OR high max movement OR consistent velocity
            is_dynamic = (
                avg_movement > self.movement_threshold or
                max_movement > self.movement_threshold * 2 or
                avg_velocity > 50 or  # mm/s
                movement_std > self.movement_threshold * 0.5
            )
            
            # Static if: consistently low movement over sufficient time
            is_static = (
                len(history) >= self.static_frames_required and
                avg_movement < self.movement_threshold * 0.3 and
                max_movement < self.movement_threshold * 0.8 and
                avg_velocity < 20
            )
            
            if is_dynamic:
                self.dynamic_objects.add(obj_id)
            elif is_static:
                self.static_objects.add(obj_id)
    
    def check_dynamic_objects_in_path(self, scan_data):
        """Enhanced dynamic object detection in path"""
        current_time = time.time()
        objects = self.update_object_positions(scan_data)
        
        dynamic_in_path = False
        closest_dynamic_distance = float('inf')
        
        for obj_id in self.dynamic_objects:
            if obj_id in self.position_history:
                history = self.position_history[obj_id]
                if history:
                    x, y = history[-1]['position']
                    distance = math.sqrt(x**2 + y**2)
                    angle = math.degrees(math.atan2(y, x)) % 360
                    
                    # Check if dynamic object is in critical path
                    if self._is_in_front_region(angle) and distance < config.SCAN_THRESHOLD:
                        dynamic_in_path = True
                        closest_dynamic_distance = min(closest_dynamic_distance, distance)
                        self.dynamic_object_detected = True
                        self.dynamic_object_last_seen = current_time
                        print(f"DYNAMIC OBJECT in path: {distance:.0f}mm at {angle:.0f}°")
        
        # Enhanced timeout logic
        if self.dynamic_object_detected:
            if current_time - self.dynamic_object_last_seen > self.dynamic_timeout:
                print("Dynamic object timeout - resuming")
                self.dynamic_object_detected = False
                self.waiting_for_dynamic_object = False
            elif not dynamic_in_path:
                print("Dynamic object cleared - resuming")
                self.dynamic_object_detected = False
                self.waiting_for_dynamic_object = False
            else:
                self.waiting_for_dynamic_object = True
        
        return self.waiting_for_dynamic_object, closest_dynamic_distance
    
    def _is_in_front_region(self, angle):
        """Check if angle is in front region"""
        return (330 <= angle <= 360) or (0 <= angle <= 30)

# Enhanced UWB Tracker
class UWBTracker:
    def __init__(self):
        # Adaptive bias correction
        self.bias = {'A0': 50.0, 'A1': 50.0, 'A2': 50.0}
        self.scale_factor = {'A0': 1.0, 'A1': 1.03, 'A2': 1.02}
        
        # Kalman filter for UWB data
        self.uwb_filter = {'A0': deque(maxlen=5), 'A1': deque(maxlen=5), 'A2': deque(maxlen=5)}
        
        self.target_direction = None
        self.target_distance = None
        
        # Calibration data
        self.calibration_samples = deque(maxlen=100)
        self.auto_calibrate = True
        
    def apply_bias_correction(self, distances):
        """Enhanced bias correction with adaptive filtering"""
        corrected_distances = {}
        
        for anchor in ['A0', 'A1', 'A2']:
            # Apply bias and scale correction
            raw_distance = distances[anchor] * 100  # m to cm
            corrected = max((raw_distance * self.scale_factor[anchor]) - self.bias[anchor], 0)
            
            # Apply Kalman-like filtering
            self.uwb_filter[anchor].append(corrected)
            if len(self.uwb_filter[anchor]) >= 3:
                # Use median filter to remove outliers
                filtered_values = sorted(list(self.uwb_filter[anchor]))
                corrected_distances[anchor] = filtered_values[len(filtered_values)//2]
            else:
                corrected_distances[anchor] = corrected
                
        return corrected_distances
    
    def estimate_target_direction(self, distances):
        """Enhanced target direction estimation"""
        A0, A1, A2 = distances['A0'], distances['A1'], distances['A2']
        
        target_distance = A0 * 10  # cm to mm
        
        # Enhanced direction estimation using triangulation principles
        diff = A2 - A1
        ratio = abs(diff) / max(A0, 1)  # Normalize by distance
        
        if abs(diff) < 15:  # Target centered
            target_direction = 0
        elif diff < 0:  # Target to the right
            angle_offset = min(60, ratio * 45)
            target_direction = 90 + angle_offset
        else:  # Target to the left
            angle_offset = min(60, ratio * 45)
            target_direction = 270 - angle_offset
        
        # Normalize angle
        target_direction = target_direction % 360
        
        self.target_direction = target_direction
        self.target_distance = target_distance
        
        return target_direction, target_distance

# Enhanced LIDAR Processor
class LidarProcessor:
    def __init__(self):
        self.scan_data = {}
        self.lock = threading.Lock()
        
        # Obstacle status
        self.front_obstacle = False
        self.left_obstacle = False
        self.right_obstacle = False
        self.back_obstacle = False
        self.danger_zone = False
        self.critical_danger = False
        
        # Distances
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.back_distance = float('inf')
        
        self.target_direction = None
        self.target_distance = None
        self.last_scan_time = 0
        self.last_scan_msg = None
        
        # Enhanced filtering
        self.dynamic_detector = DynamicObjectDetector()
        self.moving_avg_window = 3
        self.distance_history = {}
        
        # Adaptive processing
        self.scan_quality_threshold = 0.8
        self.min_scan_points = 100
        
    def process_scan(self, scan_msg):
        """Enhanced scan processing with better filtering"""
        with self.lock:
            self.last_scan_msg = scan_msg
            self.last_scan_time = time.time()
            self.scan_data.clear()
            
            ranges = scan_msg.ranges
            angle_increment = scan_msg.angle_increment
            angle_min = scan_msg.angle_min
            
            # Adaptive step size based on scan quality
            total_valid_points = sum(1 for r in ranges if 0.01 < r < 10.0 and not math.isinf(r))
            scan_quality = total_valid_points / len(ranges)
            
            if scan_quality > self.scan_quality_threshold:
                step_size = 8  # High quality, use more points
            else:
                step_size = 12  # Lower quality, skip more points
            
            valid_points = 0
            for i in range(0, len(ranges), step_size):
                distance = ranges[i]
                
                if distance < 0.01 or distance > 10.0 or math.isinf(distance):
                    continue
                
                angle_rad = angle_min + (i * angle_increment)
                angle_deg = int(math.degrees(angle_rad) % 360)
                distance_mm = int(distance * 1000)
                
                self.scan_data[angle_deg] = distance_mm
                valid_points += 1
            
            # Only process if we have sufficient data
            if valid_points >= self.min_scan_points:
                self.scan_data = self._filter_lidar_data(self.scan_data)
                self._analyze_obstacles_with_dynamic_detection()
            else:
                print(f"Insufficient LIDAR data: {valid_points} points")
    
    def _filter_lidar_data(self, scan_data):
        """Enhanced LIDAR data filtering"""
        filtered_data = {}
        
        for angle, distance in scan_data.items():
            if angle not in self.distance_history:
                self.distance_history[angle] = deque(maxlen=self.moving_avg_window)
            
            # Outlier detection
            if len(self.distance_history[angle]) > 0:
                recent_distances = list(self.distance_history[angle])
                median_distance = sorted(recent_distances)[len(recent_distances)//2]
                
                # Reject outliers (more than 50% different from median)
                if abs(distance - median_distance) / median_distance > 0.5:
                    distance = median_distance  # Use median instead
            
            self.distance_history[angle].append(distance)
            
            # Apply moving average
            filtered_data[angle] = sum(self.distance_history[angle]) / len(self.distance_history[angle])
        
        return filtered_data
    
    def _analyze_obstacles_with_dynamic_detection(self):
        """Enhanced obstacle analysis"""
        if not self.scan_data:
            return
        
        # Reset status
        self.front_obstacle = self.left_obstacle = self.right_obstacle = self.back_obstacle = False
        self.danger_zone = self.critical_danger = False
        self.front_distance = self.left_distance = self.right_distance = self.back_distance = float('inf')
        
        # Check dynamic objects first
        waiting_for_dynamic, dynamic_distance = self.dynamic_detector.check_dynamic_objects_in_path(self.scan_data)
        
        if waiting_for_dynamic:
            self.front_obstacle = True
            self.danger_zone = True
            if dynamic_distance < config.CRITICAL_DANGER_THRESHOLD:
                self.critical_danger = True
            return
        
        # Analyze static obstacles
        for angle, distance in self.scan_data.items():
            # Skip target if it's identified
            if (self.target_direction is not None and
                abs(angle - self.target_direction) < config.TARGET_EXCLUSION_ANGLE):
                continue
            
            # Categorize by region
            if self._is_in_front_region(angle):
                self.front_distance = min(self.front_distance, distance)
                if distance < config.SCAN_THRESHOLD:
                    self.front_obstacle = True
                if distance < config.DANGER_THRESHOLD:
                    self.danger_zone = True
                if distance < config.CRITICAL_DANGER_THRESHOLD:
                    self.critical_danger = True
                    
            elif self._is_in_back_region(angle):
                self.back_distance = min(self.back_distance, distance)
                if distance < config.SCAN_THRESHOLD:
                    self.back_obstacle = True
                    
            elif 31 <= angle <= 140:  # Right
                self.right_distance = min(self.right_distance, distance)
                if distance < config.SCAN_THRESHOLD:
                    self.right_obstacle = True
                    
            elif 220 <= angle <= 329:  # Left
                self.left_distance = min(self.left_distance, distance)
                if distance < config.SCAN_THRESHOLD:
                    self.left_obstacle = True
    
    def _is_in_front_region(self, angle):
        return (330 <= angle <= 360) or (0 <= angle <= 30)
    
    def _is_in_back_region(self, angle):
        return 150 <= angle <= 210
    
    def set_target_info(self, direction, distance):
        """Set target information"""
        self.target_direction = direction
        self.target_distance = distance
    
    def get_obstacle_status(self):
        """Get comprehensive obstacle status"""
        with self.lock:
            scan_age = time.time() - self.last_scan_time
            data_valid = scan_age < 0.1  # Stricter validity check
            
            waiting_for_dynamic = self.dynamic_detector.waiting_for_dynamic_object
            
            return {
                'front': {'obstacle': self.front_obstacle if data_valid else False,
                         'distance': self.front_distance if data_valid else float('inf')},
                'left': {'obstacle': self.left_obstacle if data_valid else False,
                        'distance': self.left_distance if data_valid else float('inf')},
                'right': {'obstacle': self.right_obstacle if data_valid else False,
                         'distance': self.right_distance if data_valid else float('inf')},
                'back': {'obstacle': self.back_obstacle if data_valid else False,
                        'distance': self.back_distance if data_valid else float('inf')},
                'danger_zone': self.danger_zone if data_valid else False,
                'critical_danger': self.critical_danger if data_valid else False,
                'data_valid': data_valid,
                'scan_points': len(self.scan_data) if data_valid else 0,
                'scan_age': scan_age,
                'waiting_for_dynamic': waiting_for_dynamic,
                'dynamic_objects': len(self.dynamic_detector.dynamic_objects),
                'static_objects': len(self.dynamic_detector.static_objects)
            }
    
    def get_safe_direction(self):
        """Enhanced safe direction determination"""
        status = self.get_obstacle_status()
        
        if not status['data_valid']:
            return None
        
        if status['waiting_for_dynamic']:
            return "STOP_DYNAMIC"
        
        if status['critical_danger']:
            return "STOP"
        
        # Multi-criteria decision making
        directions = {
            'FORWARD': status['front']['distance'],
            'LEFT': status['left']['distance'],
            'RIGHT': status['right']['distance'],
            'BACK': status['back']['distance']
        }
        
        # Filter out blocked directions
        safe_directions = {k: v for k, v in directions.items() 
                          if v > config.DANGER_THRESHOLD}
        
        if not safe_directions:
            return "STOP"
        
        # Choose direction with maximum clearance
        best_direction = max(safe_directions, key=safe_directions.get)
        
        # Prefer forward if it's safe enough
        if directions['FORWARD'] > config.DANGER_THRESHOLD * 1.5:
            return "FORWARD"
        
        return best_direction

# Enhanced Robot Controller
class RobotController:
    def __init__(self, r_wheel_port, l_wheel_port):
        # Motor controllers
        self.right_motor = MotorControl(device=r_wheel_port)
        self.left_motor = MotorControl(device=l_wheel_port)
        self.right_motor.set_drive_mode(1, 2)
        self.left_motor.set_drive_mode(1, 2)
        
        # Enhanced control parameters
        self.speed = config.DEFAULT_SPEED
        self.rotation_factor = config.ROTATION_FACTOR
        self.stop_threshold = config.STOP_THRESHOLD
        
        # Status tracking
        self.obstacle_avoidance_active = False
        self.current_direction = "STOP"
        self.emergency_stop = False
        self.waiting_for_dynamic_object = False
        
        # Enhanced command batching
        self.last_command = (0, 0)
        self.command_threshold = 3  # Reduced for smoother control
        self.last_command_time = 0
        self.min_command_interval = 0.01  # Increased frequency
        
        # Performance tracking
        self.command_history = deque(maxlen=100)
        self.performance_metrics = {
            'collision_count': 0,
            'false_positive_count': 0,
            'target_reach_times': deque(maxlen=20),
            'path_efficiency_scores': deque(maxlen=20)
        }
        
        # Adaptive control
        self.adaptive_speed_factor = 1.0
        self.last_obstacle_time = 0
        
    def move(self, right_speed, left_speed):
        """Enhanced move with adaptive control"""
        current_time = time.time()
        
        # Apply adaptive speed factor
        right_speed *= self.adaptive_speed_factor
        left_speed *= self.adaptive_speed_factor
        
        # Enhanced command batching
        speed_diff = abs(right_speed - self.last_command[0]) + abs(left_speed - self.last_command[1])
        time_diff = current_time - self.last_command_time
        
        if speed_diff < self.command_threshold and time_diff < self.min_command_interval:
            return
        
        # Smooth acceleration/deceleration
        if self.last_command != (0, 0):
            max_accel = 20  # Max acceleration per command
            right_speed = self._limit_acceleration(right_speed, self.last_command[0], max_accel)
            left_speed = self._limit_acceleration(left_speed, self.last_command[1], max_accel)
        
        # Send command
        try:
            self.right_motor.send_rpm(1, int(right_speed))
            self.left_motor.send_rpm(1, int(left_speed))
            
            self.last_command = (right_speed, left_speed)
            self.last_command_time = current_time
            
            # Track command for performance analysis
            self.command_history.append({
                'timestamp': current_time,
                'command': (right_speed, left_speed),
                'direction': self.current_direction
            })
            
        except Exception as e:
            print(f"Motor command error: {e}")
            gpio_pin_27.on()
    
    def _limit_acceleration(self, target_speed, current_speed, max_accel):
        """Limit acceleration for smoother movement"""
        diff = target_speed - current_speed
        if abs(diff) > max_accel:
            return current_speed + (max_accel if diff > 0 else -max_accel)
        return target_speed
    
    def stop(self):
        """Enhanced stop with gradual deceleration"""
        if self.last_command != (0, 0):
            # Gradual stop for smoother operation
            current_right, current_left = self.last_command
            steps = 3
            for i in range(steps):
                factor = (steps - i - 1) / steps
                self.right_motor.send_rpm(1, int(current_right * factor))
                self.left_motor.send_rpm(1, int(current_left * factor))
                time.sleep(0.01)
        
        self.move(0, 0)
        self.current_direction = "STOP"
    
    def check_critical_obstacles(self, lidar):
        """Enhanced critical obstacle checking"""
        status = lidar.get_obstacle_status()
        
        if status['waiting_for_dynamic']:
            print("CRITICAL: Dynamic object in path")
            self.stop()
            self.waiting_for_dynamic_object = True
            self.last_obstacle_time = time.time()
            return True
        
        if status['data_valid'] and status['critical_danger']:
            print("CRITICAL: Immediate danger detected")
            self.stop()
            self.emergency_stop = True
            self.last_obstacle_time = time.time()
            return True
        
        # Adaptive speed based on obstacle proximity
        if status['danger_zone']:
            self.adaptive_speed_factor = 0.6
        elif status['front']['distance'] < config.SCAN_THRESHOLD:
            self.adaptive_speed_factor = 0.8
        else:
            self.adaptive_speed_factor = 1.0
        
        self.emergency_stop = False
        self.waiting_for_dynamic_object = False
        return False
    
    def handle_obstacle_avoidance(self, lidar, target_in_view=False):
        """Enhanced obstacle avoidance with predictive behavior"""
        status = lidar.get_obstacle_status()
        
        if not status['data_valid']:
            if status['scan_age'] > 0.05:
                print(f"LIDAR data stale: {status['scan_age']:.3f}s")
            return False
        
        # Priority 1: Dynamic objects
        if status['waiting_for_dynamic']:
            print("AVOIDANCE: Waiting for dynamic object")
            self.stop()
            self.obstacle_avoidance_active = True
            return True
        
        # Priority 2: Critical danger
        if status['critical_danger']:
            print("AVOIDANCE: Critical danger - emergency stop")
            self.stop()
            self.obstacle_avoidance_active = True
            return True
        
        # Priority 3: Target proximity override
        if target_in_view and lidar.target_distance and lidar.target_distance < config.STOP_THRESHOLD * 10:
            if not status['critical_danger']:
                print("TARGET OVERRIDE: Ignoring non-critical obstacles")
                return False
        
        # Priority 4: Predictive avoidance
        safe_direction = lidar.get_safe_direction()
        
        if safe_direction in ["STOP", "STOP_DYNAMIC"]:
            print("AVOIDANCE: No safe direction - stopping")
            self.stop()
            self.obstacle_avoidance_active = True
            return True
        
        # Enhanced turning logic
        if status['danger_zone'] or status['front']['distance'] < config.DANGER_THRESHOLD * 1.2:
            if safe_direction == "LEFT":
                turn_speed = self.speed * 0.7
                print(f"AVOIDANCE: Turning LEFT (speed: {turn_speed})")
                self.move(-turn_speed, turn_speed / config.ROTATION_FACTOR)
                self.current_direction = "LEFT"
                self.obstacle_avoidance_active = True
                return True
                
            elif safe_direction == "RIGHT":
                turn_speed = self.speed * 0.7
                print(f"AVOIDANCE: Turning RIGHT (speed: {turn_speed})")
                self.move(-turn_speed / config.ROTATION_FACTOR, turn_speed)
                self.current_direction = "RIGHT"
                self.obstacle_avoidance_active = True
                return True
        
        self.obstacle_avoidance_active = False
        return False
    
    def process_uwb_control(self, uwb_distances):
        """Enhanced UWB control with adaptive behavior"""
        A0, A1, A2 = uwb_distances['A0'], uwb_distances['A1'], uwb_distances['A2']
        
        print(f"\nUWB: A0={A0:.1f} A1={A1:.1f} A2={A2:.1f} cm")
        
        # Target reached check
        if A0 <= self.stop_threshold:
            print(f"TARGET REACHED (A0={A0:.1f} <= {self.stop_threshold})")
            self.stop()
            return
        
        # Enhanced control logic
        diff = A2 - A1
        abs_diff = abs(diff)
        
        # Adaptive turning based on distance and difference
        if A0 < 150:  # Close to target
            turn_sensitivity = 2.0
            max_turn_ratio = 0.9
        else:  # Far from target
            turn_sensitivity = 1.5
            max_turn_ratio = 0.7
        
        if abs_diff < 10:  # Target centered
            print("FORWARD: Target centered")
            self.move(-self.speed, self.speed)
            self.current_direction = "FORWARD"
            
        elif diff < 0:  # Target to the right
            turn_ratio = min(max_turn_ratio, abs_diff / 30.0 * turn_sensitivity)
            right_speed = -self.speed * (1.0 - turn_ratio)
            left_speed = self.speed
            print(f"RIGHT: diff={diff:.1f}, ratio={turn_ratio:.2f}")
            self.move(right_speed, left_speed)
            self.current_direction = "RIGHT"
            
        else:  # Target to the left
            turn_ratio = min(max_turn_ratio, abs_diff / 30.0 * turn_sensitivity)
            right_speed = -self.speed
            left_speed = self.speed * (1.0 - turn_ratio)
            print(f"LEFT: diff={diff:.1f}, ratio={turn_ratio:.2f}")
            self.move(right_speed, left_speed)
            self.current_direction = "LEFT"
    
    def process_control(self, uwb_distances, lidar):
        """Enhanced main control logic"""
        # Critical safety check first
        if self.check_critical_obstacles(lidar):
            return
        
        # Update target information
        target_direction, target_distance = uwb_tracker.estimate_target_direction(uwb_distances)
        lidar.set_target_info(target_direction, target_distance)
        
        # Determine target visibility
        target_in_view = (
            uwb_distances['A0'] < 200 and
            abs(uwb_distances['A1'] - uwb_distances['A2']) < 25
        )
        
        # Obstacle avoidance with enhanced logic
        obstacle_action_needed = self.handle_obstacle_avoidance(lidar, target_in_view)
        
        # UWB control if path is clear
        if not obstacle_action_needed and not self.emergency_stop and not self.waiting_for_dynamic_object:
            self.process_uwb_control(uwb_distances)
    
    def get_performance_metrics(self):
        """Get current performance metrics"""
        return {
            'collision_rate': self.performance_metrics['collision_count'] / max(len(self.command_history), 1),
            'target_reach_time': np.mean(self.performance_metrics['target_reach_times']) if self.performance_metrics['target_reach_times'] else 10,
            'path_efficiency': np.mean(self.performance_metrics['path_efficiency_scores']) if self.performance_metrics['path_efficiency_scores'] else 0.5,
            'false_positive_rate': self.performance_metrics['false_positive_count'] / max(len(self.command_history), 1)
        }

# Enhanced Main Node
class EnhancedFollowingRobotNode(Node):
    def __init__(self):
        super().__init__('enhanced_following_robot_node')
        
        # Network setup
        self.udp_ip = get_ip_from_wifi()
        if not self.udp_ip:
            raise ValueError("Unable to retrieve IP address")
        
        target_last_digit = 128
        self.udp_ip = get_ip_from_subnet(self.udp_ip, target_last_digit)
        self.udp_port = 5005
        
        print(f"Enhanced Robot IP: {self.udp_ip}")
        
        # Hardware configuration
        self.r_wheel_port = "/dev/ttyRS485-1"
        self.l_wheel_port = "/dev/ttyRS485-2"
        
        # Performance tracking
        self.lidar_frame_counter = 0
        self.last_control_update = 0
        self._processing_lidar = False
        
        # Initialize enhanced components
        self.adaptive_threshold = AdaptiveThresholdManager()
        self.kalman_filter = RobotKalmanFilter()
        self.parameter_optimizer = ParameterOptimizer()
        self.uwb_tracker = UWBTracker()
        self.lidar = LidarProcessor()
        self.controller = RobotController(self.r_wheel_port, self.l_wheel_port)
        
        # ROS2 setup
        self.lidar_cb_group = ReentrantCallbackGroup()
        
        lidar_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=config.LIDAR_BUFFER_SIZE
        )
        
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.enhanced_scan_callback,
            qos_profile=lidar_qos,
            callback_group=self.lidar_cb_group
        )
        
        # UWB setup
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, config.UWB_BUFFER_SIZE)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.settimeout(config.UWB_TIMEOUT)
        
        # Data storage
        self.raw_uwb_distances = {'A0': 1000, 'A1': 1000, 'A2': 1000}
        self.corrected_uwb_distances = {'A0': 1000, 'A1': 1000, 'A2': 1000}
        
        # Control state
        self.running = True
        self.last_uwb_update = 0
        
        # Enhanced control timer
        self.control_timer = self.create_timer(
            1.0 / config.CONTROL_FREQUENCY,
            self.enhanced_control_loop
        )
        
        # Statistics and optimization
        self.stats_timer = self.create_timer(5.0, self.log_enhanced_statistics)
        self.optimization_timer = self.create_timer(30.0, self.run_parameter_optimization)
        
        # Performance monitoring
        self.last_lidar_time = time.time()
        self.lidar_update_count = 0
        self.control_loop_times = deque(maxlen=100)
        
        self.get_logger().info('Enhanced Following Robot Node started')
        self.get_logger().info('Features: Kalman Filter, Adaptive Thresholds, Parameter Optimization')
        
        # Signal successful start
        gpio_pin_17.on()
    
    def enhanced_scan_callback(self, msg):
        """Enhanced LIDAR callback with performance monitoring"""
        if self._processing_lidar:
            return
        
        self._processing_lidar = True
        start_time = time.time()
        
        try:
            if self.lidar_frame_counter % config.LIDAR_SKIP_FRAMES == 0:
                self.lidar.process_scan(msg)
            
            self.lidar_frame_counter += 1
            self.lidar_update_count += 1
            
            # Update Kalman filter with LIDAR data if available
            if hasattr(self.lidar, 'scan_data') and self.lidar.scan_data:
                # Simple position estimation from LIDAR (could be enhanced)
                front_distance = self.lidar.front_distance
                if front_distance < float('inf'):
                    estimated_x = front_distance * math.cos(0)  # Assuming forward direction
                    estimated_y = front_distance * math.sin(0)
                    self.kalman_filter.update_lidar([estimated_x, estimated_y])
            
        except Exception as e:
            self.get_logger().error(f"LIDAR processing error: {e}")
            gpio_pin_27.on()
        finally:
            self._processing_lidar = False
            
        # Monitor callback performance
        callback_time = time.time() - start_time
        if callback_time > 0.01:  # 10ms threshold
            self.get_logger().warn(f"Slow LIDAR callback: {callback_time*1000:.1f}ms")
    
    def process_uwb_data_enhanced(self):
        """Enhanced UWB data processing with Kalman filter"""
        try:
            self.sock.setblocking(False)
            ready = select.select([self.sock], [], [], config.UWB_TIMEOUT)
            
            if ready[0]:
                data, addr = self.sock.recvfrom(config.UWB_BUFFER_SIZE)
                parts = data.decode().split(",")
                
                if len(parts) >= 3:
                    self.raw_uwb_distances = {
                        'A0': float(parts[0]),
                        'A1': float(parts[1]),
                        'A2': float(parts[2])
                    }
                    
                    # Apply enhanced bias correction
                    self.corrected_uwb_distances = self.uwb_tracker.apply_bias_correction(
                        self.raw_uwb_distances
                    )
                    
                    # Update Kalman filter with UWB data
                    self.kalman_filter.update_uwb(self.corrected_uwb_distances)
                    
                    self.last_uwb_update = time.time()
                    return True
                    
        except (socket.error, ValueError, IndexError) as e:
            if str(e) != "timed out":  # Don't log timeout errors
                self.get_logger().debug(f"UWB error: {e}")
        
        return False
    
    def update_adaptive_systems(self):
        """Update adaptive thresholds and parameters"""
        # Calculate current robot speed
        robot_speed = abs(self.controller.last_command[0]) + abs(self.controller.last_command[1])
        
        # Calculate obstacle density
        obstacle_density = 0
        if hasattr(self.lidar, 'scan_data') and self.lidar.scan_data:
            close_obstacles = sum(1 for d in self.lidar.scan_data.values() if d < config.SCAN_THRESHOLD)
            obstacle_density = close_obstacles / len(self.lidar.scan_data) * 10
        
        # Get recent performance
        performance_metrics = self.controller.get_performance_metrics()
        recent_performance = 1.0 - performance_metrics['collision_rate']
        
        # Update adaptive thresholds
        self.adaptive_threshold.update_thresholds(
            robot_speed, obstacle_density, recent_performance
        )
    
    def enhanced_control_loop(self):
        """Enhanced control loop with all improvements"""
        if not self.running:
            return
        
        start_time = time.time()
        
        try:
            # Update Kalman filter prediction
            self.kalman_filter.predict(1.0 / config.CONTROL_FREQUENCY)
            
            # Priority 1: Critical safety check
            if self.controller.check_critical_obstacles(self.lidar):
                return
            
            # Priority 2: Update adaptive systems
            self.update_adaptive_systems()
            
            # Priority 3: Process UWB data
            uwb_updated = self.process_uwb_data_enhanced()
            
            # Priority 4: Execute control decision
            if uwb_updated or time.time() - self.last_control_update > 0.1:
                self.controller.process_control(self.corrected_uwb_distances, self.lidar)
                self.last_control_update = time.time()
            
        except Exception as e:
            self.get_logger().error(f"Enhanced control loop error: {e}")
            gpio_pin_27.on()
        
        # Performance monitoring
        loop_time = time.time() - start_time
        self.control_loop_times.append(loop_time)
        
        if loop_time > config.MAX_LOOP_TIME:
            self.get_logger().warn(f"Slow control loop: {loop_time*1000:.1f}ms")
    
    def log_enhanced_statistics(self):
        """Enhanced statistics logging"""
        current_time = time.time()
        
        # Calculate LIDAR update rate
        update_interval = current_time - self.last_lidar_time
        if update_interval > 0:
            lidar_rate = self.lidar_update_count / update_interval
            self.last_lidar_time = current_time
            self.lidar_update_count = 0
        else:
            lidar_rate = 0
        
        # Get system status
        status = self.lidar.get_obstacle_status()
        performance = self.controller.get_performance_metrics()
        
        # Calculate average control loop time
        avg_loop_time = np.mean(self.control_loop_times) if self.control_loop_times else 0
        
        # Get Kalman filter position
        kf_x, kf_y, kf_theta = self.kalman_filter.get_position()
        
        self.get_logger().info(
            f"ENHANCED STATS:\n"
            f"  LIDAR: {lidar_rate:.1f}Hz | Loop: {avg_loop_time*1000:.1f}ms\n"
            f"  UWB age: {current_time - self.last_uwb_update:.2f}s\n"
            f"  Dynamic: {status['dynamic_objects']} | Static: {status['static_objects']}\n"
            f"  Thresholds: S={config.SCAN_THRESHOLD} D={config.DANGER_THRESHOLD} C={config.CRITICAL_DANGER_THRESHOLD}\n"
            f"  Position: ({kf_x:.0f}, {kf_y:.0f}, {math.degrees(kf_theta):.0f}°)\n"
            f"  Performance: Collision={performance['collision_rate']:.3f} Efficiency={performance['path_efficiency']:.3f}"
        )
    
    def run_parameter_optimization(self):
        """Run parameter optimization periodically"""
        try:
            performance_metrics = self.controller.get_performance_metrics()
            performance_score = self.parameter_optimizer.evaluate_performance(performance_metrics)
            
            if len(self.parameter_optimizer.performance_metrics) > 20:
                optimized_params = self.parameter_optimizer.optimize_parameters()
                
                # Apply optimized parameters
                config.ROTATION_FACTOR = optimized_params['rotation_factor']['current']
                config.LIDAR_SKIP_FRAMES = int(optimized_params['lidar_skip_frames']['current'])
                
                self.get_logger().info(
                    f"OPTIMIZATION: Score={performance_score:.3f} "
                    f"RotFactor={config.ROTATION_FACTOR:.2f} "
                    f"SkipFrames={config.LIDAR_SKIP_FRAMES}"
                )
                
        except Exception as e:
            self.get_logger().error(f"Parameter optimization error: {e}")
    
    def stop(self):
        """Enhanced shutdown procedure"""
        self.get_logger().info("Shutting down Enhanced Following Robot...")
        self.running = False
        
        # Gradual stop
        self.controller.stop()
        
        # Close connections
        try:
            self.sock.close()
        except:
            pass
        
        # Turn off GPIO
        gpio_pin_17.off()
        gpio_pin_27.off()
        
        self.get_logger().info("Enhanced robot systems shut down successfully")

def main(args=None):
    """Enhanced main function"""
    rclpy.init(args=args)
    
    # Create global UWB tracker reference
    global uwb_tracker
    uwb_tracker = UWBTracker()
    
    # Create enhanced node
    node = EnhancedFollowingRobotNode()
    
    # Create multi-threaded executor
    executor = MultiThreadedExecutor(num_threads=4)  # Increased threads
    executor.add_node(node)
    
    try:
        print("Starting Enhanced Following Robot with advanced features...")
        print("Features: Kalman Filter, Adaptive Thresholds, Parameter Optimization, Enhanced Safety")
        executor.spin()
    except KeyboardInterrupt:
        print("\nUser interrupted. Shutting down...")
    except Exception as e:
        print(f"Error: {e}")
        gpio_pin_27.on()
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
