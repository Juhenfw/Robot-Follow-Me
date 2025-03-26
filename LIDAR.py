import math
import numpy as np
import time
import serial
import pygame
import ddsm115 as motor
from rplidar import RPLidar
from threading import Thread
from collections import deque

# Configuration parameters unchanged...

# NEW: Obstacle avoidance parameters
OBSTACLE_DETECTION_RANGE = 100  # cm
OBSTACLE_SAFE_DISTANCE = 40     # cm
OBSTACLE_CRITICAL_DISTANCE = 30 # cm
OBSTACLE_WEIGHT = 1.5           # Weight of obstacle avoidance vs. person following
FIELD_OF_VIEW = 220             # degrees - detect obstacles in front and sides
SIDE_CLEARANCE = 25             # cm - minimum side clearance

# NEW: Enhanced person tracking 
TRACKING_TIMEOUT = 2.0          # seconds until we consider person lost
PERSON_WIDTH_TOLERANCE = 0.15   # meters - added tolerance for person width
TRACKING_CONFIDENCE_THRESHOLD = 0.6  # minimum confidence to consider valid

# Existing global variables...

# NEW: Global variables for obstacle avoidance
obstacles = []  # List of detected obstacles [angle, distance, size]
obstacle_vectors = []  # Repulsive vectors from obstacles
last_person_detection = 0  # Timestamp of last confident person detection
person_tracking_id = None  # For continuous tracking of the same person
person_tracking_confidence = 0.0  # Current tracking confidence

# Modified buffer sizes for better responsiveness
uwb_distance_buffer = deque(maxlen=3)  # Reduced buffer size for faster response
target_angle_buffer = deque(maxlen=3)
target_distance_buffer = deque(maxlen=3)

# NEW: Create a tracking history for better prediction during brief sensor outages
person_position_history = deque(maxlen=20)  # Store [time, angle, distance] tuples

def median_filter(values):
    """Apply median filter to remove outliers."""
    if not values:
        return None
    return sorted(values)[len(values)//2]

# MODIFIED: Enhanced distance correction with improved bias handling
def correct_bias(raw_distance, bias=0):
    """Corrects bias in UWB sensor readings with dynamic calibration."""
    # Apply nonlinear correction for different distance ranges
    if raw_distance < 50:  # Close range correction
        correction_factor = 0.9
        bias = 5  # More aggressive bias correction at close range
    elif raw_distance < 150:  # Mid range
        correction_factor = 0.95
        bias = 2
    else:  # Far range
        correction_factor = 1.0
        bias = 0
        
    # Apply corrections
    corrected_distance = (raw_distance * correction_factor) - bias
    
    # Enforce reasonable limits
    return max(20, min(300, corrected_distance))

# NEW: Vector-based obstacle avoidance
def calculate_obstacle_avoidance_vector():
    """Calculate repulsive vector from obstacles."""
    if not obstacles:
        return 0, 0  # No obstacles, no avoidance vector
    
    # Initialize repulsive vector components
    repulsive_x = 0
    repulsive_y = 0
    
    for angle, distance, size in obstacles:
        # Skip obstacles that are too far
        if distance > OBSTACLE_DETECTION_RANGE:
            continue
            
        # Calculate inverse-square repulsive force (stronger as distance decreases)
        force_magnitude = OBSTACLE_WEIGHT * (1.0 / max(10, distance - OBSTACLE_CRITICAL_DISTANCE)**2)
        
        # Convert to radians for calculation
        angle_rad = math.radians(angle)
        
        # Calculate repulsive vector components (opposite direction from obstacle)
        repulsive_x -= force_magnitude * math.cos(angle_rad)
        repulsive_y -= force_magnitude * math.sin(angle_rad)
    
    return repulsive_x, repulsive_y

# MODIFIED: Enhanced person prediction with better tracking history
def predict_future_position(prediction_time, use_history=True):
    """Predict future position with improved history-based tracking."""
    global person_position_history, person_velocity, person_angular_velocity
    
    if use_history and len(person_position_history) >= 3:
        # Extract history data
        times = [t for t, _, _ in person_position_history]
        angles = [a for _, a, _ in person_position_history]
        distances = [d for _, _, d in person_position_history]
        
        # Use linear regression for more stable velocity estimates
        if len(times) >= 3:
            # Normalize times relative to first point
            times_normalized = [t - times[0] for t in times]
            
            # Calculate distance velocity using linear regression
            distance_coeffs = np.polyfit(times_normalized, distances, 1)
            distance_velocity = distance_coeffs[0]  # Slope is velocity
            
            # Handle angle wrap-around for angular velocity calculation
            unwrapped_angles = np.unwrap([math.radians(a) for a in angles])
            angle_coeffs = np.polyfit(times_normalized, unwrapped_angles, 1)
            angle_velocity = math.degrees(angle_coeffs[0])  # Convert back to degrees/sec
            
            # Current values
            current_distance = distances[-1]
            current_angle = angles[-1]
            
            # Predict future position
            predicted_distance = current_distance + (distance_velocity * prediction_time)
            predicted_angle = (current_angle + (angle_velocity * prediction_time)) % 360
            
            return predicted_distance, predicted_angle
    
    # Fallback to simpler prediction if history is insufficient
    if uwb_distance is not None and target_angle is not None:
        predicted_distance = uwb_distance + (person_velocity * prediction_time)
        predicted_angle = (target_angle + (person_angular_velocity * prediction_time)) % 360
        return predicted_distance, predicted_angle
        
    return None, None

# NEW: Detect obstacles from LiDAR data
def detect_obstacles(scan_data, ignore_angle=None, ignore_tolerance=30):
    """
    Process LiDAR data to identify obstacles.
    
    Parameters:
    - scan_data: List of (angle, distance) tuples from LiDAR
    - ignore_angle: Angle where the person is (to avoid treating person as obstacle)
    - ignore_tolerance: Angular width to ignore around the person
    
    Returns: List of detected obstacles as (angle, distance, size) tuples
    """
    if not scan_data:
        return []
    
    # Filter to relevant field of view (in front of robot)
    # Convert to front-centered coordinate system where 0° is front
    centered_scan = []
    for angle, distance in scan_data:
        # Normalize angle to -180 to 180 range (0 = front)
        centered_angle = (angle - 180) % 360
        if centered_angle > 180:
            centered_angle -= 360
            
        # Check if in our field of view
        if abs(centered_angle) <= FIELD_OF_VIEW/2:
            centered_scan.append((centered_angle, distance))
    
    # Sort by angle for clustering
    centered_scan.sort(key=lambda x: x[0])
    
    # Cluster points to identify obstacles
    clusters = []
    current_cluster = []
    
    for i, (angle, distance) in enumerate(centered_scan):
        # Check if this point should be ignored (person location)
        if ignore_angle is not None:
            # Convert person angle to same coordinate system
            person_centered = (ignore_angle - 180) % 360
            if person_centered > 180:
                person_centered -= 360
                
            # Skip points near person
            if abs((angle - person_centered + 180) % 360 - 180) < ignore_tolerance:
                continue
        
        # Start new cluster or add to existing
        if not current_cluster or abs(angle - current_cluster[-1][0]) < 5:
            current_cluster.append((angle, distance))
        else:
            # Process completed cluster
            if len(current_cluster) >= 3:
                avg_angle = sum(a for a, _ in current_cluster) / len(current_cluster)
                avg_distance = sum(d for _, d in current_cluster) / len(current_cluster)
                cluster_size = len(current_cluster)
                clusters.append((avg_angle, avg_distance * 100, cluster_size))  # Convert to cm
            
            # Start new cluster
            current_cluster = [(angle, distance)]
    
    # Process last cluster
    if len(current_cluster) >= 3:
        avg_angle = sum(a for a, _ in current_cluster) / len(current_cluster)
        avg_distance = sum(d for _, d in current_cluster) / len(current_cluster)
        cluster_size = len(current_cluster)
        clusters.append((avg_angle, avg_distance * 100, cluster_size))  # Convert to cm
    
    return clusters

# MODIFIED: Enhanced person detection with temporal tracking
def identify_person_cluster(clusters, previous_angle=None, previous_distance=None):
    """
    Identify which cluster is most likely to be a person with temporal consistency.
    
    Parameters:
    - clusters: List of cluster data
    - previous_angle: Last known angle of person
    - previous_distance: Last known distance of person
    
    Returns: Best person candidate or None
    """
    global person_tracking_id, person_tracking_confidence, last_person_detection
    
    if not clusters:
        # Decrease confidence over time
        person_tracking_confidence *= 0.9
        return None
    
    person_candidates = []
    
    for i, cluster in enumerate(clusters):
        # Check if cluster width is in human range
        if HUMAN_SIZE_RANGE[0] <= cluster['width'] <= HUMAN_SIZE_RANGE[1] + PERSON_WIDTH_TOLERANCE:
            # Basic confidence from physical properties
            confidence = min(cluster['points'] / MIN_CLUSTER_POINTS, 1.0)
            
            # Add distance factor
            distance_factor = max(0.5, 1.0 - (cluster['distance'] / 5.0))
            score = confidence * distance_factor
            
            # Add temporal consistency if we have previous detection
            if previous_angle is not None and previous_distance is not None:
                # Calculate consistency with previous detection
                angle_diff = min(abs((cluster['angle'] - previous_angle) % 360), 
                                  abs((previous_angle - cluster['angle']) % 360))
                
                dist_diff = abs(cluster['distance'] - previous_distance)
                
                # More weight to angle consistency than distance (UWB is more reliable for distance)
                if angle_diff < 30 and dist_diff < 100:
                    consistency_bonus = 1.0 - (angle_diff / 60.0) - (dist_diff / 200.0)
                    score += consistency_bonus
            
            person_candidates.append((score, i, cluster))
    
    # Sort by score (highest first)
    person_candidates.sort(reverse=True, key=lambda x: x[0])
    
    # Return the best candidate if score passes threshold
    if person_candidates and person_candidates[0][0] > TRACKING_CONFIDENCE_THRESHOLD:
        best_score, best_idx, best_cluster = person_candidates[0]
        
        # Update tracking ID and confidence
        person_tracking_id = best_idx
        person_tracking_confidence = best_score
        last_person_detection = time.time()
        
        return best_cluster
    
    # Decrease confidence if no good match
    person_tracking_confidence *= 0.9
    return None

# MODIFIED: Enhanced LiDAR scan processing with obstacle detection
def scan_lidar():
    """Process LiDAR scans to track people and detect obstacles."""
    global running, target_angle, target_distance, obstacles, last_person_detection
    
    # Initialize smoothing values
    smoothed_angle = None
    smoothed_distance = None
    
    try:
        lidar.connect()
        print("LiDAR connected.")
        
        # Skip first few scans to allow LiDAR to stabilize
        for _ in range(5):
            lidar.iter_scans()
            time.sleep(0.1)
        
        for scan in lidar.iter_scans():
            if not running:
                break
                
            if not scan:
                continue
                
            # Prepare scan data
            scan_data = []
            for _, angle, distance in scan:
                # Convert mm to meters for clustering
                distance_m = distance / 1000.0
                if distance_m <= 5.0:  # Limit to 5 meters
                    scan_data.append((angle, distance_m))
            
            if scan_data:
                # First, identify the person if possible
                clusters = cluster_points(scan_data)
                
                # Use previous detection as hint
                previous_angle = target_angle
                previous_distance = target_distance/10 if target_distance else None
                
                person = identify_person_cluster(clusters, previous_angle, previous_distance)
                
                if person:
                    # Store raw angle and distance in mm
                    raw_angle = person['angle']
                    raw_distance = person['distance'] * 1000  # Convert back to mm
                    
                    # Add to smoothing buffers
                    target_angle_buffer.append(raw_angle)
                    target_distance_buffer.append(raw_distance)
                    
                    # Apply median filter
                    filtered_angle = median_filter(target_angle_buffer)
                    filtered_distance = median_filter(target_distance_buffer)
                    
                    # Apply exponential smoothing
                    smoothed_angle = exponential_moving_average(filtered_angle, smoothed_angle, alpha=0.5)  # More responsive
                    smoothed_distance = exponential_moving_average(filtered_distance, smoothed_distance, alpha=0.5)
                    
                    # Update global values
                    target_angle = smoothed_angle
                    target_distance = smoothed_distance
                    
                    # Add to tracking history
                    person_position_history.append((time.time(), target_angle, target_distance/10))
                    
                    # Update velocity estimates
                    update_velocity_estimates()
                    
                    direction = get_direction_name(target_angle)
                    confidence = person['points']
                    print(f"LiDAR Target: {direction} at angle={target_angle:.2f}°, distance={target_distance/10:.2f}cm (confidence: {confidence})")
                
                # Next, detect obstacles (excluding person location)
                person_angle = target_angle if target_angle is not None else None
                obstacles = detect_obstacles(scan_data, ignore_angle=person_angle)
                
                # Log obstacles if any are close
                close_obstacles = [o for o in obstacles if o[1] < OBSTACLE_DETECTION_RANGE]
                if close_obstacles:
                    print(f"Detected {len(close_obstacles)} obstacles within {OBSTACLE_DETECTION_RANGE}cm")
            
            time.sleep(0.05)
            
    except Exception as e:
        print(f"LiDAR Error: {e}")
    finally:
        if running:  # Only stop if we're still running (avoid double-stop)
            lidar.stop()
            print("LiDAR stopped.")

# MODIFIED: Enhanced movement function with obstacle avoidance
def move_robot(forward_speed, rotation_speed, ramp=True, avoid_obstacles=True):
    """Controls robot movement with obstacle avoidance."""
    # Static variables to track current speeds
    if not hasattr(move_robot, "current_speeds"):
        move_robot.current_speeds = [0, 0]  # [forward, rotation]
    
    # Apply obstacle avoidance if enabled
    if avoid_obstacles and obstacles:
        # Calculate avoidance vector
        avoid_x, avoid_y = calculate_obstacle_avoidance_vector()
        
        # Check if any obstacles are critically close
        critical_obstacles = [o for o in obstacles if o[1] < OBSTACLE_CRITICAL_DISTANCE]
        
        if critical_obstacles:
            # Emergency avoidance - override forward command if obstacle directly ahead
            ahead_obstacles = [o for o in critical_obstacles if abs(o[0]) < 30]
            if ahead_obstacles:
                print("CRITICAL OBSTACLE AHEAD! Stopping forward motion")
                forward_speed = min(0, forward_speed)  # Allow backward but not forward
        
        # Convert avoidance vector to speed adjustments
        if abs(avoid_x) > 0.01 or abs(avoid_y) > 0.01:
            # Calculate magnitude of avoidance
            magnitude = math.sqrt(avoid_x**2 + avoid_y**2)
            
            # Normalize and scale the avoidance effect
            avoid_scale = min(1.0, magnitude * 2)  # Limit maximum effect
            
            # Apply to forward speed (y component)
            forward_adjustment = -avoid_y * 100 * avoid_scale
            forward_speed += forward_adjustment
            
            # Apply to rotation (x component)
            rotation_adjustment = -avoid_x * 100 * avoid_scale
            rotation_speed += rotation_adjustment
            
            print(f"Obstacle avoidance: forward adj={forward_adjustment:.1f}, rotation adj={rotation_adjustment:.1f}")
    
    # Apply ramping if enabled
    if ramp:
        # Limit change in forward speed
        speed_diff = forward_speed - move_robot.current_speeds[0]
        if abs(speed_diff) > MAX_ACCELERATION:
            forward_speed = move_robot.current_speeds[0] + MAX_ACCELERATION * (1 if speed_diff > 0 else -1)
        
        # Limit change in rotation speed
        rot_diff = rotation_speed - move_robot.current_speeds[1]
        if abs(rot_diff) > MAX_ACCELERATION:
            rotation_speed = move_robot.current_speeds[1] + MAX_ACCELERATION * (1 if rot_diff > 0 else -1)
    
    # Update current speeds
    move_robot.current_speeds[0] = forward_speed
    move_robot.current_speeds[1] = rotation_speed
    
    # Calculate wheel speeds with improved differential drive model
    rotation_gain = 1.2  # Increase rotation response
    
    right_speed = forward_speed - (rotation_speed * rotation_gain)
    left_speed = forward_speed + (rotation_speed * rotation_gain)
    
    # Balance speeds if they exceed maximum
    max_speed = max(abs(right_speed), abs(left_speed))
    if max_speed > 100:
        right_speed = right_speed * 100 / max_speed
        left_speed = left_speed * 100 / max_speed
    
    # Clamp speeds to valid range
    right_speed = max(min(right_speed, 100), -100)
    left_speed = max(min(left_speed, 100), -100)
    
    # Convert to wheel direction (DDSM115 specific)
    right_motor.send_rpm(1, -right_speed)  # Inverted based on motor orientation
    left_motor.send_rpm(1, left_speed)
    
    # Create movement description
    movement_desc = ""
    if forward_speed > 5:
        movement_desc += "Moving Forward"
    elif forward_speed < -5:
        movement_desc += "Moving Backward"
    else:
        movement_desc += "Stationary"
        
    if rotation_speed > 5:
        movement_desc += " + Rotating Left"
    elif rotation_speed < -5:
        movement_desc += " + Rotating Right"
    
    print(f"Movement: {movement_desc} (R={-right_speed:.1f}, L={left_speed:.1f})")

# MODIFIED: Enhanced person following with obstacle avoidance
def follow_person():
    """Autonomous control logic with obstacle avoidance."""
    global running, uwb_distance, target_angle, target_distance, autonomous_mode
    global last_update_time, last_person_detection
    
    # Reset PID controllers
    reset_pid_controllers()
    
    last_command_time = time.time()
    stop_count = 0  # Counter for continuous stop commands
    
    try:
        while running:
            current_time = time.time()
            dt = current_time - last_update_time
            last_update_time = current_time
            
            # Skip if in manual mode
            if not autonomous_mode:
                time.sleep(0.1)
                continue
            
            # Check if we've lost track of the person
            person_lost = (current_time - last_person_detection) > TRACKING_TIMEOUT
            
            # Skip if no sensor data or person is lost
            if (uwb_distance is None and target_angle is None) or person_lost:
                if person_lost:
                    print("Warning: Person tracking lost! Stopping robot.")
                    move_robot(0, 0, ramp=True)
                time.sleep(0.1)
                continue
            
            # Use UWB for distance if available, otherwise LiDAR
            current_distance = uwb_distance if uwb_distance is not None else (target_distance / 10 if target_distance is not None else None)
            current_angle = target_angle
                
            # If we have complete data, use prediction for smoother following
            if current_distance is not None and current_angle is not None:
                # Get predicted position
                predicted_distance, predicted_angle = predict_future_position(prediction_time)
                
                if predicted_distance is not None and predicted_angle is not None:
                    target_distance_for_control = predicted_distance
                    target_angle_for_control = predicted_angle
                else:
                    target_distance_for_control = current_distance
                    target_angle_for_control = current_angle
                
                # Default to stopped
                forward_command = 0
                rotation_command = 0
                
                # Apply PID control if we have data
                if target_angle_for_control is not None:
                    # Calculate rotation command with PID
                    rotation_command = pid_angle_control(0, target_angle_for_control, dt)
                    
                    # Limit rotation command
                    rotation_command = max(min(rotation_command, NORMAL_SPEED), -NORMAL_SPEED)
                    
                    direction = get_direction_name(target_angle_for_control)
                    
                    if abs(calculate_rotation(target_angle_for_control, 0)) > ANGLE_TOLERANCE:
                        if rotation_command > 0:
                            print(f"Navigation: Rotating LEFT to face target at {direction}")
                        else:
                            print(f"Navigation: Rotating RIGHT to face target at {direction}")
                    else:
                        print(f"Navigation: Target angle good, facing {direction}")
                
                # Distance control only if we're roughly facing the right direction
                if target_distance_for_control is not None and abs(calculate_rotation(target_angle_for_control, 0)) <= ANGLE_TOLERANCE * 3:
                    # Calculate forward command with PID
                    dist_error = target_distance_for_control - FOLLOW_DISTANCE
                    forward_command = pid_distance_control(target_distance_for_control, FOLLOW_DISTANCE, dt)
                    
                    # Limit forward command
                    forward_command = max(min(forward_command, NORMAL_SPEED), -NORMAL_SPEED)
                    
                    if abs(dist_error) > DISTANCE_TOLERANCE:
                        if dist_error > 0:
                            print(f"Navigation: Moving FORWARD {dist_error:.2f}cm to reach target distance")
                        else:
                            print(f"Navigation: Moving BACKWARD {-dist_error:.2f}cm (too close)")
                    else:
                        print(f"Navigation: Distance good at {target_distance_for_control:.2f}cm")
                
                # Execute movement with obstacle avoidance
                move_robot(forward_command, rotation_command, ramp=True, avoid_obstacles=True)
                
                # Reset stop counter if we're moving
                if abs(forward_command) > 5 or abs(rotation_command) > 5:
                    stop_count = 0
                else:
                    stop_count += 1
                    
                # If stopped for too long, send explicit stop command occasionally
                if stop_count > 10 and stop_count % 5 == 0:
                    move_robot(0, 0, ramp=False)
                    print("Navigation: Sending explicit stop command")
            
            # Wait a bit
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("Follow operation stopped by user.")
    except Exception as e:
        print(f"Follow Error: {e}")

# The rest of the functions remain largely unchanged...

def main():
    """Main function to run the robot."""
    global running
    
    try:
        # Set running flag
        running = True
        
        # Create threads
        uwb_thread = Thread(target=read_uwb_data)
        lidar_thread = Thread(target=scan_lidar)
        follow_thread = Thread(target=follow_person)
        manual_thread = Thread(target=handle_manual_control)
        vis_thread = Thread(target=update_visualization)
        
        # Start threads
        uwb_thread.daemon = True
        uwb_thread.start()
        
        lidar_thread.daemon = True
        lidar_thread.start()
        
        follow_thread.daemon = True
        follow_thread.start()
        
        manual_thread.daemon = True
        manual_thread.start()
        
        vis_thread.daemon = True
        vis_thread.start()
        
        print("All systems started. Press Ctrl+C to stop.")
        
        # Main loop - just keep the main thread alive
        while running:
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("Program stopped by user.")
    except Exception as e:
        print(f"Main Error: {e}")
    finally:
        # Cleanup
        cleanup()

if __name__ == "__main__":
    main()
