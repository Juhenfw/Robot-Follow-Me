import math
import numpy as np
import time
import serial
import pygame
import ddsm115 as motor
from rplidar import RPLidar
from threading import Thread
from collections import deque

# UWB Configuration
PORT_UWB = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BAUDRATE_UWB = 115200
TIMEOUT_UWB = 1

# LiDAR Configuration
PORT_LIDAR = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_f235ae4105e1d247940e6441b646a0b3-if00-port0"

# Wheel Configuration
RIGHT_WHEEL_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B0049TUZ-if00-port0"
LEFT_WHEEL_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B0045S9B-if00-port0"

# Global variables
running = False
angles = []
distances = []
colors = []
uwb_distance = None
target_angle = None
target_distance = None
autonomous_mode = True  # True for follow mode, False for manual control

# Sensor data smoothing
uwb_distance_buffer = deque(maxlen=5)
target_angle_buffer = deque(maxlen=5)
target_distance_buffer = deque(maxlen=5)

# Tracking history for velocity prediction
uwb_history = deque(maxlen=10)  # Store time, distance pairs
angle_history = deque(maxlen=10)  # Store time, angle pairs

# Enhanced tracking parameters
last_update_time = time.time()
person_velocity = 0.0  # cm/s
person_angular_velocity = 0.0  # degrees/s
prediction_time = 0.5  # seconds into future to predict

# Initialize pygame for manual control
pygame.init()
screen = pygame.display.set_mode((800, 600))
pygame.display.set_caption("Follow Me Robot - Press TAB to switch modes")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 36)

# Initialize wheel motors
right_motor = motor.MotorControl(device=RIGHT_WHEEL_PORT)
right_motor.set_drive_mode(1, 2)
left_motor = motor.MotorControl(device=LEFT_WHEEL_PORT)
left_motor.set_drive_mode(1, 2)

# Initialize LiDAR
lidar = RPLidar(PORT_LIDAR, baudrate=256000)

# Motor speed parameters - Adjusted for better responsiveness
NORMAL_SPEED = 85  # Increased normal speed
SLOW_SPEED = 25  # Increased slow rotation speed
TURNING_SPEED = 40  # Increased turning speed
DRIFT_BOOST = 110  # Increased drift speed
MAX_ACCELERATION = 15  # Maximum speed change per update cycle
STOP = 0

# Robot following parameters - Optimized
FOLLOW_DISTANCE = 80  # Target distance to maintain (cm) - increased for better tracking
DISTANCE_TOLERANCE = 20  # Reduced tolerance for more accurate following
ANGLE_TOLERANCE = 5  # Reduced for more accurate direction following
MIN_CONFIDENCE_LEVEL = 2  # Minimum number of consistent readings

# PID Control parameters
P_DIST = 1.2  # Proportional gain for distance
I_DIST = 0.1  # Integral gain for distance
D_DIST = 0.4  # Derivative gain for distance
P_ANGLE = 1.5  # Proportional gain for angle
I_ANGLE = 0.1  # Integral gain for angle
D_ANGLE = 0.5  # Derivative gain for angle

# PID state variables
distance_integral = 0.0
distance_previous_error = 0.0
angle_integral = 0.0
angle_previous_error = 0.0

# Human detection parameters for LiDAR
MIN_CLUSTER_POINTS = 3  # Minimum points to consider as a person
MAX_POINT_DISTANCE = 0.3  # Maximum distance between cluster points (meters)
HUMAN_SIZE_RANGE = (0.3, 0.8)  # Expected width of human in meters

def median_filter(values):
    """Apply median filter to remove outliers."""
    if not values:
        return None
    return sorted(values)[len(values)//2]

def exponential_moving_average(new_value, previous_avg, alpha=0.3):
    """Apply exponential moving average for smoothing."""
    if previous_avg is None:
        return new_value
    return alpha * new_value + (1 - alpha) * previous_avg

def correct_bias(raw_distance, bias=0):
    """Corrects bias in UWB sensor readings with dynamic calibration."""
    if raw_distance < 20:  # Very close readings tend to be less accurate
        return max(20, raw_distance - bias)
    elif raw_distance > 300:  # Very far readings might be unreliable
        return min(300, raw_distance - bias)
    return raw_distance - bias

def calculate_rotation(target_angle, current_angle):
    """Calculates required rotation to face target with shortest path."""
    angle_diff = ((target_angle - current_angle) + 180) % 360 - 180
    return angle_diff

def get_direction_name(angle):
    """Convert angle to human-readable direction with finer granularity."""
    directions = [
        "Front", "Front-Right-1", "Front-Right-2", "Right-Front", 
        "Right", "Right-Back", "Back-Right", "Back-Right-2",
        "Back", "Back-Left-2", "Back-Left", "Left-Back", 
        "Left", "Left-Front", "Front-Left-2", "Front-Left-1"
    ]
    
    # Normalize angle to 0-360
    normalized_angle = angle % 360
    
    # Convert to direction index (16 directions for more precision)
    index = round(normalized_angle / 22.5) % 16
    
    return directions[index]

def predict_future_position(prediction_time):
    """Predict where the person will be in the future based on their current velocity."""
    global uwb_history, angle_history, person_velocity, person_angular_velocity
    
    if not uwb_history or not angle_history:
        return None, None
    
    # Calculate future distance
    predicted_distance = uwb_distance + (person_velocity * prediction_time)
    
    # Calculate future angle
    predicted_angle = (target_angle + (person_angular_velocity * prediction_time)) % 360
    
    return predicted_distance, predicted_angle

def update_velocity_estimates():
    """Update velocity estimates based on sensor history."""
    global uwb_history, angle_history, person_velocity, person_angular_velocity
    
    current_time = time.time()
    
    # Update distance velocity if we have enough history
    if len(uwb_history) >= 2 and uwb_distance is not None:
        uwb_history.append((current_time, uwb_distance))
        
        # Calculate velocity based on linear regression
        times = np.array([t for t, _ in uwb_history])
        distances = np.array([d for _, d in uwb_history])
        
        # Normalize times relative to first point
        times = times - times[0]
        
        if len(times) > 1:  # Need at least 2 points for regression
            # Fit line to distance vs time
            coeffs = np.polyfit(times, distances, 1)
            person_velocity = coeffs[0]  # Slope is velocity
    
    # Update angular velocity if we have enough history
    if len(angle_history) >= 2 and target_angle is not None:
        angle_history.append((current_time, target_angle))
        
        # Calculate angular velocities, handling wrap-around
        angular_velocities = []
        for i in range(1, len(angle_history)):
            prev_time, prev_angle = angle_history[i-1]
            curr_time, curr_angle = angle_history[i]
            
            time_diff = curr_time - prev_time
            if time_diff > 0:
                # Handle angle wrap-around
                angle_diff = calculate_rotation(curr_angle, prev_angle)
                angular_velocities.append(angle_diff / time_diff)
        
        if angular_velocities:
            # Filter out extreme values and take mean
            filtered_velocities = [v for v in angular_velocities if abs(v) < 180]
            if filtered_velocities:
                person_angular_velocity = sum(filtered_velocities) / len(filtered_velocities)

def move_robot(forward_speed, rotation_speed, ramp=True):
    """
    Controls robot movement using differential drive with improved smoothing and ramping.
    
    Parameters:
    - forward_speed: Target speed for forward/backward movement (-100 to 100)
    - rotation_speed: Target speed for rotation (-100 to 100)
    - ramp: Whether to ramp speed changes (smooth acceleration)
    """
    # Static variables to track current speeds (using a list to create static-like behavior)
    if not hasattr(move_robot, "current_speeds"):
        move_robot.current_speeds = [0, 0]  # [forward, rotation]
    
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
    # Use different gain for rotation vs forward motion for more natural movement
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

def read_uwb_data():
    """Reads data from UWB sensor with improved filtering."""
    global uwb_distance, running, uwb_distance_buffer
    
    # Initialize smoothed distance
    smoothed_uwb_distance = None
    
    try:
        ser = serial.Serial(PORT_UWB, BAUDRATE_UWB, timeout=TIMEOUT_UWB)
        print("UWB sensor connected.")
        
        while running:
            # Read a line from serial
            line = ser.readline().decode('utf-8').strip()
            
            # Parse UWB data
            try:
                if line.startswith("mc"):
                    parts = line.split(',')
                    if len(parts) >= 2:
                        raw_distance = float(parts[1])
                        
                        # Apply bias correction
                        corrected_distance = correct_bias(raw_distance)
                        
                        # Add to smoothing buffer
                        uwb_distance_buffer.append(corrected_distance)
                        
                        # Apply median filter
                        filtered_distance = median_filter(uwb_distance_buffer)
                        
                        # Apply exponential smoothing
                        smoothed_uwb_distance = exponential_moving_average(filtered_distance, smoothed_uwb_distance)
                        
                        # Update global value
                        uwb_distance = smoothed_uwb_distance
                        
                        # Log the distance if it's changed significantly
                        print(f"UWB Distance: {uwb_distance:.2f} cm")
            except ValueError:
                pass  # Skip invalid data
                
            time.sleep(0.05)
            
    except Exception as e:
        print(f"UWB Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("UWB connection closed.")

def cluster_points(scan_data):
    """
    Cluster LiDAR scan points to identify objects.
    Returns a list of clusters, each containing angle, distance, and points count.
    """
    if not scan_data:
        return []
    
    # Sort by angle for sequential processing
    sorted_scan = sorted(scan_data, key=lambda x: x[0])
    
    clusters = []
    current_cluster = []
    
    for i, (angle, distance) in enumerate(sorted_scan):
        # Check if this is the first point
        if not current_cluster:
            current_cluster.append((angle, distance))
            continue
        
        # Get the last point in the current cluster
        prev_angle, prev_distance = current_cluster[-1]
        
        # Calculate Euclidean distance between points (converting to Cartesian)
        x1 = prev_distance * math.cos(math.radians(prev_angle))
        y1 = prev_distance * math.sin(math.radians(prev_angle))
        x2 = distance * math.cos(math.radians(angle))
        y2 = distance * math.sin(math.radians(angle))
        
        point_distance = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        
        # Check if this point belongs to the current cluster
        if point_distance <= MAX_POINT_DISTANCE:
            current_cluster.append((angle, distance))
        else:
            # Finish the current cluster if it has enough points
            if len(current_cluster) >= MIN_CLUSTER_POINTS:
                # Calculate average values for the cluster
                avg_angle = sum(a for a, _ in current_cluster) / len(current_cluster)
                avg_distance = sum(d for _, d in current_cluster) / len(current_cluster)
                
                clusters.append({
                    'angle': avg_angle,
                    'distance': avg_distance,
                    'points': len(current_cluster),
                    'width': calculate_cluster_width(current_cluster)
                })
            
            # Start a new cluster with the current point
            current_cluster = [(angle, distance)]
    
    # Don't forget the last cluster
    if len(current_cluster) >= MIN_CLUSTER_POINTS:
        avg_angle = sum(a for a, _ in current_cluster) / len(current_cluster)
        avg_distance = sum(d for _, d in current_cluster) / len(current_cluster)
        
        clusters.append({
            'angle': avg_angle,
            'distance': avg_distance,
            'points': len(current_cluster),
            'width': calculate_cluster_width(current_cluster)
        })
    
    return clusters

def calculate_cluster_width(cluster_points):
    """Calculate the width of a cluster in meters."""
    if len(cluster_points) < 2:
        return 0
    
    # Convert polar to Cartesian
    cartesian_points = []
    for angle, distance in cluster_points:
        x = distance * math.cos(math.radians(angle))
        y = distance * math.sin(math.radians(angle))
        cartesian_points.append((x, y))
    
    # Find the maximum distance between any two points
    max_width = 0
    for i in range(len(cartesian_points)):
        for j in range(i+1, len(cartesian_points)):
            x1, y1 = cartesian_points[i]
            x2, y2 = cartesian_points[j]
            width = math.sqrt((x2-x1)**2 + (y2-y1)**2)
            max_width = max(max_width, width)
    
    return max_width

def identify_person_cluster(clusters):
    """
    Identify which cluster is most likely to be a person.
    Considers width, point count, and movement patterns.
    """
    if not clusters:
        return None
    
    person_candidates = []
    
    for cluster in clusters:
        # Check if cluster width is in human range
        if HUMAN_SIZE_RANGE[0] <= cluster['width'] <= HUMAN_SIZE_RANGE[1]:
            # The more points, the more confident we are
            confidence = min(cluster['points'] / MIN_CLUSTER_POINTS, 1.0)
            
            # Add distance factor - closer objects are more likely to be the person
            # but don't weight too heavily to avoid issues when the person is far
            distance_factor = max(0.5, 1.0 - (cluster['distance'] / 5.0))
            
            score = confidence * distance_factor
            
            person_candidates.append((score, cluster))
    
    # Sort by score (highest first)
    person_candidates.sort(reverse=True, key=lambda x: x[0])
    
    # Return the best candidate if any
    if person_candidates and person_candidates[0][0] > 0.4:  # Minimum confidence threshold
        return person_candidates[0][1]
    
    return None

def scan_lidar():
    """Process LiDAR scans to track people."""
    global running, target_angle, target_distance
    
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
                # Cluster the scan data
                clusters = cluster_points(scan_data)
                
                # Identify person
                person = identify_person_cluster(clusters)
                
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
                    smoothed_angle = exponential_moving_average(filtered_angle, smoothed_angle, alpha=0.4)
                    smoothed_distance = exponential_moving_average(filtered_distance, smoothed_distance, alpha=0.4)
                    
                    # Update global values
                    target_angle = smoothed_angle
                    target_distance = smoothed_distance
                    
                    # Update velocity estimates
                    update_velocity_estimates()
                    
                    direction = get_direction_name(target_angle)
                    confidence = person['points']
                    print(f"LiDAR Target: {direction} at angle={target_angle:.2f}°, distance={target_distance/10:.2f}cm (confidence: {confidence})")
            
            time.sleep(0.05)
            
    except Exception as e:
        print(f"LiDAR Error: {e}")
    finally:
        if running:  # Only stop if we're still running (avoid double-stop)
            lidar.stop()
            print("LiDAR stopped.")

def reset_pid_controllers():
    """Reset PID controller state variables."""
    global distance_integral, distance_previous_error, angle_integral, angle_previous_error
    distance_integral = 0.0
    distance_previous_error = 0.0
    angle_integral = 0.0
    angle_previous_error = 0.0

def pid_distance_control(current_distance, target_distance, dt):
    """PID controller for distance."""
    global distance_integral, distance_previous_error
    
    # Calculate error
    error = current_distance - target_distance
    
    # Integrate error with anti-windup
    distance_integral += error * dt
    distance_integral = max(min(distance_integral, 50), -50)  # Limit integral term
    
    # Calculate derivative term
    derivative = (error - distance_previous_error) / dt if dt > 0 else 0
    
    # Save error for next iteration
    distance_previous_error = error
    
    # Calculate PID output
    output = P_DIST * error + I_DIST * distance_integral + D_DIST * derivative
    
    return output

def pid_angle_control(current_angle, target_angle, dt):
    """PID controller for angle."""
    global angle_integral, angle_previous_error
    
    # Calculate error (handle angle wrap-around)
    error = calculate_rotation(target_angle, current_angle)
    
    # Integrate error with anti-windup
    angle_integral += error * dt
    angle_integral = max(min(angle_integral, 50), -50)  # Limit integral term
    
    # Calculate derivative term
    derivative = (error - angle_previous_error) / dt if dt > 0 else 0
    
    # Save error for next iteration
    angle_previous_error = error
    
    # Calculate PID output
    output = P_ANGLE * error + I_ANGLE * angle_integral + D_ANGLE * derivative
    
    return output

def follow_person():
    """Autonomous control logic with PID controllers and predictive tracking."""
    global running, uwb_distance, target_angle, target_distance, autonomous_mode
    global last_update_time
    
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
                
            # Skip if no sensor data yet
            if uwb_distance is None and target_angle is None:
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
                
                # Distance control only if we're facing the right direction
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
                
                # Execute movement with smoothing
                move_robot(forward_command, rotation_command, ramp=True)
                
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

def handle_manual_control():
    """Process manual control inputs with improved responsiveness."""
    global running, autonomous_mode
    
    try:
        last_time = time.time()
        
        while running:
            # Skip if in autonomous mode
            if autonomous_mode:
                time.sleep(0.1)
                continue
                
            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
                
            # Update pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_TAB:
                        autonomous_mode = not autonomous_mode
                        mode_str = "AUTONOMOUS" if autonomous_mode else "MANUAL"
                        print(f"Switching to {mode_str} control mode")
                        # Reset PIDs when switching to autonomous
                        if autonomous_mode:
                            reset_pid_controllers()
            
            # Skip the rest if we switched to autonomous mode
            if autonomous_mode:
                continue
                
            # Get pressed keys
            keys = pygame.key.get_pressed()
            
            # Variable to track if we're in drift mode
            drifting = keys[pygame.K_LSHIFT]
            
            # Reset speeds
            forward_speed = 0
            rotation_speed = 0
            
            # ======== Movement Controls ========
            # Forward/Backward
            if keys[pygame.K_w]:
                forward_speed = NORMAL_SPEED
            elif keys[pygame.K_s]:
                forward_speed = -NORMAL_SPEED
                
            # Left/Right rotation
            if keys[pygame.K_a]:
                rotation_speed = TURNING_SPEED
            elif keys[pygame.K_d]:
                rotation_speed = -TURNING_SPEED
                
            # Apply drift boost if shift is held
            if drifting and (forward_speed != 0 or rotation_speed != 0):
                # Enhance the dominant direction for a drift effect
                if abs(forward_speed) > abs(rotation_speed):
                    forward_speed *= 1.3
                else:
                    rotation_speed *= 1.3
                
                print("DRIFT MODE ENGAGED")
                
            # Space bar to stop immediately
            if keys[pygame.K_SPACE]:
                forward_speed = 0
                rotation_speed = 0
                print("EMERGENCY STOP")
                
            # Execute movement with smoothing
            move_robot(forward_speed, rotation_speed, ramp=True)
            
            # Update display
            screen.fill((0, 0, 0))
            
            # Show current mode
            mode_text = font.render("MANUAL CONTROL MODE", True, (255, 0, 0))
            screen.blit(mode_text, (20, 20))
            
            # Show controls
            controls = [
                "W/S: Forward/Backward",
                "A/D: Rotate Left/Right",
                "SHIFT: Drift Mode",
                "SPACE: Emergency Stop",
                "TAB: Switch to Autonomous"
            ]
            
            for i, control in enumerate(controls):
                text = font.render(control, True, (255, 255, 255))
                screen.blit(text, (20, 70 + i * 30))
                
            # Show sensor data if available
            y_pos = 250
            if uwb_distance is not None:
                uwb_text = font.render(f"UWB Distance: {uwb_distance:.2f} cm", True, (0, 255, 0))
                screen.blit(uwb_text, (20, y_pos))
                y_pos += 30

            if target_angle is not None:
                direction = get_direction_name(target_angle)
                angle_text = font.render(f"Target Direction: {direction} ({target_angle:.1f}°)", True, (0, 255, 0))
                screen.blit(angle_text, (20, y_pos))
                y_pos += 30
                
            if target_distance is not None:
                dist_text = font.render(f"LiDAR Distance: {target_distance/10:.2f} cm", True, (0, 255, 0))
                screen.blit(dist_text, (20, y_pos))
                
            # Update display
            pygame.display.flip()
            clock.tick(30)
                
    except Exception as e:
        print(f"Manual Control Error: {e}")

def update_visualization():
    """Updates visualization data for debugging."""
    global running, angles, distances, colors
    
    try:
        while running:
            # Skip if not in visualization mode
            if not pygame.display.get_active():
                time.sleep(0.1)
                continue
                
            # Clear previous data
            angles = []
            distances = []
            colors = []
            
            # Wait a bit
            time.sleep(0.1)
                
    except Exception as e:
        print(f"Visualization Error: {e}")

def cleanup():
    """Clean up resources and stop motors."""
    global running
    running = False
    
    # Stop motors
    try:
        right_motor.send_rpm(1, 0)
        left_motor.send_rpm(1, 0)
        print("Motors stopped.")
    except:
        pass
    
    # Stop LiDAR
    try:
        lidar.stop()
        lidar.disconnect()
        print("LiDAR disconnected.")
    except:
        pass
    
    # Close pygame
    try:
        pygame.quit()
        print("Pygame closed.")
    except:
        pass
    
    print("Cleanup complete.")

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
