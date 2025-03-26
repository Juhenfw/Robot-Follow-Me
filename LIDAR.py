import math
import numpy as np
import time
import serial
import pygame
import ddsm115 as motor
from rplidar import RPLidar, RPLidarException
from threading import Thread, Lock
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger('follow_robot')

# UWB Configuration
PORT_UWB = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BAUDRATE_UWB = 115200
TIMEOUT_UWB = 1

# LiDAR Configuration - A2M12 settings
PORT_LIDAR = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_f235ae4105e1d247940e6441b646a0b3-if00-port0"
BAUDRATE_LIDAR = 115200  # A2M12 standard baudrate
LIDAR_TIMEOUT = 3  # Timeout for lidar operations in seconds

# Wheel Configuration
RIGHT_WHEEL_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B0049TUZ-if00-port0"
LEFT_WHEEL_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_B0045S9B-if00-port0"

# Global variables
running = False
lidar_lock = Lock()  # Lock for thread-safe lidar access
data_lock = Lock()   # Lock for shared data access

# Sensor data
angles = []
distances = []
colors = []
uwb_distance = None
target_angle = None
target_distance = None
lidar_health = {"connected": False, "error_count": 0, "last_scan_time": 0}

# Control mode
autonomous_mode = True  # True for follow mode, False for manual control

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

# Motor speed parameters
NORMAL_SPEED = 30     # Normal speed
SLOW_SPEED = NORMAL_SPEED // 6  # Slow rotation speed
TURNING_SPEED = NORMAL_SPEED // 4  # Normal turning speed
DRIFT_BOOST = int(NORMAL_SPEED * 1.3)  # Drift speed
STOP = 0

# Robot following parameters
FOLLOW_DISTANCE = 50  # Target distance to maintain (cm)
DISTANCE_TOLERANCE = 50  # Tolerance range (cm)
ANGLE_TOLERANCE = 30  # Angle tolerance (degrees)

# LiDAR filtering parameters
MAX_POINTS_PER_SCAN = 360  # Maximum points to process per scan
MIN_QUALITY = 10           # Minimum quality threshold (A2M12 specific)
SCAN_HISTORY_SIZE = 3      # Number of scans to keep for smoothing
DISTANCE_FILTER_MAX = 5000  # Maximum valid distance in mm
DISTANCE_FILTER_MIN = 150   # Minimum valid distance in mm

# Initialize LiDAR
lidar = None

def initialize_lidar():
    """Initialize the A2M12 LiDAR with proper settings."""
    global lidar, lidar_health
    
    try:
        if lidar is not None:
            try:
                lidar.stop()
                lidar.disconnect()
                time.sleep(0.5)
            except:
                pass
        
        logger.info(f"Initializing A2M12 LiDAR on {PORT_LIDAR} at {BAUDRATE_LIDAR} baud")
        lidar = RPLidar(PORT_LIDAR, baudrate=BAUDRATE_LIDAR, timeout=LIDAR_TIMEOUT)
        
        # Check info and health
        info = lidar.get_info()
        health = lidar.get_health()
        
        logger.info(f"A2M12 LiDAR info: {info}")
        logger.info(f"A2M12 LiDAR health: {health}")
        
        with data_lock:
            lidar_health["connected"] = True
            lidar_health["error_count"] = 0
            lidar_health["last_scan_time"] = time.time()
        
        # Reset motor speed to ensure proper scan rate
        lidar.motor_speed = 0  # First reset
        time.sleep(0.1)
        lidar.motor_speed = 660  # Set to standard A2M12 RPM
        time.sleep(1.0)  # Allow motor to stabilize
        
        return True
    
    except Exception as e:
        logger.error(f"LiDAR initialization error: {e}")
        with data_lock:
            lidar_health["connected"] = False
            lidar_health["error_count"] += 1
        
        return False

def correct_bias(raw_distance, bias):
    """Corrects bias in UWB sensor readings."""
    return raw_distance - bias

def calculate_rotation(target_angle, current_angle):
    """Calculates required rotation to face target."""
    angle_diff = target_angle - current_angle
    
    if angle_diff > 180:
        angle_diff -= 360
    elif angle_diff < -180:
        angle_diff += 360
        
    return angle_diff

def get_direction_name(angle):
    """Convert angle to human-readable direction."""
    directions = ["Front", "Front-Right", "Right", "Back-Right", 
                 "Back", "Back-Left", "Left", "Front-Left"]
    
    # Normalize angle to 0-360
    normalized_angle = angle % 360
    
    # Convert to direction index (8 directions)
    index = round(normalized_angle / 45) % 8
    
    return directions[index]

def move_robot(forward_speed, rotation_speed):
    """
    Controls robot movement using differential drive.
    
    Parameters:
    - forward_speed: Speed for forward/backward movement (-100 to 100)
    - rotation_speed: Speed for rotation (-100 to 100)
    """
    right_speed = forward_speed - rotation_speed
    left_speed = forward_speed + rotation_speed
    
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
    
    logger.debug(f"Movement: {movement_desc} (R={-right_speed:.1f}, L={left_speed:.1f})")

def read_uwb_data():
    """Reads data from UWB sensor."""
    global uwb_distance, running
    
    try:
        ser = serial.Serial(PORT_UWB, BAUDRATE_UWB, timeout=TIMEOUT_UWB)
        logger.info(f"Connected to UWB sensor at {PORT_UWB}")
        
        while running:
            if ser.in_waiting > 0:
                data = ser.readline().decode("utf-8", errors='replace').strip()
                
                if data.startswith("$KT0"):
                    parts = data.split(",")
                    if len(parts) >= 4:
                        raw_values = parts[1:4]
                        processed_values = []
                        
                        for value in raw_values:
                            if value.lower() == "null":
                                processed_values.append(0.0)
                            else:
                                try:
                                    processed_values.append(float(value))
                                except ValueError:
                                    processed_values.append(0.0)
                                
                        A0, A1, A2 = processed_values
                        
                        # Apply bias correction
                        cal_A0 = correct_bias(A0*100, 100)  # Convert to cm and correct bias
                        
                        with data_lock:
                            global uwb_distance
                            uwb_distance = cal_A0
                        
                        logger.debug(f"UWB Distance: {cal_A0:.2f} cm")
            
            time.sleep(0.05)
            
    except serial.SerialException as e:
        logger.error(f"UWB Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            logger.info("UWB serial connection closed.")

def filter_lidar_data(scan_data):
    """
    Filter and process LiDAR scan data for the A2M12.
    Returns filtered angles and distances.
    """
    filtered_angles = []
    filtered_distances = []
    
    # First pass: basic filtering
    for quality, angle, distance in scan_data:
        # Apply A2M12-specific filtering
        if (quality >= MIN_QUALITY and 
            DISTANCE_FILTER_MIN <= distance <= DISTANCE_FILTER_MAX):
            filtered_angles.append(angle)
            filtered_distances.append(distance)
    
    # If we have too many points, sample them
    if len(filtered_angles) > MAX_POINTS_PER_SCAN:
        indices = np.linspace(0, len(filtered_angles)-1, MAX_POINTS_PER_SCAN, dtype=int)
        filtered_angles = [filtered_angles[i] for i in indices]
        filtered_distances = [filtered_distances[i] for i in indices]
    
    return filtered_angles, filtered_distances

def detect_person(angles, distances):
    """
    Advanced algorithm to detect a person from LiDAR data.
    Returns the angle and distance to the detected person.
    """
    if not angles or not distances:
        return None, None
    
    # Convert to numpy arrays for easier processing
    angles_array = np.array(angles)
    distances_array = np.array(distances)
    
    # Typical human width at different distances (simplified)
    # At 1m, a human might be ~30-40° wide in scan data
    min_cluster_size = 3  # Minimum points to consider as a potential person
    max_point_distance = 300  # Maximum distance between points in a cluster (mm)
    
    # Find potential clusters (groups of points close together)
    clusters = []
    current_cluster = []
    
    # Sort points by angle for clustering
    sorted_indices = np.argsort(angles_array)
    sorted_angles = angles_array[sorted_indices]
    sorted_distances = distances_array[sorted_indices]
    
    # Use clustering to detect person-sized objects
    for i in range(len(sorted_angles)):
        if not current_cluster:
            current_cluster = [(sorted_angles[i], sorted_distances[i])]
        else:
            last_angle, last_distance = current_cluster[-1]
            
            # Check if this point is close to the last point
            # Using both angular and distance proximity
            angle_diff = min(abs(sorted_angles[i] - last_angle), 
                            360 - abs(sorted_angles[i] - last_angle))
            
            # Points are close if they're within a certain angle and distance range
            if ((angle_diff < 10 and abs(sorted_distances[i] - last_distance) < max_point_distance) or
                (angle_diff < 5)):
                current_cluster.append((sorted_angles[i], sorted_distances[i]))
            else:
                # End of cluster, check if it's large enough
                if len(current_cluster) >= min_cluster_size:
                    clusters.append(current_cluster)
                # Start a new cluster
                current_cluster = [(sorted_angles[i], sorted_distances[i])]
    
    # Check the last cluster
    if len(current_cluster) >= min_cluster_size:
        clusters.append(current_cluster)
    
    # No person-like clusters found
    if not clusters:
        return None, None
    
    # Select the most person-like cluster (closest and of reasonable size)
    best_cluster = None
    best_score = float('inf')  # Lower is better
    
    for cluster in clusters:
        # Calculate cluster properties
        cluster_size = len(cluster)
        avg_distance = sum(point[1] for point in cluster) / cluster_size
        
        # Score: prioritize closer clusters with more points
        score = avg_distance / (cluster_size ** 0.5)  # Square root to reduce over-emphasis on size
        
        if score < best_score:
            best_score = score
            best_cluster = cluster
    
    # Calculate the center of the best cluster
    if best_cluster:
        avg_angle = sum(point[0] for point in best_cluster) / len(best_cluster)
        avg_distance = sum(point[1] for point in best_cluster) / len(best_cluster)
        return avg_angle, avg_distance
    
    return None, None

def update_lidar():
    """Updates LiDAR data continuously for A2M12."""
    global running, angles, distances, colors, target_angle, target_distance, lidar_health
    
    scan_history = []  # For temporal smoothing
    last_error_time = 0
    reconnect_cooldown = 5  # Seconds between reconnection attempts
    
    # Initialize LiDAR
    if not initialize_lidar():
        logger.error("Failed to initialize A2M12 LiDAR")
        with data_lock:
            lidar_health["connected"] = False
    
    try:
        while running:
            try:
                # Check if LiDAR needs reconnection
                with data_lock:
                    connected = lidar_health["connected"]
                    last_scan = lidar_health["last_scan_time"]
                
                if not connected and time.time() - last_error_time > reconnect_cooldown:
                    logger.info("Attempting to reconnect A2M12 LiDAR...")
                    if initialize_lidar():
                        logger.info("A2M12 LiDAR reconnected successfully")
                    else:
                        last_error_time = time.time()
                        time.sleep(1)
                        continue
                
                # Check for stale data
                if time.time() - last_scan > 5:  # No scans for 5 seconds
                    logger.warning("LiDAR data is stale, attempting to restart...")
                    initialize_lidar()
                    time.sleep(1)
                    continue
                
                # Get a single scan (optimized for A2M12)
                with lidar_lock:
                    scan = list(next(lidar.iter_scans()))
                
                with data_lock:
                    lidar_health["last_scan_time"] = time.time()
                    lidar_health["connected"] = True
                
                # Process and filter scan data
                filtered_angles, filtered_distances = filter_lidar_data(scan)
                
                # Add to history for temporal smoothing
                scan_history.append((filtered_angles, filtered_distances))
                if len(scan_history) > SCAN_HISTORY_SIZE:
                    scan_history.pop(0)
                
                # Detect person from filtered data
                person_angle, person_distance = detect_person(filtered_angles, filtered_distances)
                
                # Update global target if person detected
                if person_angle is not None and person_distance is not None:
                    with data_lock:
                        target_angle = person_angle
                        target_distance = person_distance
                    
                    direction = get_direction_name(person_angle)
                    logger.debug(f"LiDAR Target: {direction} at angle={person_angle:.2f}°, distance={person_distance/10:.2f}cm")
                
                # Scan rate control - A2M12 works best with some delay between processing scans
                time.sleep(0.05)
            
            except RPLidarException as e:
                logger.error(f"A2M12 LiDAR error: {e}")
                with data_lock:
                    lidar_health["connected"] = False
                    lidar_health["error_count"] += 1
                
                last_error_time = time.time()
                
                # Try to recover
                try:
                    with lidar_lock:
                        lidar.stop()
                        time.sleep(0.5)
                except:
                    pass
                
                time.sleep(1)  # Cool-down before retry
            
            except StopIteration:
                logger.warning("A2M12 LiDAR scan iteration ended, restarting...")
                initialize_lidar()
                time.sleep(0.5)
            
            except Exception as e:
                logger.error(f"Unexpected A2M12 LiDAR error: {e}")
                last_error_time = time.time()
                time.sleep(1)
    
    except KeyboardInterrupt:
        logger.info("LiDAR operation stopped by user.")
    finally:
        if running:  # Only stop if we're still running (avoid double-stop)
            try:
                with lidar_lock:
                    lidar.stop()
                    lidar.disconnect()
                logger.info("A2M12 LiDAR stopped and disconnected.")
            except:
                pass

def follow_person():
    """Autonomous control logic to follow a person."""
    global running, uwb_distance, target_angle, target_distance, autonomous_mode
    
    try:
        while running:
            # Skip if in manual mode
            if not autonomous_mode:
                time.sleep(0.1)
                continue
                
            # Get the latest data thread-safely
            with data_lock:
                curr_uwb_distance = uwb_distance
                curr_target_angle = target_angle
                curr_target_distance = target_distance
            
            if curr_uwb_distance is None or curr_target_angle is None:
                # No sensor data yet, wait
                time.sleep(0.1)
                continue
                
            # Default to stopped
            forward_command = 0
            rotation_command = 0
            
            # Determine rotation command (based on LiDAR angle)
            if curr_target_angle is not None:
                angle_error = calculate_rotation(curr_target_angle, 0)  # Assuming 0 is forward
                
                direction = get_direction_name(curr_target_angle)
                
                if abs(angle_error) > ANGLE_TOLERANCE:
                    # Need to rotate to face person
                    # Use proportional control for smoother rotation
                    rotation_speed = min(max(angle_error / 3, -TURNING_SPEED), TURNING_SPEED)
                    rotation_command = rotation_speed
                    
                    if rotation_speed > 0:
                        logger.debug(f"Navigation: Need to rotate LEFT to face target at {direction}")
                    else:
                        logger.debug(f"Navigation: Need to rotate RIGHT to face target at {direction}")
                else:
                    logger.debug(f"Navigation: Target angle good, facing {direction}")
                
            # Determine forward/backward command (based on UWB distance)
            if curr_uwb_distance is not None:
                distance_error = curr_uwb_distance - FOLLOW_DISTANCE
                
                if abs(distance_error) > DISTANCE_TOLERANCE:
                    # Need to adjust distance - use proportional control
                    forward_scale = 0.3  # Scale factor to smooth movement
                    if distance_error > 0:
                        # Too far, move forward
                        forward_speed = min(distance_error * forward_scale, NORMAL_SPEED)
                        forward_command = forward_speed
                        logger.debug(f"Navigation: Moving FORWARD {distance_error:.2f}cm")
                    else:
                        # Too close, move backward
                        forward_speed = max(distance_error * forward_scale, -NORMAL_SPEED)
                        forward_command = forward_speed
                        logger.debug(f"Navigation: Moving BACKWARD {-distance_error:.2f}cm")
                else:
                    logger.debug(f"Navigation: Distance good at {curr_uwb_distance:.2f}cm")
            
            # Execute movement with smoothing
            move_robot(forward_command, rotation_command)
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        logger.info("Follow operation stopped by user.")
    except Exception as e:
        logger.error(f"Follow Error: {e}")
        # Attempt to recover without stopping program
        time.sleep(1)

def handle_manual_control():
    """Process manual control inputs."""
    global running, autonomous_mode
    
    try:
        while running:
            # Handle pygame events in both modes for mode switching
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_TAB:
                        autonomous_mode = not autonomous_mode
                        mode_str = "AUTONOMOUS" if autonomous_mode else "MANUAL"
                        logger.info(f"Switching to {mode_str} control mode")
                    elif event.key == pygame.K_p:
                        running = False
            
            # Skip motion control if in autonomous mode
            if autonomous_mode:
                time.sleep(0.1)
                continue
            
            # Get pressed keys
            keys = pygame.key.get_pressed()
            
            # Variable to track if we're in drift mode
            drifting = keys[pygame.K_LSHIFT]
            
            # Reset speeds
            right_speed = STOP
            left_speed = STOP
            
            # ======== Combination Keys for Turning & Drift ========
            if keys[pygame.K_w] and keys[pygame.K_a]:  # Forward + Left
                if drifting:  # Drift mode
                    right_speed = -DRIFT_BOOST
                    left_speed = TURNING_SPEED
                else:  # Normal mode
                    right_speed = -NORMAL_SPEED
                    left_speed = TURNING_SPEED
            elif keys[pygame.K_w] and keys[pygame.K_d]:  # Forward + Right
                if drifting:
                    right_speed = -TURNING_SPEED
                    left_speed = -DRIFT_BOOST
                else:
                    right_speed = -TURNING_SPEED
                    left_speed = NORMAL_SPEED
            elif keys[pygame.K_s] and keys[pygame.K_a]:  # Backward + Left
                if drifting:
                    right_speed = NORMAL_SPEED
                    left_speed = -DRIFT_BOOST
                else:
                    right_speed = NORMAL_SPEED
                    left_speed = -TURNING_SPEED
            elif keys[pygame.K_s] and keys[pygame.K_d]:  # Backward + Right
                if drifting:
                    right_speed = DRIFT_BOOST
                    left_speed = -NORMAL_SPEED
                else:
                    right_speed = TURNING_SPEED
                    left_speed = -NORMAL_SPEED
                    
            # ======== Normal Movement Controls ========
            elif keys[pygame.K_w]:  # Forward
                right_speed = -NORMAL_SPEED
                left_speed = NORMAL_SPEED
            elif keys[pygame.K_s]:  # Backward
                right_speed = NORMAL_SPEED
                left_speed = -NORMAL_SPEED
            elif keys[pygame.K_a]:  # Rotate left
                right_speed = -SLOW_SPEED
                left_speed = -SLOW_SPEED
            elif keys[pygame.K_d]:  # Rotate right
                right_speed = SLOW_SPEED
                left_speed = SLOW_SPEED
            elif keys[pygame.K_SPACE]:  # Fast rotation
                right_speed = -NORMAL_SPEED
                left_speed = -NORMAL_SPEED
                
            # Send commands to motors
            right_motor.send_rpm(1, right_speed)
            left_motor.send_rpm(1, left_speed)
            
            # Control loop rate
            time.sleep(0.05)
            
    except Exception as e:
        logger.error(f"Manual control error: {e}")

def update_ui():
    """Updates pygame UI in both modes."""
    global running, autonomous_mode, uwb_distance, target_angle, target_distance, lidar_health
    
    try:
        while running:
            # Update display
            screen.fill((0, 0, 0))
            
            # Display current mode
            mode_text = "MODE: AUTONOMOUS FOLLOW" if autonomous_mode else "MODE: MANUAL CONTROL"
            mode_color = (0, 255, 0) if autonomous_mode else (255, 100, 100)
            mode_surface = font.render(mode_text, True, mode_color)
            screen.blit(mode_surface, (20, 20))
            
            # Display LiDAR status
            with data_lock:
                connected = lidar_health["connected"]
                error_count = lidar_health["error_count"]
                last_scan = time.time() - lidar_health["last_scan_time"]
            
            lidar_status_color = (0, 255, 0) if connected and last_scan < 1 else (255, 0, 0)
            lidar_status_text = f"A2M12 LiDAR: {'CONNECTED' if connected else 'DISCONNECTED'}"
            lidar_status_surface = font.render(lidar_status_text, True, lidar_status_color)
            screen.blit(lidar_status_surface, (20, 600 - 80))
            
            if last_scan < 60:
                scan_time_text = f"Last scan: {last_scan:.1f}s ago (errors: {error_count})"
                scan_time_surface = font.render(scan_time_text, True, (200, 200, 200))
                screen.blit(scan_time_surface, (20, 600 - 50))
            
            # Display control instructions
            controls = [
                "TAB: Switch Mode",
                "W/A/S/D: Movement (Manual)",
                "SPACE: Fast Rotation (Manual)",
                "SHIFT: Drift Mode (Manual)",
                "P: Exit Program"
            ]
            
            for i, control in enumerate(controls):
                control_surface = font.render(control, True, (200, 200, 200))
                screen.blit(control_surface, (20, 70 + i * 30))
                
            # Display sensor data if available
            with data_lock:
                curr_uwb_distance = uwb_distance
                curr_target_angle = target_angle
                curr_target_distance = target_distance
                
            if curr_uwb_distance is not None:
                uwb_text = f"UWB Distance: {curr_uwb_distance:.2f} cm"
                uwb_surface = font.render(uwb_text, True, (255, 200, 0))
                screen.blit(uwb_surface, (20, 300))
                
            if curr_target_angle is not None and curr_target_distance is not None:
                target_dir = get_direction_name(curr_target_angle)
                lidar_text = f"Target: {target_dir} at {curr_target_distance/10:.2f} cm"
                lidar_surface = font.render(lidar_text, True, (0, 255, 200))
                screen.blit(lidar_surface, (20, 340))
                
                # Draw a simple visualization of target position
                center_x, center_y = 600, 400
                radius = 150
                # Convert angle to radians and adjust for coordinate system
                angle_rad = math.radians(90 - curr_target_angle)
                # Calculate target position
                target_x = center_x + radius * math.cos(angle_rad) 
                target_y = center_y - radius * math.sin(angle_rad)
                
                # Draw circle representing robot's sensing range
                pygame.draw.circle(screen, (50, 50, 50), (center_x, center_y), radius, 1)
                # Draw line pointing to target
                pygame.draw.line(screen, (0, 200, 0), (center_x, center_y), (target_x, target_y), 2)
                # Draw robot position
                pygame.draw.circle(screen, (200, 200, 200), (center_x, center_y), 15)
                # Draw target position
                pygame.draw.circle(screen, (255, 0, 0), (int(target_x), int(target_y)), 8)
                
                # Label directions
                dirs = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
                for i, d in enumerate(dirs):
                    angle = math.radians(i * 45)
                    dir_x = center_x + (radius + 20) * math.sin(angle)
                    dir_y = center_y - (radius + 20) * math.cos(angle)
                    dir_surface = font.render(d, True, (100, 100, 100))
                    screen.blit(dir_surface, (int(dir_x), int(dir_y)))
            
            pygame.display.flip()
            clock.tick(30)  # 30 FPS for UI updates
            
    except Exception as e:
        logger.error(f"UI Error: {e}")

def cleanup():
    """Safely clean up resources."""
    global lidar, running
    
    logger.info("Shutting down robot system...")
    running = False
    time.sleep(0.5)  # Allow threads to notice shutdown
    
    # Stop all motors
    try:
        right_motor.send_rpm(1, 0)
        left_motor.send_rpm(1, 0)
        logger.info("Motors stopped.")
    except Exception as e:
        logger.error(f"Error stopping motors: {e}")
    
    # Close LiDAR safely
    try:
        with lidar_lock:
            if lidar:
                lidar.stop()
                lidar.disconnect()
        logger.info("A2M12 LiDAR disconnected.")
    except Exception as e:
        logger.error(f"Error closing LiDAR: {e}")
    
    # Close pygame
    try:
        pygame.quit()
    except:
        pass
    
    logger.info("Cleanup complete, exiting program.")

def main():
    """Main function to run the robot system."""
    global running, lidar
    
    try:
        # Set running flag for threads
        running = True
        
        # Initialize LiDAR
        if not initialize_lidar():
            logger.error("Failed to initialize A2M12 LiDAR. Check connections.")
            return
        
        # Start threads
        logger.info("Starting system threads...")
        
        # Start UWB reading thread
        uwb_thread = Thread(target=read_uwb_data, daemon=True)
        uwb_thread.start()
        
        # Start LiDAR scanning thread
        lidar_thread = Thread(target=process_lidar_data, daemon=True)
        lidar_thread.start()
        
        # Start autonomous follow thread
        follow_thread = Thread(target=follow_person, daemon=True)
        follow_thread.start()
        
        # Start manual control thread
        control_thread = Thread(target=handle_manual_control, daemon=True)
        control_thread.start()
        
        # Start UI update thread
        ui_thread = Thread(target=update_ui, daemon=True)
        ui_thread.start()
        
        logger.info("All threads started. Press TAB to switch modes, P to exit.")
        
        # Main thread monitors for keyboard interrupt
        while running:
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        logger.info("Program terminated by user.")
    except Exception as e:
        logger.error(f"Unexpected error in main thread: {e}")
    finally:
        cleanup()

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        logger.critical(f"Fatal error: {e}")
        # Ensure motors are stopped in case of fatal error
        try:
            right_motor.send_rpm(1, 0)
            left_motor.send_rpm(1, 0)
        except:
            pass

