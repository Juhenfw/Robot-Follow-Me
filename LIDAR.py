import math
import numpy as np
import time
import serial
import pygame
import ddsm115 as motor
from rplidar import RPLidar
from threading import Thread

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

# Motor speed parameters
NORMAL_SPEED = 75  # Normal speed
SLOW_SPEED = NORMAL_SPEED // 6  # Slow rotation speed
TURNING_SPEED = NORMAL_SPEED // 4  # Normal turning speed
DRIFT_BOOST = int(NORMAL_SPEED * 1.3)  # Drift speed
STOP = 0

# Robot following parameters
FOLLOW_DISTANCE = 150  # Target distance to maintain (cm)
DISTANCE_TOLERANCE = 30  # Tolerance range (cm)
ANGLE_TOLERANCE = 10  # Angle tolerance (degrees)

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
    
    print(f"Movement: {movement_desc} (R={-right_speed:.1f}, L={left_speed:.1f})")

def read_uwb_data():
    """Reads data from UWB sensor."""
    global uwb_distance, running
    
    try:
        ser = serial.Serial(PORT_UWB, BAUDRATE_UWB, timeout=TIMEOUT_UWB)
        print(f"Connected to UWB sensor at {PORT_UWB}")
        
        while running:
            if ser.in_waiting > 0:
                data = ser.readline().decode("utf-8").strip()
                
                if data.startswith("$KT0"):
                    parts = data.split(",")
                    if len(parts) >= 4:
                        raw_values = parts[1:4]
                        processed_values = []
                        
                        for value in raw_values:
                            if value.lower() == "null":
                                processed_values.append(0.0)
                            else:
                                processed_values.append(float(value))
                                
                        A0, A1, A2 = processed_values
                        
                        # Apply bias correction
                        cal_A0 = correct_bias(A0*100, 15)  # Convert to cm and correct bias
                        uwb_distance = cal_A0
                        print(f"UWB Distance: {cal_A0:.2f} cm")
                
            time.sleep(0.05)
            
    except serial.SerialException as e:
        print(f"UWB Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("UWB serial connection closed.")

def update_lidar():
    """Updates LiDAR data continuously."""
    global running, angles, distances, colors, target_angle, target_distance
    
    try:
        for scan in lidar.iter_scans():
            if not running:
                break
                
            scan_angles = []
            scan_distances = []
            
            for _, angle, distance in scan:
                if distance <= 2000:  # Limit to 2000 mm
                    scan_angles.append(angle)
                    scan_distances.append(distance)
            
            if scan_distances:
                # Find clusters of points that might represent a person
                # Simple approach: find the closest point within a reasonable range
                min_distance_idx = scan_distances.index(min(scan_distances))
                target_angle = scan_angles[min_distance_idx]
                target_distance = scan_distances[min_distance_idx]
                
                direction = get_direction_name(target_angle)
                print(f"LiDAR Target: {direction} at angle={target_angle:.2f}°, distance={target_distance/10:.2f}cm")
            
            time.sleep(0.05)
            
    except Exception as e:
        print(f"LiDAR Error: {e}")
    finally:
        if running:  # Only stop if we're still running (avoid double-stop)
            lidar.stop()
            print("LiDAR stopped.")

def follow_person():
    """Autonomous control logic to follow a person."""
    global running, uwb_distance, target_angle, target_distance, autonomous_mode
    
    try:
        while running:
            # Skip if in manual mode
            if not autonomous_mode:
                time.sleep(0.1)
                continue
                
            if uwb_distance is None or target_angle is None:
                # No sensor data yet, wait
                time.sleep(0.1)
                continue
                
            # Default to stopped
            forward_command = 0
            rotation_command = 0
            
            # Determine rotation command (based on LiDAR angle)
            if target_angle is not None:
                angle_error = calculate_rotation(target_angle, 0)  # Assuming 0 is forward
                
                direction = get_direction_name(target_angle)
                
                if abs(angle_error) > ANGLE_TOLERANCE:
                    # Need to rotate to face person
                    rotation_speed = min(max(angle_error / 2, -NORMAL_SPEED), NORMAL_SPEED)
                    rotation_command = rotation_speed
                    
                    if rotation_speed > 0:
                        print(f"Navigation: Need to rotate LEFT to face target at {direction}")
                    else:
                        print(f"Navigation: Need to rotate RIGHT to face target at {direction}")
                else:
                    print(f"Navigation: Target angle good, facing {direction}")
                
            # Determine forward/backward command (based on UWB distance)
            if uwb_distance is not None:
                distance_error = uwb_distance - FOLLOW_DISTANCE
                
                if abs(distance_error) > DISTANCE_TOLERANCE:
                    # Need to adjust distance
                    if distance_error > 0:
                        # Too far, move forward
                        forward_speed = min(distance_error / 3, NORMAL_SPEED)
                        forward_command = forward_speed
                        print(f"Navigation: Need to move FORWARD {distance_error:.2f}cm to reach target distance")
                    else:
                        # Too close, move backward
                        forward_speed = max(distance_error / 3, -NORMAL_SPEED)
                        forward_command = forward_speed
                        print(f"Navigation: Need to move BACKWARD {-distance_error:.2f}cm (too close)")
                else:
                    print(f"Navigation: Distance good at {uwb_distance:.2f}cm")
            
            # Execute movement
            move_robot(forward_command, rotation_command)
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("Follow operation stopped by user.")
    except Exception as e:
        print(f"Follow Error: {e}")

def handle_manual_control():
    """Process manual control inputs."""
    global running, autonomous_mode
    
    try:
        while running:
            # Skip if in autonomous mode
            if autonomous_mode:
                time.sleep(0.1)
                continue
                
            # Update pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_TAB:
                        autonomous_mode = not autonomous_mode
                        mode_str = "AUTONOMOUS" if autonomous_mode else "MANUAL"
                        print(f"Switching to {mode_str} control mode")
            
            # Get pressed keys
            keys = pygame.key.get_pressed()
            
            # Check for mode switch
            if keys[pygame.K_TAB]:
                # Already handled in event loop to avoid repeats
                pass
                
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
            elif keys[pygame.K_p]:  # Exit program
                running = False
                
            # Send commands to motors
            right_motor.send_rpm(1, right_speed)
            left_motor.send_rpm(1, left_speed)
            
            # Update display
            screen.fill((0, 0, 0))
            
            # Display current mode
            mode_text = "MODE: MANUAL CONTROL" if not autonomous_mode else "MODE: AUTONOMOUS FOLLOW"
            mode_surface = font.render(mode_text, True, (255, 255, 255))
            screen.blit(mode_surface, (20, 20))
            
            # Display control instructions
            controls = [
                "TAB: Switch Mode",
                "W/A/S/D: Movement",
                "SPACE: Fast Rotation",
                "SHIFT: Drift Mode",
                "P: Exit Program"
            ]
            
            for i, control in enumerate(controls):
                control_surface = font.render(control, True, (200, 200, 200))
                screen.blit(control_surface, (20, 70 + i * 30))
                
            # Display sensor data if available
            if uwb_distance is not None:
                uwb_text = f"UWB Distance: {uwb_distance:.2f} cm"
                uwb_surface = font.render(uwb_text, True, (255, 200, 0))
                screen.blit(uwb_surface, (20, 300))
                
            if target_angle is not None and target_distance is not None:
                lidar_text = f"Target: {get_direction_name(target_angle)} at {target_distance/10:.2f} cm"
                lidar_surface = font.render(lidar_text, True, (0, 255, 200))
                screen.blit(lidar_surface, (20, 340))
                
            # Update display
            pygame.display.flip()
            clock.tick(30)  # Limit to 30 FPS
            
    except Exception as e:
        print(f"Manual control error: {e}")

def update_ui():
    """Updates pygame UI in both modes."""
    global running, autonomous_mode, uwb_distance, target_angle, target_distance
    
    try:
        while running:
            # Handle mode switching
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_TAB:
                        autonomous_mode = not autonomous_mode
                        mode_str = "AUTONOMOUS" if autonomous_mode else "MANUAL"
                        print(f"Switching to {mode_str} control mode")
                    elif event.key == pygame.K_p:
                        running = False
            
            # Update display
            screen.fill((0, 0, 0))
            
            # Display current mode
            mode_text = "MODE: AUTONOMOUS FOLLOW" if autonomous_mode else "MODE: MANUAL CONTROL"
            mode_color = (0, 255, 0) if autonomous_mode else (255, 100, 100)
            mode_surface = font.render(mode_text, True, mode_color)
            screen.blit(mode_surface, (20, 20))
            
            # Display control instructions
            controls = [
                "TAB: Switch Mode",
                "W/A/S/D: Movement (Manual Mode)",
                "SPACE: Fast Rotation (Manual Mode)",
                "SHIFT: Drift Mode (Manual Mode)",
                "P: Exit Program"
            ]
            
            for i, control in enumerate(controls):
                control_surface = font.render(control, True, (200, 200, 200))
                screen.blit(control_surface, (20, 70 + i * 30))
                
            # Display sensor data if available
            if uwb_distance is not None:
                uwb_text = f"UWB Distance: {uwb_distance:.2f} cm"
                uwb_surface = font.render(uwb_text, True, (255, 200, 0))
                screen.blit(uwb_surface, (20, 300))
                
            if target_angle is not None and target_distance is not None:
                target_dir = get_direction_name(target_angle)
                lidar_text = f"Target: {target_dir} at {target_distance/10:.2f} cm"
                lidar_surface = font.render(lidar_text, True, (0, 255, 200))
                screen.blit(lidar_surface, (20, 340))
                
                # Draw a simple visualization of target position
                center_x, center_y = 600, 400
                radius = 150
                # Convert angle to radians and adjust for coordinate system
                angle_rad = math.radians(90 - target_angle)
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
                    dir_rect = dir_surface.get_rect(center=(dir_x, dir_y))
                    screen.blit(dir_surface, dir_rect)
                
            # Update display
            pygame.display.flip()
            clock.tick(30)  # Limit to 30 FPS
            
    except Exception as e:
        print(f"UI update error: {e}")

def cleanup():
    """Cleans up resources before exiting."""
    global running
    
    running = False
    time.sleep(0.5)  # Allow threads to terminate
    
    # Stop motors
    right_motor.send_rpm(1, 0)
    left_motor.send_rpm(1, 0)
    right_motor.close()
    left_motor.close()
    
    # Stop LiDAR
    try:
        lidar.stop()
        lidar.disconnect()
    except:
        pass
    
    # Quit pygame
    pygame.quit()
    
    print("All systems stopped and disconnected.")

def main():
    """Main program entry point."""
    global running, autonomous_mode
    
    print("Starting Follow Me Robot System...")
    print("Press TAB to switch between autonomous and manual control modes")
    
    running = True
    autonomous_mode = True  # Start in autonomous mode by default
    
    # Start sensor threads
    uwb_thread = Thread(target=read_uwb_data, daemon=True)
    lidar_thread = Thread(target=update_lidar, daemon=True)
    follow_thread = Thread(target=follow_person, daemon=True)
    ui_thread = Thread(target=update_ui, daemon=True)
    
    uwb_thread.start()
    lidar_thread.start()
    follow_thread.start()
    ui_thread.start()
    
    time.sleep(2)  # Give sensors time to start up
    
    try:
        # Handle manual control (if active)
        handle_manual_control()
    except KeyboardInterrupt:
        print("Program terminated by user.")
    finally:
        cleanup()

if __name__ == "__main__":
    main()
