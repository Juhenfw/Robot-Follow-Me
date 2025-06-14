#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from robot_control_system.ddsm115 import MotorControl
import os

class RobotModeController(Node):
    def __init__(self):
        super().__init__('robot_mode_controller')
        
        # Parameters
        self.declare_parameter('default_mode', 'manual')
        self.current_mode = self.get_parameter('default_mode').get_parameter_value().string_value

        self.r_wheel_port = "/dev/ttyRS485-1"
        self.l_wheel_port = "/dev/ttyRS485-2"

        self.speed = 100
        self.lambat = self.speed / 4
        self.serong = self.speed / 3
        self.drift_boost = self.speed * 1.3
        self.stop = 0

        self.running = True
        self.current_speed_rwheel = self.stop
        self.current_speed_lwheel = self.stop

        self.motor_kiri = None
        self.motor_kanan = None
        self.connect_motors()
        
        # Obstacle avoidance parameters
        self.declare_parameter('safety_distance', 0.5)      # meters
        self.declare_parameter('danger_distance', 0.3)      # meters
        self.declare_parameter('scan_angle_range', 180)     # degrees
        self.declare_parameter('max_linear_speed', 0.2)     # m/s
        self.declare_parameter('max_angular_speed', 0.5)    # rad/s
        
        self.safety_distance = self.get_parameter('safety_distance').get_parameter_value().double_value
        self.danger_distance = self.get_parameter('danger_distance').get_parameter_value().double_value
        self.scan_angle_range = self.get_parameter('scan_angle_range').get_parameter_value().integer_value
        self.max_linear_speed = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.mode_pub = self.create_publisher(String, 'robot/current_mode', 10)
        
        # Subscribers
        self.gamepad_sub = self.create_subscription(
            Twist, 'gamepad/cmd_vel', self.gamepad_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.mode_switch_sub = self.create_subscription(
            String, 'robot/mode_switch', self.mode_switch_callback, 10)
        self.shutdown_sub = self.create_subscription(
            String, '/system/shutdown', self.shutdown_callback, 10)
        
        # Timer for autonomous behavior and status updates
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # State variables
        self.latest_scan = None
        self.obstacle_detected = False
        self.obstacle_sectors = [False, False, False]  # [left, center, right]
        self.min_distance = float('inf')
        self.obstacle_direction = 0.0  # Direction to obstacle (-1 to 1, left to right)
        self.last_turn_direction = 1.0  # Track last turn direction (1.0=left, -1.0=right)
        self.stuck_timer = 0  # Count iterations of being stuck
        
        self.get_logger().info(f'Robot Mode Controller initialized in {self.current_mode} mode')
        self.get_logger().info(f'Safety distance: {self.safety_distance}m')
        
    def verify_port_exists(self, port):
        return os.path.exists(port)    
    
    def gamepad_callback(self, msg):
        """Handle gamepad commands in manual mode"""
        if self.current_mode == 'manual':
            # In manual mode, forward gamepad commands but check for safety
            if self.obstacle_detected and msg.linear.x > 0:
                # If obstacle ahead and trying to move forward
                safe_msg = Twist()
                safe_msg.angular.z = msg.angular.z  # Keep turning ability
                self.get_logger().warn('Obstacle detected! Forward motion restricted.')
                self.cmd_vel_pub.publish(safe_msg)
            else:
                # No obstacle or not moving toward it, allow command
                self.cmd_vel_pub.publish(msg)
    
    def scan_callback(self, msg):
        """Process LiDAR scan data for obstacle detection"""
        self.latest_scan = msg
        
        if not msg.ranges:
            return
            
        # Filter out invalid readings
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Only consider the forward arc for navigation
        arc_angle = math.radians(self.scan_angle_range)
        arc_start = -arc_angle/2
        arc_end = arc_angle/2
        
        # Find indices within our arc of interest
        arc_indices = np.where((angles >= arc_start) & (angles <= arc_end))[0]
        arc_ranges = ranges[arc_indices]
        arc_angles = angles[arc_indices]
        
        # Filter out invalid readings
        valid_indices = np.where(~np.isnan(arc_ranges) & ~np.isinf(arc_ranges) & (arc_ranges > 0.1))[0]
        
        if len(valid_indices) == 0:
            # No valid readings in our arc
            self.obstacle_detected = False
            self.obstacle_sectors = [False, False, False]
            self.min_distance = float('inf')
            return
            
        valid_ranges = arc_ranges[valid_indices]
        valid_angles = arc_angles[valid_indices]
        
        # Find minimum distance and its angle
        min_idx = np.argmin(valid_ranges)
        self.min_distance = valid_ranges[min_idx]
        min_angle = valid_angles[min_idx]
        
        # Normalize angle to [-1, 1] for direction
        self.obstacle_direction = min_angle / (arc_angle/2)
        
        # Update obstacle detection status
        self.obstacle_detected = self.min_distance < self.safety_distance
        
        # Divide arc into three sectors (left, center, right)
        sector_size = arc_angle / 3
        left_arc = np.where((valid_angles >= arc_start) & (valid_angles < arc_start + sector_size))[0]
        center_arc = np.where((valid_angles >= arc_start + sector_size) & (valid_angles < arc_end - sector_size))[0]
        right_arc = np.where((valid_angles >= arc_end - sector_size) & (valid_angles <= arc_end))[0]
        
        # Check if each sector has obstacles
        self.obstacle_sectors[0] = False  # Left
        self.obstacle_sectors[1] = False  # Center
        self.obstacle_sectors[2] = False  # Right
        
        # Weighted detection - closer obstacles have higher importance
        if len(left_arc) > 0:
            left_ranges = valid_ranges[left_arc]
            # Consider more than just minimum - use weighted approach
            left_close_count = np.sum(left_ranges < self.safety_distance)
            left_close_ratio = left_close_count / len(left_arc) if len(left_arc) > 0 else 0
            self.obstacle_sectors[0] = left_close_ratio > 0.3 or np.min(left_ranges) < self.safety_distance * 0.7
            
        if len(center_arc) > 0:
            center_ranges = valid_ranges[center_arc]
            # Center is most critical
            center_close_count = np.sum(center_ranges < self.safety_distance)
            center_close_ratio = center_close_count / len(center_arc) if len(center_arc) > 0 else 0
            self.obstacle_sectors[1] = center_close_ratio > 0.2 or np.min(center_ranges) < self.safety_distance * 0.8
            
        if len(right_arc) > 0:
            right_ranges = valid_ranges[right_arc]
            right_close_count = np.sum(right_ranges < self.safety_distance)
            right_close_ratio = right_close_count / len(right_arc) if len(right_arc) > 0 else 0
            self.obstacle_sectors[2] = right_close_ratio > 0.3 or np.min(right_ranges) < self.safety_distance * 0.7
        
        if self.obstacle_detected and self.current_mode == 'autonomous':
            sectors_str = "Left: {}, Center: {}, Right: {}".format(
                "BLOCKED" if self.obstacle_sectors[0] else "clear",
                "BLOCKED" if self.obstacle_sectors[1] else "clear",
                "BLOCKED" if self.obstacle_sectors[2] else "clear"
            )
            self.get_logger().debug(f'Obstacle at {self.min_distance:.2f}m, {sectors_str}')
    
    def mode_switch_callback(self, msg):
        """Handle mode switch requests"""
        requested_mode = msg.data.lower()
        if requested_mode in ['manual', 'autonomous']:
            if requested_mode != self.current_mode:
                # Send stop command for safety during mode transition
                self.send_stop_command()
                self.current_mode = requested_mode
                self.get_logger().info(f'Switched to {self.current_mode} mode')
                
                # Publish current mode right away
                mode_msg = String()
                mode_msg.data = self.current_mode
                self.mode_pub.publish(mode_msg)
        else:
            self.get_logger().error(f'Invalid mode requested: {requested_mode}')

    def connect_motors(self):
        if self.verify_port_exists(self.r_wheel_port):
            try:
                self.motor_kanan = MotorControl(device=self.r_wheel_port)
                self.motor_kanan.set_drive_mode(1, 2)
            except Exception as e:
                print(f"Error motor kanan: {e}")
                self.motor_kanan = None

        if self.verify_port_exists(self.l_wheel_port):
            try:
                self.motor_kiri = MotorControl(device=self.l_wheel_port)
                self.motor_kiri.set_drive_mode(1, 2)
            except Exception as e:
                print(f"Error motor kiri: {e}")
                self.motor_kiri = None

    def safe_send_rpm(self, motor_obj, motor_id, speed):
        if not motor_obj:
            return False
        try:
            motor_obj.send_rpm(motor_id, speed)
            return True
        except Exception as e:
            print(f"Error kirim RPM: {e}")
            return False
    
    def shutdown_callback(self, msg):
        """Handle shutdown requests"""
        if msg.data == "shutdown":
            self.get_logger().warn('Shutdown command received, stopping motors')
            self.send_stop_command()
    
    def send_stop_command(self):
        """Send stop command to motors"""
        cmd = Twist()  # All fields initialize to 0
        # Send multiple times to ensure it's received
        for _ in range(3):
            self.cmd_vel_pub.publish(cmd)
    
    def control_loop(self):
        """Main control loop for autonomous behavior and status updates"""
        # Publish current mode regularly
        mode_msg = String()
        mode_msg.data = self.current_mode
        self.mode_pub.publish(mode_msg)
        
        # Autonomous behavior
        if self.current_mode == 'autonomous' and self.latest_scan is not None:
            self.autonomous_control()
    
    def autonomous_control(self):
        """Advanced autonomous control with obstacle avoidance"""
        cmd = Twist()
        
        # Check if we're stuck and need to execute escape maneuver
        if self.is_stuck():
            self.escape_maneuver()
            return
        
        # Emergency stop for very close obstacles
        if self.min_distance < self.danger_distance:
            self.get_logger().warn(f'Danger! Obstacle too close ({self.min_distance:.2f}m)')
            cmd.linear.x = 0.0
            # Turn away from obstacle - rotate more aggressively
            cmd.angular.z = self.max_angular_speed * 1.5 * (-1.0 if self.obstacle_direction > 0 else 1.0)
            self.cmd_vel_pub.publish(cmd)
            return
        
        # Check obstacle pattern for smart navigation decision
        if self.obstacle_detected:
            # Analyze which sectors are blocked
            left_blocked = self.obstacle_sectors[0]
            center_blocked = self.obstacle_sectors[1]
            right_blocked = self.obstacle_sectors[2]
            
            # Decision logic for finding the best escape route
            if left_blocked and center_blocked and right_blocked:
                # All directions blocked - rotate in place to find an exit
                self.get_logger().info('All directions blocked - rotating to find exit')
                cmd.linear.x = 0.0
                cmd.angular.z = self.max_angular_speed  # Consistently turn one direction to scan 360°

            
            elif center_blocked:
                # Center is blocked - determine best turn direction
                if left_blocked and not right_blocked:
                    # Left and center blocked, turn right
                    cmd.linear.x = 0.0  # Stop and turn
                    cmd.angular.z = -self.max_angular_speed  # Turn right sharply
                    self.current_speed_rwheel = -self.serong
                    self.current_speed_lwheel = self.speed
                    self.get_logger().info('Center and left blocked - turning right')
                
                elif right_blocked and not left_blocked:
                    # Right and center blocked, turn left
                    cmd.linear.x = 0.0  # Stop and turn
                    cmd.angular.z = self.max_angular_speed  # Turn left sharply
                    self.current_speed_rwheel = -self.speed
                    self.current_speed_lwheel = self.serong
                    self.get_logger().info('Center and right blocked - turning left')
                
                elif not left_blocked and not right_blocked:
                    # Only center blocked, turn based on previous obstacle position
                    turn_direction = -1.0 if self.obstacle_direction > 0 else 1.0
                    cmd.linear.x = 0.0  # Stop and turn
                    cmd.angular.z = self.max_angular_speed * turn_direction
                    if turn_direction < 0:
                        self.current_speed_rwheel = self.lambat
                        self.current_speed_lwheel = self.lambat
                    else:
                        self.current_speed_rwheel = -self.lambat
                        self.current_speed_lwheel = -self.lambat
                    self.get_logger().info(f'Center blocked - turning {("right" if turn_direction < 0 else "left")}')
                
                else:
                    # Shouldn't hit this condition, but just in case
                    cmd.linear.x = 0.0
                    cmd.angular.z = self.max_angular_speed  # Default turn
            
            elif left_blocked and not center_blocked:
                # Only left blocked, turn slightly right but keep moving
                cmd.linear.x = self.max_linear_speed * 0.7
                cmd.angular.z = -self.max_angular_speed * 0.5
                self.get_logger().info('Left blocked - veering right')
            
            elif right_blocked and not center_blocked:
                # Only right blocked, turn slightly left but keep moving
                cmd.linear.x = self.max_linear_speed * 0.7
                cmd.angular.z = self.max_angular_speed * 0.5
                self.get_logger().info('Right blocked - veering left')
            
            else:
                # Something detected but not in critical areas, slow down slightly
                cmd.linear.x = self.max_linear_speed * 0.8
                # Small correction away from closest obstacle
                cmd.angular.z = -0.3 * self.max_angular_speed * self.obstacle_direction
        else:
            # No obstacles, move forward at full speed
            cmd.linear.x = self.max_linear_speed
            cmd.angular.z = 0.0

        self.safe_send_rpm(self.motor_kanan, 1, self.current_speed_rwheel)
        self.safe_send_rpm(self.motor_kiri, 1, self.current_speed_lwheel)
        
        self.cmd_vel_pub.publish(cmd)

    def is_stuck(self):
        """Detect if robot is stuck in a situation for too long"""
        # Implementation - detect if minimum distance hasn't changed significantly
        # and obstacle pattern remains similar for several consecutive cycles
        if self.min_distance < self.safety_distance * 1.2:
            self.stuck_timer += 1
        else:
            self.stuck_timer = 0
        
        return self.stuck_timer > 20  # Stuck for more than ~2 seconds

    def escape_maneuver(self):
        """Execute special maneuver to escape from stuck situations"""
        cmd = Twist()
        
        # Try backing up and turning
        if self.stuck_timer % 40 < 10:  # First back up
            cmd.linear.x = -0.15
            cmd.angular.z = 0.0
            self.current_speed_rwheel = self.speed
            self.current_speed_lwheel = -self.speed
            self.get_logger().warn('Escape maneuver: backing up')
        elif self.stuck_timer % 40 < 30:  # Then turn significantly
            cmd.linear.x = 0.0
            cmd.angular.z = self.max_angular_speed * 1.5 * self.last_turn_direction

            if self.last_turn_direction > 0:
                self.current_speed_rwheel = -self.lambat
                self.current_speed_lwheel = -self.lambat
            else:
                self.current_speed_rwheel = self.lambat
                self.current_speed_lwheel = self.lambat
            self.get_logger().warn(f'Escape maneuver: turning {("left" if self.last_turn_direction > 0 else "right")}')
        else:  # Then try moving forward
            cmd.linear.x = 0.15
            cmd.angular.z = 0.0
            self.get_logger().warn('Escape maneuver: moving forward')
            
            # Flip turn direction for next time
            self.last_turn_direction *= -1.0
        
        self.safe_send_rpm(self.motor_kanan, 1, self.current_speed_rwheel)
        self.safe_send_rpm(self.motor_kiri, 1, self.current_speed_lwheel)
        
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = RobotModeController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
