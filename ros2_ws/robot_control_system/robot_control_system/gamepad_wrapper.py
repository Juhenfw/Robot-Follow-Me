#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool

class GamepadWrapper(Node):
    def __init__(self):
        super().__init__('gamepad_wrapper')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'gamepad/cmd_vel', 10)
        self.mode_switch_pub = self.create_publisher(String, 'robot/mode_switch', 10)
        
        # Subscribers
        self.gamepad_sub = self.create_subscription(
            Twist, 'gamepad_robot_controller/cmd_vel', self.gamepad_callback, 10)
        
        # Assuming there might be a button for mode switching
        # Adjust the topic based on your gamepad implementation
        self.button_sub = self.create_subscription(
            Bool, 'gamepad_robot_controller/mode_button', self.mode_button_callback, 10)
        
        self.get_logger().info('Gamepad wrapper initialized')
    
    def gamepad_callback(self, msg):
        """Forward gamepad commands to the system"""
        self.cmd_vel_pub.publish(msg)
    
    def mode_button_callback(self, msg):
        """Handle mode switch button presses"""
        if msg.data:  # If button pressed
            mode_msg = String()
            # Toggle between modes - adjust as needed
            mode_msg.data = 'autonomous'  # You can implement proper toggling if needed
            self.mode_switch_pub.publish(mode_msg)
            self.get_logger().info('Mode switch button pressed')

def main():
    rclpy.init()
    node = GamepadWrapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
