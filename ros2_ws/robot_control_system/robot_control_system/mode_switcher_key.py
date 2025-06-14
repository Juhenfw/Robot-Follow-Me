#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty
import select

class ModeSwitcherKeyboard(Node):
    def __init__(self):
        super().__init__('mode_switcher_keyboard')
        
        # Publisher untuk mengubah mode
        self.mode_pub = self.create_publisher(String, '/robot/mode_switch', 10)
        
        # Subscriber untuk melihat mode saat ini
        self.mode_sub = self.create_subscription(
            String, '/robot/current_mode', self.mode_callback, 10)
        
        # State variables
        self.current_mode = "unknown"
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Tampilkan instruksi
        self.print_instructions()
        
        # Main loop untuk membaca keyboard
        self.key_loop()
    
    def mode_callback(self, msg):
        """Keep track of current mode"""
        if self.current_mode != msg.data:
            self.current_mode = msg.data
            self.get_logger().info(f'Current mode: {self.current_mode.upper()}')
    
    def print_instructions(self):
        """Display instructions"""
        self.get_logger().info('')
        self.get_logger().info('==== ROBOT MODE SWITCHER ====')
        self.get_logger().info('m - Switch to MANUAL mode')
        self.get_logger().info('a - Switch to AUTONOMOUS mode')
        self.get_logger().info('q - Quit')
        self.get_logger().info('==========================')
        self.get_logger().info(f'Current mode: {self.current_mode.upper()}')
        self.get_logger().info('')
    
    def get_key(self):
        """Get a keystroke"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def key_loop(self):
        """Main loop for keypress handling"""
        try:
            while rclpy.ok():
                # Process callbacks
                rclpy.spin_once(self, timeout_sec=0.1)
                
                # Check for keypress
                if select.select([sys.stdin], [], [], 0.0)[0]:
                    key = self.get_key().lower()
                    
                    # Mode switching
                    if key == 'm':
                        self.publish_mode_change('manual')
                    elif key == 'a':
                        self.publish_mode_change('autonomous')
                    elif key == 'q':
                        self.get_logger().info('Exiting...')
                        break
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    
    def publish_mode_change(self, mode):
        """Publish mode change request"""
        if mode == self.current_mode:
            self.get_logger().info(f'Already in {mode.upper()} mode')
            return
            
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        self.get_logger().info(f'Requesting mode change to: {mode.upper()}')

def main(args=None):
    rclpy.init(args=args)
    node = ModeSwitcherKeyboard()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
