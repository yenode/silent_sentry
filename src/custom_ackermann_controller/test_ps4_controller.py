#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import time

class PS4ControllerTester(Node):
    def __init__(self):
        super().__init__('ps4_controller_tester')


        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.last_joy_msg = None
        self.last_cmd_vel_msg = None
        
        self.timer = self.create_timer(1.0, self.print_status)
        
        self.get_logger().info('PS4 Controller Tester Started')
        self.get_logger().info('PS4 DualShock 4 Control Guide')
        self.get_logger().info('Left Stick: Forward/Backward + Turn Left/Right')
        self.get_logger().info('L1 Button: Hold to enable movement (SAFETY)')
        self.get_logger().info('R1 Button: Hold for turbo mode')
        self.get_logger().info('Press Ctrl+C to exit')
    
    def joy_callback(self, msg):
        # Processes raw joystick input
        self.last_joy_msg = msg
        
        # PS4 Controller mappings
        if len(msg.axes) >= 6 and len(msg.buttons) >= 13:
            left_stick_x = msg.axes[0]      # Left/Right (turn)
            left_stick_y = msg.axes[1]      # Forward/Backward  
            l1_button = msg.buttons[4]      # Enable button
            r1_button = msg.buttons[5]      # Turbo button
            
            if abs(left_stick_x) > 0.1 or abs(left_stick_y) > 0.1 or l1_button or r1_button:
                self.get_logger().info(
                    f'Joy Input : Stick X/Y: {left_stick_x:.2f}/{left_stick_y:.2f}, '
                    f'L1: {l1_button}, R1: {r1_button}'
                )
    
    def cmd_vel_callback(self, msg):
        self.last_cmd_vel_msg = msg
        
        if abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001:
            self.get_logger().info(
                f'Cmd_Vel Output - Linear: {msg.linear.x:.3f} m/s, '
                f'Angular: {msg.angular.z:.3f} rad/s'
            )
    
    def print_status(self):
        joy_status = "CONNECTED" if self.last_joy_msg else "NOT DETECTED"
        cmd_vel_status = "ACTIVE" if self.last_cmd_vel_msg else "NO OUTPUT"
        
        self.get_logger().info(f'Status : Joy: {joy_status}, Cmd_Vel: {cmd_vel_status}')
        
        if not self.last_joy_msg:
            self.get_logger().warn('PS4 Controller not detected! Check connection and relaunch')
        elif not self.last_cmd_vel_msg:
            self.get_logger().warn('No cmd_vel output! Hold L1 button and move left stick')

def main():
    rclpy.init()
    
    tester = PS4ControllerTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()