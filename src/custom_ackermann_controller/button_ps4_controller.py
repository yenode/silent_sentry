#!/usr/bin/env python3
"""
Simple PS4 Controller Button/Axis Mapper
This script identifies button numbers and axis directions for launch file configuration.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import sys

class PS4ButtonMapper(Node):
    def __init__(self):
        super().__init__('ps4_button_mapper')
        
        # Subscribe to joy topic
        self.joy_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # Store previous state to detect changes
        self.prev_buttons = []
        self.prev_axes = []
        
        print("\n" + "="*60)
        print("PS4 CONTROLLER BUTTON/AXIS MAPPER")
        print("="*60)
        print("Instructions:")
        print("1. Press L1 button and note the number")
        print("2. Press R1 button and note the number") 
        print("3. Move LEFT STICK:")
        print("   - Forward (away from you) = +ve linear")
        print("   - Backward (toward you) = -ve linear")
        print("   - Right = +ve angular")
        print("   - Left = -ve angular")
        print("4. Press Ctrl+C when done")
        print("="*60)
        print("Waiting for input...\n")

    def joy_callback(self, msg):
        # Simple button detection - show button number when pressed
        if len(self.prev_buttons) == len(msg.buttons):
            for i, (current, previous) in enumerate(zip(msg.buttons, self.prev_buttons)):
                if current == 1 and previous == 0:  # Button pressed
                    print(f"🎮 BUTTON {i} PRESSED")
        
        # Simple axis detection - show axis number and value for significant movements
        if len(self.prev_axes) == len(msg.axes):
            for i, (current, previous) in enumerate(zip(msg.axes, self.prev_axes)):
                if abs(current - previous) > 0.3:  # Significant movement
                    direction = "POSITIVE" if current > 0 else "NEGATIVE"
                    print(f"🕹️  AXIS {i}: {direction} ({current:.3f})")
        
        # Store current state for next comparison
        self.prev_buttons = list(msg.buttons)
        self.prev_axes = list(msg.axes)

def main():
    rclpy.init()
    
    try:
        button_mapper = PS4ButtonMapper()
        rclpy.spin(button_mapper)
    except KeyboardInterrupt:
        print("\n" + "="*60)
        print("CONFIGURATION SUMMARY")
        print("="*60)
        print("Use these values in your launch file:")
        print("")
        print("Based on your testing above:")
        print("1. Find L1 button number → use for 'enable_button'")
        print("2. Find R1 button number → use for 'enable_turbo_button'")
        print("3. Find left stick axis for forward/back → use for 'axis_linear.x'")
        print("4. Find left stick axis for left/right → use for 'axis_angular.yaw'")
        print("")
        print("Launch file should look like:")
        print("'enable_button': [L1_BUTTON_NUMBER],")
        print("'enable_turbo_button': [R1_BUTTON_NUMBER],")
        print("'axis_linear.x': [FORWARD_BACK_AXIS],")
        print("'axis_angular.yaw': [LEFT_RIGHT_AXIS],")
        print("="*60)
    finally:
        if rclpy.ok():
            button_mapper.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()