#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',        # PS4 controller device
                'deadzone': 0.05,               # Small deadzone for PS4 controller
                'autorepeat_rate': 20.0,        # 20 Hz update rate
                'coalesce_interval': 0.01,      # Coalesce interval for smoother input
            }],
            output='screen'
        ),
        
        # Teleop twist joy (converts joy to twist) - PS4 DualShock 4 Configuration
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            parameters=[{
                # PS4 DualShock 4 Axis Mappings:
                # Axis 0: Left stick horizontal (left/right)  
                # Axis 1: Left stick vertical (up/down) - INVERTED
                # Axis 2: L2 trigger (0 to -1 when pressed)
                # Axis 3: Right stick horizontal (left/right)
                # Axis 4: Right stick vertical (up/down) - INVERTED  
                # Axis 5: R2 trigger (0 to -1 when pressed)
                
                'axis_linear.x': 1,             # Left stick vertical (forward/backward)
                'scale_linear.x': 1.0,          # Max linear speed (m/s)
                'scale_linear_turbo.x': 2.0,    # Turbo linear speed with turbo button
                
                'axis_angular.yaw': 0,          # Left stick horizontal (turn left/right)  
                'scale_angular.yaw': 2.0,       # Max angular speed (rad/s)
                'scale_angular_turbo.yaw': 4.0, # Turbo angular speed with turbo button
                
                # PS4 Button Mappings:
                # Button 0: X (Cross)
                # Button 1: Circle  
                # Button 2: Triangle
                # Button 3: Square
                # Button 4: L1 (Left shoulder)
                # Button 5: R1 (Right shoulder) 
                # Button 6: L2 (Left trigger button)
                # Button 7: R2 (Right trigger button)
                # Button 8: Share
                # Button 9: Options
                # Button 10: PlayStation button
                # Button 11: L3 (Left stick press)
                # Button 12: R3 (Right stick press)
                
                'enable_button': 4,             # L1 button to enable movement (safety)
                'enable_turbo_button': 5,       # R1 button for turbo mode
                'require_enable_button': True,   # Must hold L1 to move
                
                # Invert axes if needed (PS4 sticks may be inverted)
                'inverted_reverse': False,      # Set to True if reverse direction is wrong
                'inverted_turn': False,         # Set to True if turn direction is wrong
            }],
            remappings=[
                ('/cmd_vel', '/cmd_vel'),
            ],
            output='screen'
        ),
    ])