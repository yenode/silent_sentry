#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf_transformations import quaternion_from_euler
import numpy as np

class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry_node')

        self.declare_parameter('wheelbase', 0.9)  # Distance between front and rear axles
        self.declare_parameter('wheel_radius', 0.175) 
        self.declare_parameter('track_width', 0.67)  # Distance between left and right wheels 
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('left_wheel_joint', 'base_back_left_wheel_joint')
        self.declare_parameter('right_wheel_joint', 'base_back_right_wheel_joint')
        self.declare_parameter('left_steering_joint', 'base_front_left_steering_joint')
        self.declare_parameter('right_steering_joint', 'base_front_right_steering_joint')
        self.declare_parameter('publish_rate', 50.0)

        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.track_width = self.get_parameter('track_width').get_parameter_value().double_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.left_wheel_joint = self.get_parameter('left_wheel_joint').get_parameter_value().string_value
        self.right_wheel_joint = self.get_parameter('right_wheel_joint').get_parameter_value().string_value
        self.left_steering_joint = self.get_parameter('left_steering_joint').get_parameter_value().string_value
        self.right_steering_joint = self.get_parameter('right_steering_joint').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = self.get_clock().now()
        self.last_wheel_pos = {'left': 0.0, 'right': 0.0}
        self.last_wheel_vel = {'left': 0.0, 'right': 0.0}
        self.joint_names = []

        
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_odometry)

        self.current_steering_angle = 0.0
        self.current_wheel_vel = {'left': 0.0, 'right': 0.0}
        self.initialized = False

        # Various Filtering Parameters
        self.velocity_filter_alpha = 0.3  # Low Pass filter coefficient
        self.filtered_wheel_vel = {'left': 0.0, 'right': 0.0}
        self.max_wheel_acceleration = 5.0  # Maximum acceleration of the robot (in m/s^2)
        
        # Slip Detection Parameters
        self.expected_vel_diff_threshold = 0.1  # Maximum velocity difference between wheels (in m/s)
        self.slip_detection_enabled = True

        self.get_logger().info('Wheel Odometry Node Started - waiting for joint states...')

    def joint_state_callback(self, msg: JointState):
        # Use simulation time from the node's clock (respects use_sim_time parameter)
        current_time = self.get_clock().now()
        
        # Initializing on first callback
        if not self.initialized:
            self.joint_names = list(msg.name)
            self.get_logger().info(f"Available joints: {self.joint_names}")
            
            # Checking if required joints are being published
            required_joints = [self.left_wheel_joint, self.right_wheel_joint, 
                             self.left_steering_joint, self.right_steering_joint]
            missing_joints = [joint for joint in required_joints if joint not in self.joint_names]
            
            if missing_joints:
                self.get_logger().warn(f"Missing joints: {missing_joints}")
                return
            
            # Initializing wheel positions
            try:
                left_wheel_pos = msg.position[self.joint_names.index(self.left_wheel_joint)]
                right_wheel_pos = msg.position[self.joint_names.index(self.right_wheel_joint)]
                self.last_wheel_pos['left'] = left_wheel_pos
                self.last_wheel_pos['right'] = right_wheel_pos
                self.last_time = current_time
                self.initialized = True
                self.get_logger().info("Wheel odometry initialized with simulation time!")
            except (ValueError, IndexError) as e:
                self.get_logger().error(f"Failed to initialize: {e}")
                return

        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.joint_names = list(msg.name)

        try:
            left_wheel_pos = msg.position[self.joint_names.index(self.left_wheel_joint)]
            right_wheel_pos = msg.position[self.joint_names.index(self.right_wheel_joint)]
            left_steer_pos = msg.position[self.joint_names.index(self.left_steering_joint)]
            right_steer_pos = msg.position[self.joint_names.index(self.right_steering_joint)]

            # Calculating velocity for odometry calculation only if enough time have passed to avoid noise
            if dt > 0.001: 
                # Wheel angular velocities
                left_wheel_ang_vel = (left_wheel_pos - self.last_wheel_pos['left']) / dt
                right_wheel_ang_vel = (right_wheel_pos - self.last_wheel_pos['right']) / dt
                
                # Calculating raw linear velocities of the wheels
                raw_left_vel = left_wheel_ang_vel * self.wheel_radius
                raw_right_vel = right_wheel_ang_vel * self.wheel_radius

                # Appling acceleration limitng to prevent unrealisitic jumps in odometry
                max_vel_change = self.max_wheel_acceleration * dt
                left_vel_change = raw_left_vel - self.last_wheel_vel['left']
                right_vel_change = raw_right_vel - self.last_wheel_vel['right']
                    
                # Limiting acceleration
                if abs(left_vel_change) > max_vel_change:
                    raw_left_vel = self.last_wheel_vel['left'] + math.copysign(max_vel_change, left_vel_change)
                if abs(right_vel_change) > max_vel_change:
                    raw_right_vel = self.last_wheel_vel['right'] + math.copysign(max_vel_change, right_vel_change)

                # Applying Low Pass filter to filter out high frequency data
                self.filtered_wheel_vel['left'] = (1 - self.velocity_filter_alpha) * self.filtered_wheel_vel['left'] + self.velocity_filter_alpha * raw_left_vel
                self.filtered_wheel_vel['right'] = (1 - self.velocity_filter_alpha) * self.filtered_wheel_vel['right'] + self.velocity_filter_alpha * raw_right_vel

                # Applying velocity deadzone to handle encoder noise
                if abs(self.filtered_wheel_vel['left']) < 0.005:  # 5mm/s deadzone
                    self.filtered_wheel_vel['left'] = 0.0
                if abs(self.filtered_wheel_vel['right']) < 0.005:
                    self.filtered_wheel_vel['right'] = 0.0

                # Updating with filtered velocities
                self.current_wheel_vel['left'] = self.filtered_wheel_vel['left']
                self.current_wheel_vel['right'] = self.filtered_wheel_vel['right']
                
                self.last_wheel_vel = {'left': raw_left_vel, 'right': raw_right_vel}

                # Calculating robot steering angle from steer wheel positions
                self.current_steering_angle = self.calculate_robot_steering_angle(left_steer_pos, right_steer_pos)

                self.last_wheel_pos['left'] = left_wheel_pos
                self.last_wheel_pos['right'] = right_wheel_pos
                self.last_time = current_time
            else:
                # If dt is too small, ignoring it and keeping the current state
                pass

        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Joints not found in /joint_states topic: {e}')
            self.get_logger().info(f'Available joints: {self.joint_names}')
            self.get_logger().info(f'Looking for: {self.left_wheel_joint}, {self.right_wheel_joint}, {self.left_steering_joint}, {self.right_steering_joint}')

    def calculate_robot_steering_angle(self, left_steer_angle, right_steer_angle):
        
        if abs(left_steer_angle) < 0.001 and abs(right_steer_angle) < 0.001:
            return 0.0
        
        # Calculate turning radius to ICR for each wheel
        # For Ackermann geometry: R_wheel = wheelbase / tan(steer_angle)
        try:
            # Calculate the radius from each wheel to the instantaneous center of rotation
            left_wheel_radius = self.wheelbase / math.tan(abs(left_steer_angle))
            right_wheel_radius = self.wheelbase / math.tan(abs(right_steer_angle))
     
            # Determining Turn Direction (Positive for Left turn , Negative for Right turn)
            turn_direction = 0.0
            if left_steer_angle > 0.001 or right_steer_angle > 0.001:
                turn_direction = 1.0  # Left turn
            elif left_steer_angle < -0.001 or right_steer_angle < -0.001:
                turn_direction = -1.0  # Right turn
            
            # Robot's effective turning radius
            if turn_direction > 0: 
                # Left Turn , left wheel is inner wheel with smaller radius
                inner_radius = min(left_wheel_radius, right_wheel_radius)
                robot_center_radius = inner_radius + (self.track_width / 2.0)
            else: 
                # Right Turn , right wheel is inner wheel with smaller radius
                inner_radius = min(left_wheel_radius, right_wheel_radius)
                robot_center_radius = inner_radius + (self.track_width / 2.0)
            
            # Calculating effective Robot Steering angle using Bicycle Kinematic Model
            # δ = atan(wheelbase / radius)
            if robot_center_radius > 0.001:
                robot_steering_angle = math.atan(self.wheelbase / robot_center_radius)
                return robot_steering_angle * turn_direction
            else:
                return 0.0
                
        except (ZeroDivisionError, ValueError):
            # If Bicycle Kinematic Model fails , then going for average method
            self.get_logger().warn("Steering angle calculation failed, using average")
            return (left_steer_angle + right_steer_angle) / 2.0

    def publish_odometry(self):
        # Not publishing odometry until we have initialized the joint states
        if not self.initialized:
            return
            
        # Use simulation time from the node's clock
        current_time = self.get_clock().now()
        
        # Calculate time since last odometry update
        if not hasattr(self, 'last_odom_time'):
            self.last_odom_time = current_time
            return
            
        dt = (current_time - self.last_odom_time).nanoseconds / 1e9
        
        # Updating pose only if the time step is ideal
        if dt > 0.001 and dt < 0.1:  # Between 1ms and 100ms
            # Average linear velocity of the robot 
            linear_velocity = (self.current_wheel_vel['left'] + self.current_wheel_vel['right']) / 2.0
            
            # Angular velocity of the robot (bicycle model)
            # omega = v * tan(delta) / L
            if abs(self.wheelbase) > 0.001 and abs(self.current_steering_angle) > 0.001:
                angular_velocity = linear_velocity * math.tan(self.current_steering_angle) / self.wheelbase
            else:
                angular_velocity = 0.0

            # Integrating for Pose (X-axis forward robot - ROS standard)
            # For X-forward robot: forward motion increases X, turning changes Y
            delta_x = linear_velocity * math.cos(self.theta) * dt
            delta_y = linear_velocity * math.sin(self.theta) * dt
            delta_theta = angular_velocity * dt

            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta

            # Normalizing robot's yaw
            while self.theta > math.pi:
                self.theta -= 2 * math.pi
            while self.theta < -math.pi:
                self.theta += 2 * math.pi
        else:
            # Using current velocities if time step is not ideal, thus preventing pose from getting distorted
            linear_velocity = (self.current_wheel_vel['left'] + self.current_wheel_vel['right']) / 2.0
            if abs(self.wheelbase) > 0.001 and abs(self.current_steering_angle) > 0.001:
                angular_velocity = linear_velocity * math.tan(self.current_steering_angle) / self.wheelbase
            else:
                angular_velocity = 0.0
        
        self.last_odom_time = current_time

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # Pose
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # Twist (for X-axis forward robot - ROS standard)
        odom_msg.twist.twist.linear.x = linear_velocity  # Forward velocity in X-axis
        odom_msg.twist.twist.linear.y = 0.0  # No sideways motion
        odom_msg.twist.twist.angular.z = angular_velocity
        
        # Lower covariance values means more accurate and vice versa
        speed = abs(linear_velocity)
        angular_speed = abs(angular_velocity)
        
        # Position covariance increases with speed and steering angle
        pos_var = 0.01 + 0.05 * speed + 0.1 * abs(self.current_steering_angle)
        # Orientation covariance increases with angular velocity and steering
        orient_var = 0.01 + 0.1 * angular_speed + 0.2 * abs(self.current_steering_angle)
        # Velocity covariance is relatively lower because linear velocity is accurate from encoders
        vel_var = 0.02 + 0.05 * speed
        
        # Pose covariance: [x, y, z, roll, pitch, yaw]
        odom_msg.pose.covariance = [pos_var, 0.0, 0.0, 0.0, 0.0, 0.0,          # x
                                    0.0, pos_var, 0.0, 0.0, 0.0, 0.0,          # y  
                                    0.0, 0.0, 1e6, 0.0, 0.0, 0.0,              # z (not used)
                                    0.0, 0.0, 0.0, 1e6, 0.0, 0.0,              # roll (not used)
                                    0.0, 0.0, 0.0, 0.0, 1e6, 0.0,              # pitch (not used)
                                    0.0, 0.0, 0.0, 0.0, 0.0, orient_var]       # yaw
        
        # Twist covariance: [vx, vy, vz, roll_rate, pitch_rate, yaw_rate] (X-axis forward)                            
        odom_msg.twist.covariance = [vel_var, 0.0, 0.0, 0.0, 0.0, 0.0,         # vx (accurate - forward motion)
                                     0.0, 1e6, 0.0, 0.0, 0.0, 0.0,             # vy (not used - no lateral motion)
                                     0.0, 0.0, 1e6, 0.0, 0.0, 0.0,             # vz (not used)
                                     0.0, 0.0, 0.0, 1e6, 0.0, 0.0,             # roll_rate (not used)
                                     0.0, 0.0, 0.0, 0.0, 1e6, 0.0,             # pitch_rate (not used)
                                     0.0, 0.0, 0.0, 0.0, 0.0, vel_var * 2]     # yaw_rate (from kinematics)

        self.odom_pub.publish(odom_msg)
        
        # Publishing every 50 messages to avoid spam
        if hasattr(self, 'debug_counter'):
            self.debug_counter += 1
        else:
            self.debug_counter = 0
            
        # Debugging the input data at a a low frequency
        if (not self.debug_counter % 50):
            self.get_logger().info(f'Raw Wheel Odom: x={self.x:.2f}, y={self.y:.2f}, θ={math.degrees(self.theta):.1f}°, v={linear_velocity:.2f}, ω={math.degrees(angular_velocity):.1f}°/s')
            
def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode() 
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
