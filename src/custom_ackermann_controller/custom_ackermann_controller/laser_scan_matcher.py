#!/usr/bin/env python3
"""
Custom Laser Scan Matcher Node for Odometry

This node estimates robot motion using laser scan matching without publishing TF transforms.
It only publishes odometry on /odom_laser topic for fusion with EKF.

Author: Aditya Pachauri
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import math
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3, PoseWithCovariance, TwistWithCovariance
from std_msgs.msg import Header


class LaserScanMatcher(Node):
    """
    Laser scan matcher for pure laser-based odometry estimation
    """
    
    def __init__(self):
        super().__init__('laser_scan_matcher')
        
        # Declare parameters
        self.declare_parameter('publish_odom_topic', '/odom_laser')
        self.declare_parameter('subscribe_scan_topic', '/scan')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('min_range', 0.1)
        self.declare_parameter('max_iterations', 50)
        self.declare_parameter('convergence_threshold', 0.001)
        self.declare_parameter('max_correspondence_distance', 0.3)
        self.declare_parameter('outlier_threshold', 0.5)
        
        # Get parameters
        self.odom_topic = self.get_parameter('publish_odom_topic').get_parameter_value().string_value
        self.scan_topic = self.get_parameter('subscribe_scan_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.min_range = self.get_parameter('min_range').get_parameter_value().double_value
        self.max_iterations = self.get_parameter('max_iterations').get_parameter_value().integer_value
        self.convergence_threshold = self.get_parameter('convergence_threshold').get_parameter_value().double_value
        self.max_correspondence_distance = self.get_parameter('max_correspondence_distance').get_parameter_value().double_value
        self.outlier_threshold = self.get_parameter('outlier_threshold').get_parameter_value().double_value
        
        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, qos_profile)
        
        # Publishers
        self.odom_pub = self.create_publisher(
            Odometry, self.odom_topic, qos_profile)
        
        # State variables
        self.previous_scan = None
        self.current_pose = np.array([0.0, 0.0, 0.0])  # [x, y, theta]
        self.current_velocity = np.array([0.0, 0.0, 0.0])  # [vx, vy, w]
        self.last_time = None
        
        # Statistics
        self.match_success_count = 0
        self.total_matches = 0
        
        self.get_logger().info(f"Laser Scan Matcher initialized")
        self.get_logger().info(f"Publishing odometry to: {self.odom_topic}")
        self.get_logger().info(f"Subscribing to laser scans: {self.scan_topic}")
    
    def scan_callback(self, scan_msg):
        """Process incoming laser scan and perform scan matching"""
        current_time = self.get_clock().now()
        
        # Convert scan to cartesian points
        points = self.scan_to_points(scan_msg)
        
        if points is None or len(points) < 10:
            self.get_logger().debug("Insufficient valid points in scan")
            return
        
        if self.previous_scan is not None and self.last_time is not None:
            # Perform scan matching
            dt = (current_time - self.last_time).nanoseconds / 1e9
            
            if dt > 0.001:  # Minimum time difference
                transform = self.match_scans(self.previous_scan, points, dt)
                
                if transform is not None:
                    # Update pose
                    self.update_pose(transform, dt)
                    self.match_success_count += 1
                    
                    # Publish odometry
                    self.publish_odometry(scan_msg.header.stamp)
                
                self.total_matches += 1
                
                # Log success rate periodically
                if self.total_matches % 100 == 0:
                    success_rate = self.match_success_count / self.total_matches * 100
                    self.get_logger().info(f"Scan match success rate: {success_rate:.1f}%")
        
        # Store for next iteration
        self.previous_scan = points
        self.last_time = current_time
    
    def scan_to_points(self, scan_msg):
        """Convert laser scan to cartesian points"""
        points = []
        
        for i, range_val in enumerate(scan_msg.ranges):
            # Skip invalid ranges
            if not (self.min_range <= range_val <= self.max_range):
                continue
            if not math.isfinite(range_val):
                continue
            
            # Calculate angle
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            
            # Convert to cartesian
            x = range_val * math.cos(angle)
            y = range_val * math.sin(angle)
            
            points.append([x, y])
        
        return np.array(points) if points else None
    
    def match_scans(self, prev_points, curr_points, dt):
        """
        Perform ICP-like scan matching to find transformation
        Returns [dx, dy, dtheta] or None if matching fails
        """
        if len(prev_points) < 10 or len(curr_points) < 10:
            return None
        
        # Initial guess based on previous velocity
        initial_guess = self.current_velocity * dt
        
        # Define optimization function
        def cost_function(transform):
            dx, dy, dtheta = transform
            
            # Transform current points
            cos_theta = math.cos(dtheta)
            sin_theta = math.sin(dtheta)
            
            transformed_points = []
            for point in curr_points:
                x, y = point
                new_x = cos_theta * x - sin_theta * y + dx
                new_y = sin_theta * x + cos_theta * y + dy
                transformed_points.append([new_x, new_y])
            
            transformed_points = np.array(transformed_points)
            
            # Find correspondences and calculate cost
            total_cost = 0.0
            correspondence_count = 0
            
            for t_point in transformed_points:
                # Find closest point in previous scan
                distances = np.linalg.norm(prev_points - t_point, axis=1)
                min_dist = np.min(distances)
                
                if min_dist < self.max_correspondence_distance:
                    total_cost += min_dist**2
                    correspondence_count += 1
            
            if correspondence_count < 5:  # Too few correspondences
                return float('inf')
            
            # Average cost with penalty for few correspondences
            avg_cost = total_cost / correspondence_count
            correspondence_penalty = max(0, (20 - correspondence_count) * 0.01)
            
            return avg_cost + correspondence_penalty
        
        # Optimization bounds
        bounds = [(-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5)]  # Reasonable motion limits
        
        try:
            # Perform optimization
            result = minimize(cost_function, initial_guess, 
                            method='L-BFGS-B', bounds=bounds,
                            options={'maxiter': self.max_iterations})
            
            if result.success and result.fun < self.outlier_threshold:
                return result.x
            else:
                return None
                
        except Exception as e:
            self.get_logger().debug(f"Optimization failed: {e}")
            return None
    
    def update_pose(self, transform, dt):
        """Update robot pose and velocity estimates"""
        dx, dy, dtheta = transform
        
        # Update velocity estimate (exponential smoothing)
        alpha = 0.7  # Smoothing factor
        new_velocity = np.array([dx/dt, dy/dt, dtheta/dt])
        self.current_velocity = alpha * new_velocity + (1-alpha) * self.current_velocity
        
        # Update pose
        cos_theta = math.cos(self.current_pose[2])
        sin_theta = math.sin(self.current_pose[2])
        
        # Transform to global frame
        global_dx = cos_theta * dx - sin_theta * dy
        global_dy = sin_theta * dx + cos_theta * dy
        
        self.current_pose[0] += global_dx
        self.current_pose[1] += global_dy
        self.current_pose[2] += dtheta
        
        # Normalize angle
        self.current_pose[2] = math.atan2(math.sin(self.current_pose[2]), 
                                         math.cos(self.current_pose[2]))
    
    def publish_odometry(self, timestamp):
        """Publish odometry message"""
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        
        # Pose
        odom_msg.pose.pose.position.x = self.current_pose[0]
        odom_msg.pose.pose.position.y = self.current_pose[1]
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation from yaw
        quat = R.from_euler('z', self.current_pose[2]).as_quat(canonical=True)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        
        # Velocity
        odom_msg.twist.twist.linear.x = self.current_velocity[0]
        odom_msg.twist.twist.linear.y = self.current_velocity[1]
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.current_velocity[2]
        
        # Covariance matrices (diagonal)
        pose_cov_diagonal = [0.1, 0.1, 0.0, 0.0, 0.0, 0.05]  # x, y, z, roll, pitch, yaw
        twist_cov_diagonal = [0.05, 0.05, 0.0, 0.0, 0.0, 0.03]  # vx, vy, vz, wx, wy, wz
        
        # Fill covariance matrices
        for i in range(6):
            odom_msg.pose.covariance[i*6 + i] = pose_cov_diagonal[i]
            odom_msg.twist.covariance[i*6 + i] = twist_cov_diagonal[i]
        
        self.odom_pub.publish(odom_msg)


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    node = LaserScanMatcher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Laser Scan Matcher shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()