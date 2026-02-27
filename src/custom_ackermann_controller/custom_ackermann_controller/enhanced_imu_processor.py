#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
from sensor_msgs.msg import Imu, Temperature
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from collections import deque
import time

class EnhancedIMUProcessorNode(Node):
    def __init__(self):
        super().__init__('enhanced_imu_processor')
        
        # Parameters
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('temperature_topic', '/imu/temperature')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('imu_frame', 'imu_link')
        
        # Bias estimation parameters
        self.declare_parameter('bias_estimation_samples', 1000)
        self.declare_parameter('bias_update_rate', 0.1)
        self.declare_parameter('temperature_compensation', True)
        self.declare_parameter('static_bias_gyro_x', 0.0)
        self.declare_parameter('static_bias_gyro_y', 0.0)
        self.declare_parameter('static_bias_gyro_z', 0.0)
        self.declare_parameter('static_bias_accel_x', 0.0)
        self.declare_parameter('static_bias_accel_y', 0.0)
        self.declare_parameter('static_bias_accel_z', 0.0)
        
        # Noise characterization parameters
        self.declare_parameter('gyro_noise_density', 0.01)  # rad/s/sqrt(Hz)
        self.declare_parameter('gyro_random_walk', 0.001)   # rad/s^2/sqrt(Hz)
        self.declare_parameter('accel_noise_density', 0.1)  # m/s^2/sqrt(Hz)
        self.declare_parameter('accel_random_walk', 0.01)   # m/s^3/sqrt(Hz)
        
        # Get parameters
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.temperature_topic = self.get_parameter('temperature_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.imu_frame = self.get_parameter('imu_frame').get_parameter_value().string_value
        
        self.bias_estimation_samples = self.get_parameter('bias_estimation_samples').get_parameter_value().integer_value
        self.bias_update_rate = self.get_parameter('bias_update_rate').get_parameter_value().double_value
        self.temperature_compensation = self.get_parameter('temperature_compensation').get_parameter_value().bool_value
        
        # Static biases
        self.static_bias_gyro = np.array([
            self.get_parameter('static_bias_gyro_x').get_parameter_value().double_value,
            self.get_parameter('static_bias_gyro_y').get_parameter_value().double_value,
            self.get_parameter('static_bias_gyro_z').get_parameter_value().double_value
        ])
        self.static_bias_accel = np.array([
            self.get_parameter('static_bias_accel_x').get_parameter_value().double_value,
            self.get_parameter('static_bias_accel_y').get_parameter_value().double_value,
            self.get_parameter('static_bias_accel_z').get_parameter_value().double_value
        ])
        
        # Noise parameters
        self.gyro_noise_density = self.get_parameter('gyro_noise_density').get_parameter_value().double_value
        self.gyro_random_walk = self.get_parameter('gyro_random_walk').get_parameter_value().double_value
        self.accel_noise_density = self.get_parameter('accel_noise_density').get_parameter_value().double_value
        self.accel_random_walk = self.get_parameter('accel_random_walk').get_parameter_value().double_value
        
        # State variables
        self.bias_gyro = self.static_bias_gyro.copy()
        self.bias_accel = self.static_bias_accel.copy()
        self.temperature = 25.0  # Default temperature
        self.last_temperature = 25.0
        
        # Bias estimation
        self.gyro_samples = deque(maxlen=self.bias_estimation_samples)
        self.accel_samples = deque(maxlen=self.bias_estimation_samples)
        self.is_stationary = False
        self.stationary_threshold_gyro = 0.02  # rad/s
        self.stationary_threshold_accel = 0.5  # m/s^2 (accounting for gravity)
        self.stationary_duration_threshold = 2.0  # seconds
        self.stationary_start_time = None
        
        # Temperature compensation coefficients (example values)
        self.temp_coeff_gyro = np.array([0.001, 0.001, 0.001])  # bias change per degree C
        self.temp_coeff_accel = np.array([0.01, 0.01, 0.01])
        self.reference_temperature = 25.0
        
        # Allan variance estimation
        self.allan_samples_gyro = deque(maxlen=10000)
        self.allan_samples_accel = deque(maxlen=10000)
        
        # Publishers
        self.imu_corrected_pub = self.create_publisher(Imu, '/imu/data_corrected', 10)
        self.bias_pub = self.create_publisher(Float64MultiArray, '/imu/bias_estimate', 10)
        self.noise_pub = self.create_publisher(Float64MultiArray, '/imu/noise_estimate', 10)
        self.calibration_status_pub = self.create_publisher(Float64MultiArray, '/imu/calibration_status', 10)
        
        # Subscribers
        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        if self.temperature_compensation:
            self.temp_sub = self.create_subscription(Temperature, self.temperature_topic, self.temperature_callback, 10)
        
        # Timers
        self.bias_update_timer = self.create_timer(1.0 / self.bias_update_rate, self.update_bias_estimate)
        self.noise_estimation_timer = self.create_timer(10.0, self.estimate_noise_parameters)
        
        self.get_logger().info('Enhanced IMU Processor Node initialized')

    def temperature_callback(self, temp_msg):
        """Handle temperature updates for compensation"""
        self.temperature = temp_msg.temperature

    def is_robot_stationary(self, gyro_data, accel_data):
        """Determine if robot is stationary for bias estimation"""
        # Check gyroscope
        gyro_magnitude = np.linalg.norm(gyro_data)
        
        # Check accelerometer (subtract gravity)
        gravity_magnitude = 9.81
        accel_magnitude = np.linalg.norm(accel_data)
        accel_deviation = abs(accel_magnitude - gravity_magnitude)
        
        # Criteria for stationary state
        gyro_stationary = gyro_magnitude < self.stationary_threshold_gyro
        accel_stationary = accel_deviation < self.stationary_threshold_accel
        
        return gyro_stationary and accel_stationary

    def apply_temperature_compensation(self, bias, temp_coeff, current_temp):
        """Apply temperature compensation to bias"""
        if not self.temperature_compensation:
            return bias
        
        temp_delta = current_temp - self.reference_temperature
        temp_correction = temp_coeff * temp_delta
        
        return bias + temp_correction

    def calculate_dynamic_covariance(self, sample_rate):
        """Calculate dynamic covariance based on noise characteristics"""
        dt = 1.0 / sample_rate if sample_rate > 0 else 0.01
        
        # Angular velocity covariance (gyroscope)
        gyro_var = (self.gyro_noise_density ** 2) / dt
        gyro_cov = np.diag([gyro_var] * 3)
        
        # Linear acceleration covariance (accelerometer)
        accel_var = (self.accel_noise_density ** 2) / dt
        accel_cov = np.diag([accel_var] * 3)
        
        # Random walk contribution
        gyro_rw_var = (self.gyro_random_walk ** 2) * dt
        accel_rw_var = (self.accel_random_walk ** 2) * dt
        
        # Add temperature-dependent noise if compensation is enabled
        if self.temperature_compensation:
            temp_factor = 1.0 + 0.01 * abs(self.temperature - self.reference_temperature)
            gyro_cov *= temp_factor
            accel_cov *= temp_factor
        
        return gyro_cov, accel_cov

    def imu_callback(self, imu_msg):
        """Process incoming IMU data"""
        # Extract data
        gyro_raw = np.array([
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z
        ])
        
        accel_raw = np.array([
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z
        ])
        
        # Check if robot is stationary
        stationary = self.is_robot_stationary(gyro_raw, accel_raw)
        
        current_time = time.time()
        if stationary:
            if not self.is_stationary:
                self.stationary_start_time = current_time
                self.is_stationary = True
            elif self.stationary_start_time is not None and (current_time - self.stationary_start_time) > self.stationary_duration_threshold:
                # Collect samples for bias estimation
                self.gyro_samples.append(gyro_raw.copy())
                self.accel_samples.append(accel_raw.copy())
        else:
            self.is_stationary = False
            self.stationary_start_time = None
        
        # Collect samples for Allan variance (always)
        self.allan_samples_gyro.append(gyro_raw.copy())
        self.allan_samples_accel.append(accel_raw.copy())
        
        # Apply temperature compensation to bias
        bias_gyro_compensated = self.apply_temperature_compensation(
            self.bias_gyro, self.temp_coeff_gyro, self.temperature
        )
        bias_accel_compensated = self.apply_temperature_compensation(
            self.bias_accel, self.temp_coeff_accel, self.temperature
        )
        
        # Apply bias correction
        gyro_corrected = gyro_raw - bias_gyro_compensated
        accel_corrected = accel_raw - bias_accel_compensated
        
        # Calculate sample rate for covariance
        sample_rate = 100.0  # Assume 100 Hz, could be calculated from timestamps
        gyro_cov, accel_cov = self.calculate_dynamic_covariance(sample_rate)
        
        # Create corrected IMU message
        corrected_msg = Imu()
        corrected_msg.header = imu_msg.header
        corrected_msg.header.frame_id = self.imu_frame
        
        # Set corrected angular velocity
        corrected_msg.angular_velocity.x = gyro_corrected[0]
        corrected_msg.angular_velocity.y = gyro_corrected[1]
        corrected_msg.angular_velocity.z = gyro_corrected[2]
        
        # Set corrected linear acceleration
        corrected_msg.linear_acceleration.x = accel_corrected[0]
        corrected_msg.linear_acceleration.y = accel_corrected[1]
        corrected_msg.linear_acceleration.z = accel_corrected[2]
        
        # Keep original orientation (if available)
        corrected_msg.orientation = imu_msg.orientation
        
        # Set covariance matrices
        # Angular velocity covariance
        for i in range(3):
            for j in range(3):
                corrected_msg.angular_velocity_covariance[i*3 + j] = gyro_cov[i, j]
        
        # Linear acceleration covariance
        for i in range(3):
            for j in range(3):
                corrected_msg.linear_acceleration_covariance[i*3 + j] = accel_cov[i, j]
        
        # Orientation covariance (use original or set based on integration uncertainty)
        if hasattr(imu_msg, 'orientation_covariance') and any(imu_msg.orientation_covariance):
            corrected_msg.orientation_covariance = imu_msg.orientation_covariance
        else:
            # Set reasonable orientation uncertainty
            orientation_var = 0.01  # rad^2
            corrected_msg.orientation_covariance = [
                orientation_var, 0.0, 0.0,
                0.0, orientation_var, 0.0,
                0.0, 0.0, orientation_var
            ]
        
        # Publish corrected IMU data
        self.imu_corrected_pub.publish(corrected_msg)

    def update_bias_estimate(self):
        """Update bias estimates from collected stationary samples"""
        if len(self.gyro_samples) >= 50:  # Minimum samples for reliable estimate
            gyro_array = np.array(list(self.gyro_samples))
            new_bias_gyro = np.mean(gyro_array, axis=0)
            
            # Exponential moving average for smooth updates
            alpha = 0.1
            self.bias_gyro = (1 - alpha) * self.bias_gyro + alpha * new_bias_gyro
        
        if len(self.accel_samples) >= 50:
            accel_array = np.array(list(self.accel_samples))
            # For accelerometer, subtract gravity from z-axis
            accel_array[:, 2] -= 9.81  # Assuming z-up orientation
            new_bias_accel = np.mean(accel_array, axis=0)
            
            alpha = 0.1
            self.bias_accel = (1 - alpha) * self.bias_accel + alpha * new_bias_accel
        
        # Publish bias estimates
        bias_msg = Float64MultiArray()
        bias_msg.data = np.concatenate([self.bias_gyro, self.bias_accel]).tolist()
        self.bias_pub.publish(bias_msg)
        
        # Publish calibration status
        status_msg = Float64MultiArray()
        gyro_sample_ratio = len(self.gyro_samples) / max(self.bias_estimation_samples, 1)
        accel_sample_ratio = len(self.accel_samples) / max(self.bias_estimation_samples, 1)
        temp_stability = 1.0 / (1.0 + abs(self.temperature - self.last_temperature))
        
        status_msg.data = [
            gyro_sample_ratio,
            accel_sample_ratio,
            temp_stability,
            float(self.is_stationary)
        ]
        self.calibration_status_pub.publish(status_msg)
        
        self.last_temperature = self.temperature

    def estimate_noise_parameters(self):
        """Estimate noise parameters using Allan variance"""
        if len(self.allan_samples_gyro) < 1000:
            return
        
        # Simple noise estimation (could be improved with Allan variance analysis)
        gyro_array = np.array(list(self.allan_samples_gyro))
        accel_array = np.array(list(self.allan_samples_accel))
        
        # Calculate standard deviations
        gyro_std = np.std(gyro_array, axis=0)
        accel_std = np.std(accel_array, axis=0)
        
        # Estimate noise densities (simplified)
        estimated_gyro_noise = np.mean(gyro_std)
        estimated_accel_noise = np.mean(accel_std)
        
        # Publish noise estimates
        noise_msg = Float64MultiArray()
        noise_msg.data = [
            estimated_gyro_noise,
            estimated_accel_noise,
            self.gyro_noise_density,
            self.accel_noise_density
        ]
        self.noise_pub.publish(noise_msg)
        
        self.get_logger().debug(
            f'Estimated noise - Gyro: {estimated_gyro_noise:.4f}, Accel: {estimated_accel_noise:.4f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = EnhancedIMUProcessorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()