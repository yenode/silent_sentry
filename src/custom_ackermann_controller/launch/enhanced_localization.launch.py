import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Package path
    pkg_share = FindPackageShare('custom_ackermann_controller')
    
    # Configuration file paths
    enhanced_wheel_config = PathJoinSubstitution([
        pkg_share, 'config', 'enhanced_wheel_odometry.yaml'
    ])
    
    adaptive_scan_config = PathJoinSubstitution([
        pkg_share, 'config', 'adaptive_scan_matcher.yaml'
    ])
    
    enhanced_imu_config = PathJoinSubstitution([
        pkg_share, 'config', 'enhanced_imu_processor.yaml'
    ])
    
    enhanced_ekf_config = PathJoinSubstitution([
        pkg_share, 'config', 'enhanced_ekf.yaml'
    ])
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    enable_scan_matching_arg = DeclareLaunchArgument(
        'enable_scan_matching',
        default_value='true',
        description='Enable adaptive scan matching'
    )
    
    enable_imu_processing_arg = DeclareLaunchArgument(
        'enable_imu_processing',
        default_value='true',
        description='Enable enhanced IMU processing'
    )
    
    
    
    
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Laser scan topic'
    )
    
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu',
        description='IMU data topic'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_footprint',
        description='Robot base frame'
    )
    
    # Enhanced wheel odometry node
    enhanced_wheel_odometry_node = Node(
        package='custom_ackermann_controller',
        executable='enhanced_wheel_odometry',
        name='enhanced_wheel_odometry',
        parameters=[
            enhanced_wheel_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'base_frame': LaunchConfiguration('base_frame'),
            }
        ],
        output='screen',
        remappings=[
            ('/wheel_states', '/wheel_states'),
            ('/cmd_vel', '/cmd_vel'),
        ]
    )
    
    # Adaptive scan matcher node (conditional)
    adaptive_scan_matcher_node = Node(
        package='custom_ackermann_controller',
        executable='adaptive_scan_matcher',
        name='adaptive_scan_matcher',
        parameters=[
            adaptive_scan_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'scan_topic': LaunchConfiguration('scan_topic'),
                'base_frame': LaunchConfiguration('base_frame'),
            }
        ],
        output='screen',
        condition=launch.conditions.IfCondition(LaunchConfiguration('enable_scan_matching')),
        remappings=[
            ('/scan', LaunchConfiguration('scan_topic')),
        ]
    )
    
    # Enhanced IMU processor node (conditional)
    enhanced_imu_processor_node = Node(
        package='custom_ackermann_controller',
        executable='enhanced_imu_processor',
        name='enhanced_imu_processor',
        parameters=[
            enhanced_imu_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'base_frame': LaunchConfiguration('base_frame'),
            }
        ],
        output='screen',
        condition=launch.conditions.IfCondition(LaunchConfiguration('enable_imu_processing')),
        remappings=[
            ('/imu/data', LaunchConfiguration('imu_topic')),
        ]
    )
    
    # Robot localization EKF node
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            enhanced_ekf_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        remappings=[
            ('/odometry/filtered', '/odometry/filtered'),
        ]
    )
    
    
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        enable_scan_matching_arg,
        enable_imu_processing_arg,
        scan_topic_arg,
        imu_topic_arg,
        base_frame_arg,
        
        # Core localization nodes
        enhanced_wheel_odometry_node,
        adaptive_scan_matcher_node,
        enhanced_imu_processor_node,
        robot_localization_node,
    ])

# Additional launch file for testing individual components
def generate_test_launch_description():
    """Generate launch description for testing individual components"""
    
    pkg_share = FindPackageShare('custom_ackermann_controller')
    
    # Test arguments
    component_arg = DeclareLaunchArgument(
        'component',
        default_value='wheel_odometry',
        description='Component to test: wheel_odometry, scan_matching, imu_processing, authority_manager'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time for testing'
    )
    
    # Component-specific test nodes would be defined here
    # This is a framework for individual component testing
    
    return LaunchDescription([
        component_arg,
        use_sim_time_arg,
        # Test nodes would be added based on component argument
    ])