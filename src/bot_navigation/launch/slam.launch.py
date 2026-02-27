import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the launch directory
    pkg_share = FindPackageShare(package='bot_navigation').find('bot_navigation')
    
    # Create the launch configuration variables
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # Declare the launch arguments
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value="/home/aditya-pachauri/prototype_autonomous_bot_cir_ws/src/bot_navigation/config/slam_toolbox_params.yaml",
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='True',
        description='Automatically startup the nav2 stack')

    # Include slam_toolbox's online async launch (forward use_sim_time and slam_params_file)
    # Assumption: slam_toolbox provides 'online_async_launch.py' in its 'launch' folder.
    slam_toolbox_launch_file = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    start_slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file
        }.items(),
    )


    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(start_slam_toolbox_cmd)

    return ld