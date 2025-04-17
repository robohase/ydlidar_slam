#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
import lifecycle_msgs.msg
import os


def generate_launch_description():
    # Get package directories
    ydlidar_share_dir = get_package_share_directory('ydlidar_ros2_driver')
    slam_share_dir = get_package_share_directory('ydlidar_slam_toolbox')
    
    # Configuration files
    parameter_file = LaunchConfiguration('params_file')
    cartographer_config_dir = os.path.join(slam_share_dir, 'config')
    rviz2_config = os.path.join(slam_share_dir, 'rviz', 'slam_toolbox.rviz')
    
    # Define mapping param file (missing in original)
    mapping_param_dir = os.path.join(slam_share_dir, 'config', 'mapping.yaml')
    
    # Launch arguments
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(ydlidar_share_dir, 'params', 'Tmini.yaml'),
        description='Path to the ROS2 parameters file to use.'
    )
    
    # YDLidar driver node
    driver_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file],
        namespace='/',
    )
    
    # Laser scan matcher node
    laser_scan_matcher_node = Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        name='laser_scan_matcher_node',
        output='screen',
        parameters=[mapping_param_dir]
    )
    
    # SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            mapping_param_dir,
            {'use_sim_time': False}
        ],
    )
    
    # Map to Odom TF (親→子の順序)
    map_to_odom_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    
    # Base Link to Laser Frame TF (親→子の順序)
    base_link_to_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_tf_publisher',
        arguments=['0', '0', '0.02', '0', '0', '0', 'base_link', 'laser_frame'],
        output='screen'
    )
    
    # RViz2 node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d", rviz2_config]
    )
    
    # Create and return launch description
    return LaunchDescription([
        params_declare,
        driver_node,
        laser_scan_matcher_node,
        slam_toolbox_node,
        map_to_odom_tf_node,
        base_link_to_laser_tf_node,
        rviz2_node,
    ])