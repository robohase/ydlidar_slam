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
    rviz2_config = os.path.join(slam_share_dir, 'rviz', 'cartographer.rviz')
    
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
    
    # TF2 static transform publisher
    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0', '0', '0.02', '0', '0', '0', '1', 'base_link', 'laser_frame'],
    )
    
    # Laser filter node
    laser_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            PathJoinSubstitution([
                get_package_share_directory("laser_filters"),
                "examples",
                "box_filter_example.yaml",
            ])
        ],
    )
    
    # Cartographer nodes
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'cartographer.lua'
        ]
    )
    
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '1.0'
        ]
    )
    
    # RViz2 node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[["-d"], [rviz2_config]]
    )
    
    # Create and return launch description
    return LaunchDescription([
        params_declare,
        driver_node,
        tf2_node,
        laser_filter_node,
        cartographer_node,
        occupancy_grid_node,
        rviz2_node,
    ])