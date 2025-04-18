#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node
from launch.actions import IncludeLaunchDescription
import lifecycle_msgs.msg
import os


def get_file_path(package_name, file_path):
    """Get the path to a file in a ROS package."""
    return os.path.join(get_package_share_directory(package_name), file_path)


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
    
    driver_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file],
        namespace='/'
    )
    
    laser_scan_matcher_node = Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        name='laser_scan_matcher_node',
        output='screen',
        parameters=[mapping_param_dir]
    )
    
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            mapping_param_dir,
            {'use_sim_time': False}
        ]
    )
    
    map_to_odom_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    
    base_link_to_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_tf_publisher',
        arguments=['0', '0', '0.02', '0', '0', '0', 'base_link', 'laser_frame'],
        output='screen'
    )

    base_link_to_camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera_tf_node',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'realsense_link'],
        output='screen'
    )

    geotiff_node_slam_toolbox = Node(
        package='hector_geotiff',
        executable='geotiff_node_slam_toolbox',
        name='geotiff_node_slam_toolbox',
        output='screen',
        parameters=[
            mapping_param_dir,
            {'map_file_base_name': "Nexis-R"},
            {'mission_name': "M1"}
        ]
    )
    
    # RViz2 node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d", rviz2_config]
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('realsense2_camera'),
            '/launch/rs_launch.py'
        ]),
        launch_arguments={
            'rgb_camera.profile': '640x480x30',
            'depth_camera.profile': '640x480x30',
            'enable_rgb': 'true',
            'enable_depth': 'true',
            'camera_name': 'realsense',
            'camera_namespace': ''
        }.items()
    )
    
    # Create and return launch description
    return LaunchDescription([
        params_declare,
        driver_node,
        laser_scan_matcher_node,
        slam_toolbox_node,
        map_to_odom_tf_node,
        base_link_to_laser_tf_node,
        base_link_to_camera_tf_node,
        geotiff_node_slam_toolbox,
        rviz2_node,
        realsense_launch
    ])