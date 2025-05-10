#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('my_bot')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(pkg_dir, 'maps', 'my_map.yaml'))
    nav2_params_file = os.path.join(pkg_dir, 'config/nav2', 'nav2_params.yaml')
    rviz_config_file = os.path.join(pkg_dir, 'config', 'nav2_default_view.rviz')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')
    
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_dir, 'maps', 'my_map.yaml'),
        description='Full path to map yaml file to load')
    
    # Include robot launch file (without launching navigation)
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_dir, 'launch', 'launch_robot.launch.py')]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Include Nav2 bringup launch file
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_dir, 'launch', 'nav2_bringup.launch.py')]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': nav2_params_file,
            'autostart': 'true'
        }.items()
    )
    
    # Launch RViz2 with navigation configuration
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Static transform from base_footprint to base_link for TF tree completeness
    static_base_footprint_to_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_footprint_to_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time, 'publish_frequency': 100.0, 'transform_tolerance': 5.0}],
    )
    
    # Create and return launch description
    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        robot_launch,
        nav2_bringup_launch,
        rviz2_node,
        static_base_footprint_to_link
    ]) 