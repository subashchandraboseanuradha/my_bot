#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    ticks_meter = LaunchConfiguration('ticks_meter', default='3831.75')  # Match your actual encoder ticks per meter
    base_width = LaunchConfiguration('base_width', default='0.255')  # Match wheel_separation in controllers.yaml
    
    # Launch Arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_ticks_meter_cmd = DeclareLaunchArgument(
        'ticks_meter',
        default_value='3831.75',
        description='Encoder ticks per meter of travel'
    )
    
    declare_base_width_cmd = DeclareLaunchArgument(
        'base_width',
        default_value='0.255',
        description='Width between the wheels in meters, should match wheel_separation in controllers.yaml'
    )
    
    # Static transform publisher for map to odom
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Create and return launch description
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_ticks_meter_cmd,
        declare_base_width_cmd,
        static_map_to_odom
    ]) 