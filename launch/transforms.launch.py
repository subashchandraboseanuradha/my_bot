#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        # map to odom transform (static)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        
        # base_footprint to base_link transform (static)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_base_footprint_to_link',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])
    
    