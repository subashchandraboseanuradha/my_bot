#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # diff_tf node
    diff_tf_node = Node(
        package='my_bot',
        executable='diff_tf',
        name='diff_tf',
        output='screen',
        parameters=[{
            'ticks_meter': 3831.75,
            'base_width': 0.255,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'encoder_min': -32768,
            'encoder_max': 32768
        }]
    )
    
    return LaunchDescription([
        diff_tf_node
    ]) 