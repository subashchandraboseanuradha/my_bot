#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch argument to decide whether to use stamped or non-stamped cmd_vel
        DeclareLaunchArgument(
            'use_stamped',
            default_value='false',
            description='Use stamped cmd_vel topic if true'
        ),
        
        # Node for converting between standard and stamped cmd_vel
        Node(
            package='topic_tools',
            executable='transform',
            name='twist_to_twist_stamped',
            parameters=[{
                'input_topic': '/cmd_vel',
                'output_topic': '/cmd_vel_stamped',
                'output_type': 'geometry_msgs/msg/TwistStamped',
                'expression': '{header: {stamp: now(), frame_id: "base_link"}, twist: m}'
            }],
            condition=LaunchConfiguration('use_stamped'),
        ),
        
        # Keyboard teleop node
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            prefix='xterm -e',
            remappings=[
                ('/cmd_vel', '/cmd_vel'),
            ],
        ),
    ]) 