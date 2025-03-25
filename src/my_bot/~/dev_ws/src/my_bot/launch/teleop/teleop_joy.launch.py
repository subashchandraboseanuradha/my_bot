#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the config file
    config_file = os.path.join(
        get_package_share_directory('my_bot'),
        'config',
        'joystick.yaml'
    )
    
    return LaunchDescription([
        # Launch argument to decide whether to use stamped or non-stamped cmd_vel
        DeclareLaunchArgument(
            'use_stamped',
            default_value='false',
            description='Use stamped cmd_vel topic if true'
        ),
        
        # Launch argument for joy device
        DeclareLaunchArgument(
            'joy_device',
            default_value='/dev/input/js0',
            description='Joystick device path'
        ),

        # Launch argument for joystick type
        DeclareLaunchArgument(
            'joy_config',
            default_value='xbox',
            description='Joystick type (xbox, ps4, f710)'
        ),
        
        # Joy node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'device': LaunchConfiguration('joy_device'),
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }]
        ),
        
        # Teleop node
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[
                config_file,
                {'joy_config': LaunchConfiguration('joy_config')}
            ],
            remappings=[
                # Handle both standard and stamped cmd_vel
                ('/cmd_vel', '/cmd_vel' if LaunchConfiguration('use_stamped') == 'false' else '/cmd_vel_raw'),
            ],
        ),
        
        # Node for converting between standard and stamped cmd_vel
        Node(
            package='topic_tools',
            executable='transform',
            name='twist_to_twist_stamped',
            parameters=[{
                'input_topic': '/cmd_vel_raw',
                'output_topic': '/cmd_vel_stamped',
                'output_type': 'geometry_msgs/msg/TwistStamped',
                'expression': '{header: {stamp: now(), frame_id: "base_link"}, twist: m}'
            }],
            condition=LaunchConfiguration('use_stamped'),
        ),
    ]) 