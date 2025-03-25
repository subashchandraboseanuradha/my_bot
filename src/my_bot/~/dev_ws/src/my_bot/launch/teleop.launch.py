#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'teleop_type',
            default_value='keyboard',
            description='Type of teleop to use (keyboard or joy)'
        ),
        DeclareLaunchArgument(
            'use_stamped',
            default_value='false',
            description='Use stamped cmd_vel topic if true'
        ),
        DeclareLaunchArgument(
            'joy_device',
            default_value='/dev/input/js0',
            description='Joystick device path'
        ),
        DeclareLaunchArgument(
            'joy_config',
            default_value='xbox',
            description='Joystick type (xbox, ps4, f710)'
        ),
    ]

    # Paths to teleop launch files
    keyboard_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_bot'),
                'launch/teleop/teleop_keyboard.launch.py'
            ])
        ]),
        launch_arguments={
            'use_stamped': LaunchConfiguration('use_stamped'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('teleop_type', default='keyboard') == 'keyboard')
    )

    joystick_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_bot'),
                'launch/teleop/teleop_joy.launch.py'
            ])
        ]),
        launch_arguments={
            'use_stamped': LaunchConfiguration('use_stamped'),
            'joy_device': LaunchConfiguration('joy_device'),
            'joy_config': LaunchConfiguration('joy_config'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('teleop_type', default='keyboard') == 'joy')
    )

    return LaunchDescription(declared_arguments + [
        keyboard_teleop_launch,
        joystick_teleop_launch,
    ]) 