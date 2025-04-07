import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Add launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for the RPLIDAR')
    
    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='Baud rate for the RPLIDAR serial connection')
    
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    
    return LaunchDescription([
        serial_port_arg,
        serial_baudrate_arg,
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': 'lidar_frame',
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }],
            arguments=['--ros-args', '--log-level', 'DEBUG']
        )
    ])
