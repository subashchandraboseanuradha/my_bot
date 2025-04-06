from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    joy_params = os.path.join(get_package_share_directory('my_bot'),'config','joystick.yaml')

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
            output='screen',
         )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel','/diff_cont/cmd_vel')],
            output='screen',
         )

    # Diagnostic commands to check joystick inputs and velocity outputs
    # These will open in separate terminals to monitor the data
    joy_echo = ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '/joy'],
            output='screen',
         )

    cmd_vel_echo = ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '/diff_cont/cmd_vel_unstamped'],
            output='screen',
         )

    # For Humble, we might need the twist_stamper if using stamped velocity commands
    # Uncomment if needed later
    # twist_stamper = Node(
    #         package='twist_stamper',
    #         executable='twist_stamper',
    #         parameters=[{'use_sim_time': use_sim_time}],
    #         remappings=[('/cmd_vel_in','/diff_cont/cmd_vel_unstamped'),
    #                     ('/cmd_vel_out','/diff_cont/cmd_vel')]
    #      )


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        joy_node,
        teleop_node,
        joy_echo,
        cmd_vel_echo,
        # twist_stamper       
    ])