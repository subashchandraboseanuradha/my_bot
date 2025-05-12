#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get package directory
    pkg_dir = os.path.dirname(os.path.dirname(__file__))
    
    # Declare launch arguments
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    declare_use_mock_hardware = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='false',
        description='Whether to use mock hardware')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true, real clock if false')
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            os.path.join(pkg_dir, "description", "robot.urdf.xacro"),
            " ",
            "use_mock_hardware:=", use_mock_hardware,
            " ",
            "sim_mode:=false",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    controller_config = os.path.join(pkg_dir, 'config', 'my_controllers.yaml')

    # Define transform publishers
    # Base footprint to base_link
    static_base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_footprint_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time, 'publish_frequency': 100.0}],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description, 
            {
                'use_sim_time': use_sim_time, 
                'frame_prefix': '', 
                'publish_frequency': 100.0,  # Increased frequency
                'transform_tolerance': 2.5   # Increased transform tolerance
            }
        ],
    )

    # Controller nodes
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config, {'use_sim_time': use_sim_time}],
        output="screen",
    )

    # Joystick controller - include only if file exists
    joystick_launch_path = os.path.join(pkg_dir, 'launch', 'joystick.launch.py')
    
    joystick_ld = None
    if os.path.exists(joystick_launch_path):
        joystick_ld = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([joystick_launch_path]),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Differential drive controller spawner
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "--controller-manager", "/controller_manager"],
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Define the final launch description and return it
    nodes = [
        declare_use_mock_hardware,
        declare_use_sim_time,
        # Only include these transforms
        static_base_footprint_to_base_link,
    ]
    
    # Step 1: Launch robot_state_publisher first
    nodes.append(robot_state_pub_node)
    
    # Step 2: Delay controller_manager to ensure robot_state_publisher has established frames
    delayed_controller_manager = TimerAction(
        period=3.0,  # Increased from 2.0 to 3.0 seconds
        actions=[controller_manager]
    )
    nodes.append(delayed_controller_manager)
    
    # Step 3: Delay joint_state_broadcaster after controller_manager is running
    delayed_joint_state_broadcaster_spawner = TimerAction(
        period=6.0,  # Increased from 3.0 to 6.0 seconds (3s delay after controller_manager)
        actions=[joint_state_broadcaster_spawner]
    )
    nodes.append(delayed_joint_state_broadcaster_spawner)
    
    # Step 4: Delay diff_drive_spawner after joint_state_broadcaster
    delayed_diff_drive_spawner = TimerAction(
        period=8.0,  # Increased from 5.0 to 8.0 seconds (2s delay after joint_state_broadcaster)
        actions=[diff_drive_spawner]
    )
    nodes.append(delayed_diff_drive_spawner)

    # Add joystick if available, with a delay to ensure all controllers are running
    if joystick_ld is not None:
        delayed_joystick = TimerAction(
            period=10.0,  # Start joystick 2s after diff_drive controller
            actions=[joystick_ld]
        )
        nodes.append(delayed_joystick)

    return LaunchDescription(nodes)