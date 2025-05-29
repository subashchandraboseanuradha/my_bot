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
        description='Whether to use mock hardware'
    )

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

    # Define transform publisher

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description, 
            {
                'use_sim_time': use_sim_time, 
                'frame_prefix': '', 
                'publish_frequency': 200.0,  # Increased for higher frequency TF publishing
                'transform_tolerance': 0.01,  # Added lower transform tolerance
            }
        ],
    )

    # Controller nodes
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config, {'use_sim_time': use_sim_time}],
        output="screen",
        remappings=[
            ('/diff_cont/odom', '/odom'),
            ('/diff_cont/cmd_vel', '/cmd_vel')
        ]
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
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Define the final launch description and return it
    nodes = [
        declare_use_mock_hardware,
        declare_use_sim_time,
    ]
    
    # Step 1: Launch robot_state_publisher first
    nodes.append(robot_state_pub_node)
    
    # Step 2: Delay controller_manager to ensure robot_state_publisher has established frames
    delayed_controller_manager = TimerAction(
        period=20.0,  # Increased from previous suggestions
        actions=[controller_manager]
    )
    nodes.append(delayed_controller_manager)
    
    # Step 3: Delay joint_state_broadcaster after controller_manager is running
    delayed_joint_state_broadcaster_spawner = TimerAction(
        period=25.0,  # Increased from previous suggestions
        actions=[joint_state_broadcaster_spawner]
    )
    nodes.append(delayed_joint_state_broadcaster_spawner)
    
    # Step 4: Delay diff_drive_spawner after joint_state_broadcaster
    delayed_diff_drive_spawner = TimerAction(
        period=30.0,  # Increased from previous suggestions
        actions=[diff_drive_spawner]
    )
    nodes.append(delayed_diff_drive_spawner)
    
    # Add a transform republisher for laser scans to handle timestamp issues
    from launch_ros.actions import ComposableNodeContainer
    from launch_ros.descriptions import ComposableNode
    

    return LaunchDescription(nodes)