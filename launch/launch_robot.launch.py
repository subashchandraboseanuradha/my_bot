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

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description, 
            {
                'use_sim_time': use_sim_time, 
                'frame_prefix': '', 
                'publish_frequency': 50.0,  # Increased frequency
                'transform_tolerance': 0.5   # Added transform tolerance
            }
        ],
    )

    # Delay all other nodes to give robot_state_publisher time to publish frames
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config, {'use_sim_time': use_sim_time}],
        output="screen",
    )

    # Static transform publisher for map to odom - commented out to let EKF handle this transform
    # static_map_to_odom_publisher = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_map_to_odom',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    #     parameters=[{'use_sim_time': use_sim_time, 'publish_frequency': 100.0, 'transform_tolerance': 0.1}],
    # )

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

    # Delay joint_state_broadcaster after controller_manager
    delayed_joint_state_broadcaster_spawner = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner]
    )

    # Delay diff_drive_spawner after joint_state_broadcaster
    delayed_diff_drive_spawner = TimerAction(
        period=5.0,
        actions=[diff_drive_spawner]
    )

    # Define the final launch description and return it
    nodes = [
        declare_use_mock_hardware,
        declare_use_sim_time,
        robot_state_pub_node,
    ]

    # Delay controller_manager to ensure robot_state_publisher has published frames
    delayed_controller_manager = TimerAction(
        period=2.0,
        actions=[controller_manager]
    )
    nodes.append(delayed_controller_manager)
    
    # Add other nodes
    nodes.extend([
        delayed_joint_state_broadcaster_spawner,
        delayed_diff_drive_spawner,
    ])

    # Add transform publishers for the complete transform tree
    # Map to Odom (static)
    static_map_to_odom_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Remove static odom to base_footprint transform since it should come from odometry
    # static_odom_to_base_footprint_publisher = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_odom_to_base_footprint',
    #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
    #     parameters=[{'use_sim_time': use_sim_time}],
    # )

    # Add transform publishers to nodes list
    nodes.extend([
        static_map_to_odom_publisher,
        # static_odom_to_base_footprint_publisher,  # Removed static transform
    ])

    # Add joystick if available
    if joystick_ld is not None:
        nodes.append(joystick_ld)

    return LaunchDescription(nodes) 