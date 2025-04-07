#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
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
            PathJoinSubstitution(
                [FindPackageShare("my_bot"), "description", "robot.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=", use_mock_hardware,
            " ",
            "sim_mode:=false",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    my_bot_dir = get_package_share_directory('my_bot')
    controller_config = os.path.join(my_bot_dir, 'config', 'my_controllers.yaml')

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config, {'use_sim_time': use_sim_time}],
        output="screen",
    )

    # Static transform publisher for map to odom
    static_map_to_odom_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Static transform publisher for odom to base_footprint
    static_odom_to_base_footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_to_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Static transform publisher for base_footprint to base_link
    static_base_footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_footprint_to_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Joystick controller - include only if file exists
    joystick_launch_path = os.path.join(my_bot_dir, 'launch', 'joystick.launch.py')
    
    joystick_ld = None
    if os.path.exists(joystick_launch_path):
        from launch.actions import IncludeLaunchDescription
        from launch.launch_description_sources import PythonLaunchDescriptionSource
        
        joystick_ld = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([joystick_launch_path]),
            launch_arguments={'use_sim_time': 'false'}.items()
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
        controller_manager,
        static_map_to_odom_publisher,
        static_odom_to_base_footprint_publisher,
        static_base_footprint_publisher,
        delayed_joint_state_broadcaster_spawner,
        delayed_diff_drive_spawner,
    ]

    # Add joystick if available
    if joystick_ld is not None:
        nodes.append(joystick_ld)

    return LaunchDescription(nodes) 