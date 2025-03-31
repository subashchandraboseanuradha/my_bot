import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_bot'
    
    # Controller configuration
    controller_config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'my_controllers.yaml'
    )

    # Robot State Publisher (URDF/Xacro processing)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name),
                'launch','rsp.launch.py'
            )
        ]), 
        launch_arguments={
            'use_sim_time': 'true', 
            'use_ros2_control': 'true',
            'sim_mode': 'true'
        }.items()
    )

    # Joystick controller - only include if file exists
    joystick_launch_path = os.path.join(
        get_package_share_directory(package_name),
        'launch','joystick.launch.py'
    )
    
    # Debug print to show if joystick.launch.py is found
    print(f"Checking joystick path: {joystick_launch_path}")
    print(f"Joystick file exists: {os.path.exists(joystick_launch_path)}")

    # Add twist_mux if config exists
    twist_mux_params_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'twist_mux.yaml'
    )
    
    # Only include twist_mux if the config file exists
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[{'use_sim_time': True}],
        remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    )

    # Gazebo parameters file
    gazebo_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'gazebo_params.yaml'
    )

    # Gazebo Simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        ]),
        launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    )

    # Entity Spawning
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Load controllers after robot is spawned
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Make sure controllers start after robot is spawned
    diff_drive_delay = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[diff_drive_spawner],
        )
    )
    
    joint_state_broadcaster_delay = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # Create launch description elements list
    nodes = [
        rsp,
        gazebo,
        spawn_entity,
        diff_drive_delay,
        joint_state_broadcaster_delay
    ]
    
    # Add joystick if it exists
    if os.path.exists(joystick_launch_path):
        print("Including joystick launch file")
        joystick = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([joystick_launch_path]), 
            launch_arguments={'use_sim_time': 'true'}.items()
        )
        nodes.append(joystick)
    else:
        print("WARNING: Joystick launch file not found at", joystick_launch_path)
    
    # Add twist_mux if config exists
    if os.path.exists(twist_mux_params_path):
        twist_mux.parameters.append(twist_mux_params_path)
        nodes.append(twist_mux)

    return LaunchDescription(nodes)
