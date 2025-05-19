import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_bot'
    
    # Enable use_sim_time for all nodes
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
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
            'use_sim_time': use_sim_time, 
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

    # Gazebo parameters file
    gazebo_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'gazebo_params.yaml'
    )

    # TF Tree Configuration
    # 1. Map to Odom (static) - REMOVED to avoid conflict with SLAM toolbox
    # 2. Base Footprint to Base Link (static)
    static_base_footprint_to_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_footprint_to_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time, 'publish_frequency': 100.0, 'transform_tolerance': 1.0}],
        output='screen'
    )

    # 3. Add TF buffer and listener for debugging
    tf_buffer = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_buffer',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
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
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Load controllers after robot is spawned
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
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
        static_base_footprint_to_link,
        tf_buffer,
        rsp,
        gazebo,
        spawn_entity,
        diff_drive_delay,
        joint_state_broadcaster_delay,
    ]
    
    # Add joystick if it exists
    if os.path.exists(joystick_launch_path):
        print("Including joystick launch file")
        joystick = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([joystick_launch_path]), 
            launch_arguments={'use_sim_time': use_sim_time}.items()
        )
        nodes.append(joystick)
    else:
        print("WARNING: Joystick launch file not found at", joystick_launch_path)

    return LaunchDescription(nodes)
