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

    # Add static transform publisher for map->odom to stabilize the TF tree when needed
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Create launch description elements list
    nodes = [
        rsp,
        gazebo,
        spawn_entity,
        diff_drive_delay,
        joint_state_broadcaster_delay,
        static_tf,
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

    return LaunchDescription(nodes)
