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
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Gazebo Simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            )
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
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
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
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

    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        diff_drive_delay,
        joint_state_broadcaster_delay
    ])
