import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    # Declare arguments with proper substitution
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )
    
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('my_bot'),
            'config',
            'mapper_params_online_async.yaml'
        ]),
        description='Full path to SLAM parameters file'
    )

    # SLAM Node with critical fixes
    start_async_slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            slam_params_file,
            {
                'transform_publish_period': 0.01,
                'minimum_time_interval': 0.05,
                'ceres_num_threads': 2,         # Limit CPU cores
                'enable_interactive_mode': False,
                'enable_localization': False    # Disable if not needed
            }
        ],
        remappings=[
            ('/scan', '/scan'),
            ('/slam_toolbox/get_state', 'get_state'),
            ('/slam_toolbox/change_state', 'change_state')
        ]
    )

    # Lifecycle manager with explicit service mappings
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='slam_lifecycle_manager',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['slam_toolbox']},
            {'timeout': 60.0},  # 1 minute timeout
            {'bond_timeout': 5.0}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(lifecycle_manager)

    return ld