import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'my_bot'
    pkg_dir = get_package_share_directory(pkg_name)
    configuration_basename = 'my_lidar_2d.lua'
    load_state_filename_arg = DeclareLaunchArgument(
        'load_state_filename',
        default_value='',
        description='Full path to the .pbstream file to load. Leave empty for mapping.'
    )
    load_state_filename = LaunchConfiguration('load_state_filename')

    return LaunchDescription([
        load_state_filename_arg,
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '-configuration_directory', os.path.join(pkg_dir, 'config'),
                '-configuration_basename', configuration_basename,
                '-load_state_filename', load_state_filename
            ],
            remappings=[
                ('echoes', 'horizontal_laser_2d'),
                ('scan', 'scan') 
            ]
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'resolution': 0.05
            }]
        ),
    ])