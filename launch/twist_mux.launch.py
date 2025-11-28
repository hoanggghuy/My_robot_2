import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    #Path to .yaml file
    pkg_dir = get_package_share_directory('my_bot')
    twist_mux_config_file = os.path.join(pkg_dir,'config', 'twist_mux.yaml')

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_config_file],
        remappings=[
            ('cmd_vel_out', '/cmd_vel')
        ]
    )
    return LaunchDescription(
        [
            twist_mux_node
        ]
    )