import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('mapping'),
        'config',
        'frontier_exploration_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='mapping',
            executable='frontier_exploration',
            name='frontier_exploration',
            output='screen',
            parameters=[config_file]
        )
    ])