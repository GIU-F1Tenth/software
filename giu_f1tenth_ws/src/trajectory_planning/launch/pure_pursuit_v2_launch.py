import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('trajectory_planning'),
        'config',
        'pure_pursuit_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='trajectory_planning',
            executable='pure_pursuit_v2',
            name='pure_pursuit',
            output='screen',
            parameters=[config_file]
        )
    ])