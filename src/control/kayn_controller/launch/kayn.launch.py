from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('kayn_controller'),
        'config', 'kayn_params.yaml'
    )
    return LaunchDescription([
        Node(
            package='kayn_controller',
            executable='kayn_node',
            name='kayn_controller_node',
            parameters=[config],
            output='screen',
            emulate_tty=True,
        )
    ])
