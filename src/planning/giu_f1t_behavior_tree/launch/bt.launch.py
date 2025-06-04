import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('giu_f1t_behavior_tree'),
        'config',
        'behavior_tree_params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='giu_f1t_behavior_tree',
            executable='bt_node',
            name='behavior_tree',
            output='screen',
            parameters=[config]
        )
    ])