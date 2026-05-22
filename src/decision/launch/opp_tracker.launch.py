from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    config_path = os.path.join(
        get_package_share_directory("opp_tracker"),
        "config",
        "opp_tracker_params.yaml"
    )

    opp_tracker_node = Node(
        package='opp_tracker',
        executable='opp_tracker_node',
        name='opp_tracker',
        parameters=[config_path],
        output='screen'
    )

    ld.add_action(opp_tracker_node)

    return ld
