from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory("state_publisher")
    config_file = os.path.join(package_dir, "config", "state_publisher.yaml")

    declare_config_file = DeclareLaunchArgument(
        "config_file",
        default_value=config_file,
        description="Path to the configuration YAML file",
    )

    state_publisher_node = Node(
        package="state_publisher",
        executable="state_publisher",
        name="state_publisher",
        parameters=[LaunchConfiguration("config_file")],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_config_file,
            state_publisher_node,
        ]
    )
