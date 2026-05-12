import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("lidar_filter")
    default_config = os.path.join(package_dir, "config", "lidar_filter.yaml")

    declare_config_file = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="Path to the lidar_filter configuration YAML file",
    )

    lidar_filter_node = Node(
        package="lidar_filter",
        executable="lidar_filter",
        name="lidar_filter",
        parameters=[LaunchConfiguration("config_file")],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_config_file,
            lidar_filter_node,
        ]
    )
