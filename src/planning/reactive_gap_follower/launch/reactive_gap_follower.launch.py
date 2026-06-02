"""Standalone launch for the reactive gap follower."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("reactive_gap_follower")
    default_config = os.path.join(pkg_share, "config", "reactive_gap_follower.yaml")

    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="Path to reactive_gap_follower parameter file.",
    )

    node = Node(
        package="reactive_gap_follower",
        executable="reactive_gap_follower_node",
        name="reactive_gap_follower_node",
        parameters=[LaunchConfiguration("config")],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([config_arg, node])
