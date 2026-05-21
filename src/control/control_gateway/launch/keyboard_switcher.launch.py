from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_name = "control_gateway"

    default_config = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "keyboard_switcher_params.yaml",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_config,
        description="Params for keyboard switcher",
    )

    keyboard_switcher_node = Node(
        package=package_name,
        executable="keyboard_switcher",
        name="keyboard_switcher",
        parameters=[LaunchConfiguration("params_file")],
        output="screen",
    )

    return LaunchDescription([
        params_file_arg,
        keyboard_switcher_node,
    ])
