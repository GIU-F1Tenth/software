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
        "control_gateway_params.yaml",
    )
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_config,
        description="Params for control gateway",
    )

    control_gateway_node = Node(
        package=package_name,
        executable="control_gateway",
        name="control_gateway",
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription([
        params_file_arg,
        control_gateway_node,
    ])