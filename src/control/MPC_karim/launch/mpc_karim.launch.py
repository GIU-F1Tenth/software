"""Launch file for the MPC_karim controller.

Loads its parameters from
    src/giu_f1t_system/f1tenth_stack/config/MPC_karim_params.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    f1tenth_stack_share = get_package_share_directory('f1tenth_stack')
    default_params = os.path.join(
        f1tenth_stack_share, 'config', 'MPC_karim_params.yaml'
    )

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to MPC_karim params YAML.',
    )

    mpc_node = Node(
        package='MPC_karim',
        executable='mpc_karim_node',
        name='MPC_karim',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([params_arg, mpc_node])
