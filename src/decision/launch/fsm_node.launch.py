from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = PathJoinSubstitution(
        [FindPackageShare("decision"), "config", "fsm_params.yaml"]
    )

    fsm_node = Node(
        package="decision",
        executable="fsm_node",
        name="fsm_node",
        parameters=[config_path],
        output="screen",
    )

    return LaunchDescription([fsm_node])
