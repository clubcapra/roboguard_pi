import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
from launch_ros.actions import SetRemap


def generate_launch_description():
    pkg_roboguard_actions = get_package_share_directory("roboguard_actions")

    actions_config_path = os.path.join(
        pkg_roboguard_actions, "config", "config.yaml"
    )

    actions_node = Node(
        package="roboguard_actions",
        executable="actions",
        name="actions",
        output="screen",
        parameters=[actions_config_path],
    )

    return LaunchDescription(
        [
            actions_node,
        ]
    )