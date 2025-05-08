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
    pkg_roboguard_odrive = get_package_share_directory("roboguard_odrive")

    odrive_config_path = os.path.join(
        pkg_roboguard_odrive, "config", "odrive.yaml"
    )

    odrive_node = Node(
        package="roboguard_odrive",
        executable="odrive_control",
        name="odrive_control",
        output="screen",
        parameters=[odrive_config_path],
    )

    return LaunchDescription(
        [
            odrive_node,
        ]
    )