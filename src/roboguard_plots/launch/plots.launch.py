import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_name = "roboguard_plots"
    dir = get_package_share_directory(package_name)
    config_path = dir + "/config/layout.xml"
    
    plotter = Node(
        package="plotjuggler",
        executable="plotjuggler",
        name="plots",
        arguments=("--layout", config_path),
        output="screen",
    )
    
    return LaunchDescription(
        [
            plotter,
        ]
    )
