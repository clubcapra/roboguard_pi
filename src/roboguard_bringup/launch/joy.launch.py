import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    ExecuteProcess,
    TimerAction,
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit, OnShutdown


def generate_launch_description():
    # Get the launch directory
    pkg_roboguard_bringup = get_package_share_directory("roboguard_bringup")

    return LaunchDescription(
        [
        ]
    )