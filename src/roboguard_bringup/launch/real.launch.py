import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    ExecuteProcess,
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnShutdown


def generate_launch_description():
    # Get the launch directory
    pkg_roboguard_odrive = get_package_share_directory("roboguard_odrive")

    start_can_cmd = ExecuteProcess(
        cmd=[[
            'sudo ip link set down can0; sudo ip link set can0 type can bitrate 500000; sudo ifconfig can0 txqueuelen 1000; sudo ip link set up can0'
        ]],
        shell=True
    )

    stop_can_cmd = ExecuteProcess(
        cmd=[[
            'sudo ip link set down can0'
        ]],
        shell=True
    )

    shutdown = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[stop_can_cmd]
        )
    )

    odrive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_roboguard_odrive, "launch", "roboguard_odrive.launch.py"),
        ),
    )

    return LaunchDescription(
        [
            start_can_cmd,
            shutdown,
            odrive,
        ]
    )