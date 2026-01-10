import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = "roboguard_bringup"
    dir = get_package_share_directory(package_name)
    # Get params files
    joy_params_file = dir + "/config/joy_params.yaml"
    teleop_joy_params_file = dir + "/config/teleop_joy_params.yaml"
    
    tracks = [
        "track_fl_j",
        "track_fr_j",
        "track_rl_j",
        "track_rr_j",
    ]

    return LaunchDescription(
        [
            Node(
                package="joy",
                executable="joy_node",
                name="game_controller_node",
                output="screen",
                parameters=[joy_params_file],
                remappings=[
                    ("/joy", "/rove/joy"),
                ],
            ),
            Node(
                package="teleop_twist_joy",
                executable="teleop_node",
                name="teleop_twist_joy_node",
                parameters=[teleop_joy_params_file],
                remappings=[
                    ("/joy", "/rove/joy"),
                    ("/cmd_vel", "/diff_drive_controller/cmd_vel_unstamped"),
                ],
            ),
            Node(
                package="topic_tools",
                executable="relay",
                name="enable_joy_relay",
                output="screen",
                arguments=[
                    "/rove/joy",
                    "/enable_node/joy"
                ]
            ),
            Node(
                package="capra_joy_to_bool",
                executable="joy_to_bool",
                name="enable_node",
                output="screen",
                parameters=[joy_params_file],
                remappings=[
                    ("/enable_node/bool", "/rove/enable"),
                ],
            ),
            *[
                Node(
                    package="topic_tools",
                    executable="relay",
                    name=f"{track}_enable_relay",
                    output="screen",
                    arguments=[
                        "/rove/enable",
                        f"/odrive_controller/enable/{track}"
                    ]
                )
                for track in tracks
            ]
        ]
    )
