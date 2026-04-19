import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


handlers = []


def exit_on_error(actions=None):
    actions = list(actions) if actions else []

    def pred(event, context):
        return (
            [*actions, EmitEvent(event=Shutdown())]
            if event.returncode != 0
            else actions
        )

    return pred


def exit_on_crash(node: Node, actions=None):
    """Wraps a node so the launch file shuts down if it exits with a non-zero code."""
    handler = RegisterEventHandler(
        OnProcessExit(
            target_action=node,
            on_exit=exit_on_error(actions),
        )
    )
    handlers.append(handler)
    return node


def generate_launch_description():
    # Get the launch directory
    pkg_roboguard_bringup = get_package_share_directory("roboguard_bringup")
    pkg_input_manager = get_package_share_directory("input_manager")

    default_config = os.path.join(pkg_input_manager, 'config', 'default_config.yaml')

    # Parameters
    input_config_dec = DeclareLaunchArgument("input_config", default_value=default_config)
    input_config = LaunchConfiguration("input_config")

    joy_config_file = pkg_roboguard_bringup + "/config/teleop_params.yaml"

    joy_mux = exit_on_crash(
        Node(
            package="joy_mux",
            executable="joy_mux",
            output="screen",
            parameters=[joy_config_file],
            remappings=[
                ("/joy_mux_topic", "/rove/station/joy"),
            ],
        )
    )

    # Selects the teleop mapping
    scheme_selector = exit_on_crash(
        Node(
            package="capra_joy_to_bool",
            executable="joy_to_bool",
            name="scheme_selector",
            output="screen",
            parameters=[joy_config_file],
            remappings=[
                ("/scheme_selector/joy", "/rove/station/joy"),
                ("/scheme_selector/bool", "/rove/station/teleop_select"),
            ],
        )
    )

    # Reroutes correct teleop command based on selected scheme
    teleop_mapper = exit_on_crash(
        Node(
            package="capra_teleop_mapper",
            executable="teleop_mapper",
            name="scheme_mapper",
            output="screen",
            remappings=[
                ("/joy1", "/rove/station/twist/joy"),
                ("/joy2", "/rove/station/tank/joy"),
                ("/joy_select", "/rove/station/teleop_select"),
                ("/joy", "/rove/station/joy"),
            ],
        )
    )

    # Standard teleop controls
    teleop_twist_joy = exit_on_crash(
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_twist_joy_node",
            parameters=[joy_config_file],
            remappings=[
                ("/joy", "/rove/station/twist/joy"),
                ("/cmd_vel", "/rove/station/twist/cmd_vel"),
            ],
        )
    )

    # Tank controls
    tank_twist = exit_on_crash(
        Node(
            package="capra_tank_twist",
            executable="tank_twist",
            name="tank_twist_node",
            parameters=[joy_config_file],
            remappings=[
                ("/joy", "/rove/station/tank/joy"),
                ("/cmd_vel", "/rove/station/tank/cmd_vel"),
            ],
        )
    )

    # Flipers control
    flippers_teleop = exit_on_crash(
        Node(
            package="capra_flippers_teleop",
            executable="flippers_teleop",
            name="flippers_teleop",
            parameters=[joy_config_file],
            remappings=[
                ("/flippers_teleop/joy", "/rove/station/joy"),
                ("/flippers_teleop/flippers", "/rove/to_robot/teleop/flippers_cmd"),
            ],
        )
    )

    # ODrive enables
    enable_node = exit_on_crash(
        Node(
            package="capra_joy_to_bool",
            executable="joy_to_bool",
            name="enable_node",
            output="screen",
            parameters=[joy_config_file],
            remappings=[
                ("/enable_node/joy", "/rove/station/joy"),
                ("/enable_node/bool", "/rove/to_robot/enable_stamped"),
            ],
        )
    )

    # Estop message
    estop_control = exit_on_crash(
        Node(
            package="capra_joy_to_bool",
            executable="joy_to_bool",
            name="estop_control",
            output="screen",
            parameters=[joy_config_file],
            remappings=[
                ("/estop_control/joy", "/rove/station/joy"),
                ("/estop_control/bool", "/rove/to_robot/estop_stamped"),
            ],
        )
    )

    # input_manager
    input_manager = exit_on_crash(
        Node(
            package='input_manager',
            executable='input_manager',
            name='input_manager',
            output='screen',
            parameters=[{
                'config': input_config,
                'no_gui': True,
            }],
            remappings=[
                ("/joy_input/steam", "/rove/station/steamdeck/joy"),
                ("/joy_input/xbox_bl_controller_laptop", "/rove/station/xbox/bluetooth/joy"),
                ("/joy_input/xbox_usb_controller_laptop", "/rove/station/xbox/usb/joy"),
            ]
        )
    )

    # Demuxes
    demux_config_file = os.path.join(pkg_roboguard_bringup, "config", "station_demux.yaml")
    demuxes = ["station_cmd_vel_demux"]
    demux_nodes = [
        exit_on_crash(
            Node(
                package="capra_stamp_demux",
                executable="stamp_demux",
                name=name,
                parameters=[demux_config_file],
                output="screen",
            )
        )
        for name in demuxes
    ]

    return LaunchDescription(
        [
            input_config_dec,
            joy_mux,
            scheme_selector,
            teleop_mapper,
            teleop_twist_joy,
            tank_twist,
            flippers_teleop,
            enable_node,
            estop_control,
            input_manager,
            *demux_nodes,

            *handlers,
        ]
    )