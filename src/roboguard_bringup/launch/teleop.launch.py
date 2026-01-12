import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    ExecuteProcess,
    TimerAction,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit, OnShutdown, OnProcessStart
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare parameters
    use_mock_hardware_dec = DeclareLaunchArgument("use_mock_hardware", default_value="false")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    
    # Get the launch directory
    pkg_roboguard_description = get_package_share_directory("roboguard_description")
    pkg_roboguard_bringup = get_package_share_directory("roboguard_bringup")
    
    joy_config_file = pkg_roboguard_bringup + "/config/teleop_params.yaml"
    
    
    joy_mux = None # TODO add joy_mux node
    
    # Selects the teleop mapping
    scheme_selector = Node(
        package="capra_joy_to_bool",
        executable="joy_to_bool",
        name="scheme_selector",
        output="screen",
        parameters=[joy_config_file],
        remappings=[
            ("/scheme_selector/joy", "/rove/joy"),
            ("/scheme_selector/bool", "/rove/teleop_select"),
        ],
    )
    
    # Reroutes correct teleop command based on selected scheme
    teleop_mapper = Node(
        package="capra_teleop_mapper",
        executable="teleop_mapper",
        name="scheme_mapper",
        output="screen",
        remappings=[
            ("/scheme_mapper/joy1", "/rove/twist/joy"),
            ("/scheme_mapper/joy2", "/rove/tank/joy"),
            ("/sheme_mapper/joy_select", "/rove/teleop_select"),
            ("/scheme_mapper/joy", "/rove/teleop_joy"),
        ],
    )
    
    # Standard teleop controls
    teleop_twist_joy = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[joy_config_file],
        remappings=[
            ("/joy", "/rove/twist/joy"),
            ("/cmd_vel", "/rove/twist/cmd_vel"),
        ],
    )
    
    # Tank controls
    tank_twist = Node(
        package="capra_tank_twist",
        executable="tank_twist",
        name="tank_twist_node",
        parameters=[joy_config_file],
        remappings=[
            ("/joy", "/rove/tank/joy"),
            ("/cmd_vel", "/rove/tank/cmd_vel"),
        ],
    )
    
    flippers_teleop = Node(
        package="capra_flippers_teleop",
        executable="flippers_teleop",
        name="flippers_teleop",
        parameters=[joy_config_file],
        remappings=[
            ("/flippers_teleop/joy", "/rove/joy"),
            ("/flippers_teleop/flippers", "/rove/teleop/flippers_cmd"),
        ],
    )

    enable_node = Node(
        package="capra_joy_to_bool",
        executable="joy_to_bool",
        name="enable_node",
        output="screen",
        parameters=[joy_config_file],
        remappings=[
            ("/enable_node/joy", "/rove/joy"),
            ("/enable_node/bool", "/rove/tracks/enable"),
        ],
    )
    
    estop_control = Node(
        package="capra_joy_to_bool",
        executable="joy_to_bool",
        name="estop_control",
        output="screen",
        parameters=[joy_config_file],
        remappings=[
            ("/estop_control/bool", "/rove/teleop/estop"),
        ],
    )
    
    return LaunchDescription(
        [
            # joy_mux,
            scheme_selector,
            teleop_mapper,
            teleop_twist_joy,
            tank_twist,
            flippers_teleop,
            enable_node,
            estop_control,
        ]
    )
