import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the launch directory
    pkg_roboguard_bringup = get_package_share_directory("roboguard_bringup")
    pkg_input_manager = get_package_share_directory("input_manager")
    
    default_config = os.path.join(pkg_input_manager, 'config', 'default_config.yaml')
    
    # Parameters
    input_config_dec = DeclareLaunchArgument("input_config", default_value=default_config)
    input_config = LaunchConfiguration("input_config")
    
    joy_config_file = pkg_roboguard_bringup + "/config/teleop_params.yaml"
    
    
    joy_mux = Node(
        package="joy_mux",
        executable="joy_mux",
        output="screen",
        parameters=[joy_config_file],
        remappings=[
            ("/joy_mux_topic", "/rove/joy"),
        ],
    )
    
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
            ("/joy1", "/rove/twist/joy"),
            ("/joy2", "/rove/tank/joy"),
            ("/joy_select", "/rove/teleop_select"),
            ("/joy", "/rove/joy"),
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
    
    # Flipers control
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

    # ODrive enables
    enable_node = Node(
        package="capra_joy_to_bool",
        executable="joy_to_bool",
        name="enable_node",
        output="screen",
        parameters=[joy_config_file],
        remappings=[
            ("/enable_node/joy", "/rove/joy"),
            ("/enable_node/bool", "/rove/enable"),
        ],
    )
    
    # Estop message
    estop_control = Node(
        package="capra_joy_to_bool",
        executable="joy_to_bool",
        name="estop_control",
        output="screen",
        parameters=[joy_config_file],
        remappings=[
            ("/estop_control/joy", "/rove/joy"),
            ("/estop_control/bool", "/rove/estop"),
        ],
    )
    
    # input_manager
    input_manager = Node(
        package='input_manager',
        executable='input_manager',
        name='input_manager',
        output='screen',
        parameters=[{
            'config': input_config,
            'no_gui': True,
        }],
        remappings=[
            ("/joy_input/steam", "/rove/steamdeck/joy"),
            ("/joy_input/xbox_bl_controller_laptop", "/rove/xbox/bluetooth/joy"),
            ("/joy_input/xbox_usb_controller_laptop", "/rove/xbox/usb/joy"),
        ]
    )
    
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
        ]
    )
