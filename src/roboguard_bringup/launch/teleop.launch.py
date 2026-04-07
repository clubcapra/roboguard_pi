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
            ("/joy_mux_topic", "/rove/station/joy"),
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
            ("/scheme_selector/joy", "/rove/station/joy"),
            ("/scheme_selector/bool", "/rove/station/teleop_select"),
        ],
    )
    
    # Reroutes correct teleop command based on selected scheme
    teleop_mapper = Node(
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
    
    # Standard teleop controls
    teleop_twist_joy = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[joy_config_file],
        remappings=[
            ("/joy", "/rove/station/twist/joy"),
            ("/cmd_vel", "/rove/station/twist/cmd_vel"),
        ],
    )
    
    # Tank controls
    tank_twist = Node(
        package="capra_tank_twist",
        executable="tank_twist",
        name="tank_twist_node",
        parameters=[joy_config_file],
        remappings=[
            ("/joy", "/rove/station/tank/joy"),
            ("/cmd_vel", "/rove/station/tank/cmd_vel"),
        ],
    )
    
    # Flipers control
    flippers_teleop = Node(
        package="capra_flippers_teleop",
        executable="flippers_teleop",
        name="flippers_teleop",
        parameters=[joy_config_file],
        remappings=[
            ("/flippers_teleop/joy", "/rove/station/joy"),
            ("/flippers_teleop/flippers", "/rove/to_robot/teleop/flippers_cmd"),
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
            ("/enable_node/joy", "/rove/station/joy"),
            ("/enable_node/bool", "/rove/to_robot/enable_stamped"),
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
            ("/estop_control/joy", "/rove/station/joy"),
            ("/estop_control/bool", "/rove/to_robot/estop_stamped"),
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
            ("/joy_input/steam", "/rove/station/steamdeck/joy"),
            ("/joy_input/xbox_bl_controller_laptop", "/rove/station/xbox/bluetooth/joy"),
            ("/joy_input/xbox_usb_controller_laptop", "/rove/station/xbox/usb/joy"),
        ]
    )
    
    # Demuxes
    demux_config_file = os.path.join(pkg_roboguard_bringup, "config", "station_demux.yaml")
    demuxes = ["station_cmd_vel_demux", "station_joint_states_demux", "station_dynamic_joint_states_demux"]
    demux_nodes = [
        Node(
            package="capra_stamp_demux",
            executable="stamp_demux",
            name=name,
            parameters=[demux_config_file],
            output="screen",
        )
        for name in demuxes
    ]
    
    # UDP bridge
    udp_bridge_config_file = os.path.join(pkg_roboguard_bringup, "config", "udp_bridge.yaml")
    udp_bridge = Node(
        package="capra_udp_bridge",
        executable="udp_bridge_node",
        name="station_udb_bridge",
        output="screen",
        parameters=[udp_bridge_config_file],
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
            *demux_nodes,
            udp_bridge,
        ]
    )
