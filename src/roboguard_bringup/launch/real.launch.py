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
    

    # Get the URDF file
    urdf_path = os.path.join(pkg_roboguard_description, "urdf", "rove.base.urdf.xacro")
    robot_desc = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([urdf_path]),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )
    
    # Takes the description and joint angles as inputs and publishes
    # the 3D poses of the robot links
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_desc},
        ],
    )
    
    # Controllers
    controller_nodes = ["odrive_controller", "diff_drive_controller"]
    
    
    ###### ROS2 control ######
    robot_controllers = PathJoinSubstitution(
        [
            pkg_roboguard_description,
            # pkg_roboguard_bringup,
            "config",
            "roboguard_controllers.yaml",
        ]
    )
    

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[{"robot_description": robot_desc}, robot_controllers],
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    def create_controller_node(node_name: str, after):
        robot_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[node_name, "-c", "/controller_manager"],
        )

        # Delay start of robot_controller after `joint_state_broadcaster`
        delay_robot_controller_spawner_after = (
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=after,
                    on_exit=[robot_controller_spawner],
                )
            )
        )
        return delay_robot_controller_spawner_after, robot_controller_spawner



    delayed_controller_nodes = list()
    last_spawner = joint_state_broadcaster_spawner
    for controller in controller_nodes:
        node, spawner = create_controller_node(controller, last_spawner)
        delayed_controller_nodes.append(node)
        last_spawner = spawner

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
    
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        output="screen",
        parameters=[pkg_roboguard_bringup + "/config/twist_mux.yaml"],
        remappings={
            ("/cmd_vel_out", "/rove/cmd_vel"),
            ("nav_vel", "/rove/nav/cmd_vel"),
            ("twist_vel", "/rove/twist/cmd_vel"),
            ("tank_vel", "/rove/tank/cmd_vel"),
        },
    )
    
    cmd_vel_relay = Node(
        package="topic_tools",
        executable="relay",
        name="diff_drive_remap",
        output="screen",
        arguments=("/rove/cmd_vel", "/diff_drive_controller/cmd_vel_unstamped")
    )
    
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_roboguard_bringup, "launch", "teleop.launch.py"),
        ),
    )

    return LaunchDescription(
        [
            use_mock_hardware_dec,
            # start_can_cmd,
            # shutdown,
            robot_state_publisher,
            control_node,
            joint_state_broadcaster_spawner,
            *delayed_controller_nodes,
            twist_mux,
            cmd_vel_relay,
            teleop,
        ]
    )
