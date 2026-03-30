import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    ExecuteProcess,
    DeclareLaunchArgument,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable, AndSubstitution, NotSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import UnlessCondition

from launch.conditions import UnlessCondition

def generate_launch_description():
    IN_DISTROBOX = os.path.exists('/run/host/etc')
    
    # Declare parameters
    use_mock_odrives_dec = DeclareLaunchArgument("use_mock_odrives", default_value="false")
    use_mock_odrives = LaunchConfiguration("use_mock_odrives")
    
    use_mock_ovis_dec = DeclareLaunchArgument("use_mock_ovis", default_value="false")
    use_mock_ovis = LaunchConfiguration("use_mock_ovis")
    
    with_ovis_dec = DeclareLaunchArgument("with_ovis", default_value="true")
    with_ovis = LaunchConfiguration("with_ovis")
    real_ovis_present = AndSubstitution(NotSubstitution(use_mock_ovis), with_ovis)
    
    with_rosbag_dec = DeclareLaunchArgument("with_rosbag", default_value="true")
    with_rosbag = LaunchConfiguration("with_rosbag")
    
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
            "use_mock_odrives:=",
            use_mock_odrives,
            " ",
            "use_mock_ovis:=",
            use_mock_ovis,
            " ",
            "with_ovis:=",
            with_ovis,
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
    controller_nodes = ["odrive_controller", "diff_drive_controller", "ovis_controller"]
    controller_conditions = {
        "ovis_controller" : IfCondition(real_ovis_present),
    }
    
    ###### ROS2 control ######
    robot_controllers = PathJoinSubstitution(
        [
            pkg_roboguard_description,
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
        
        kwargs = {}
        if node_name in controller_conditions.keys():
            kwargs = {"condition": controller_conditions[node_name]}

        # Delay start of robot_controller after `joint_state_broadcaster`
        delay_robot_controller_spawner_after = (
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=after,
                    on_exit=[robot_controller_spawner],
                ),
                **kwargs
            )
        )
        return delay_robot_controller_spawner_after, robot_controller_spawner



    delayed_controller_nodes = list()
    last_spawner = joint_state_broadcaster_spawner
    for controller in controller_nodes:
        node, spawner = create_controller_node(controller, last_spawner)
        delayed_controller_nodes.append(node)
        last_spawner = spawner

    # Can configuration
    can_prefix = 'distrobox-host-exec ' if IN_DISTROBOX else ''
    
    start_can_cmd = ExecuteProcess(
        cmd=[[
            f'{can_prefix}sudo ip link set down can0 && '
            f'{can_prefix}sudo ip link set can0 type can bitrate 500000 && '
            f'{can_prefix}sudo ifconfig can0 txqueuelen 1000 && '
            f'{can_prefix}sudo ip link set up can0'
        ]],
        shell=True,
        condition=UnlessCondition(use_mock_odrives),
    )

    stop_can_cmd = ExecuteProcess(
        cmd=[[f'{can_prefix}sudo ip link set down can0']],
        shell=True,
    )

    can_shutdown = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=control_node,
            on_exit=[stop_can_cmd],
        ),
        condition=UnlessCondition(use_mock_odrives),
    )

    # Twist mux
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
    
    # Relay messages from /rove/cmd_vel -> /diff_drive_controller/cmd_vel_unstamped
    cmd_vel_relay = Node(
        package="topic_tools",
        executable="relay",
        name="diff_drive_remap",
        output="screen",
        arguments=("/rove/cmd_vel", "/diff_drive_controller/cmd_vel_unstamped")
    )
    
    # Relay messages for odrive enables
    enable_relays = [
        Node(
            package="topic_tools",
            executable="relay",
            name=f"track_{track}_remap",
            output="screen",
            arguments=("/rove/enable", f"/odrive_controller/enable/track_{track}_j")
        )
        
        for track in ["rl", "rr", "fl", "fr"]
    ]
    
    # Rosbag logging
    rosbag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_roboguard_bringup, "launch", "rosbag.launch.py"),
        ),
        condition=IfCondition(with_rosbag),
    )
    
    return LaunchDescription(
        [
            # Declare args
            use_mock_odrives_dec,
            use_mock_ovis_dec,
            with_ovis_dec,
            with_rosbag_dec,
            
            # Start nodes
            rosbag_launch,
            start_can_cmd,
            can_shutdown,
            *enable_relays,
            robot_state_publisher,
            control_node,
            joint_state_broadcaster_spawner,
            *delayed_controller_nodes,
            twist_mux,
            cmd_vel_relay,
        ]
    )
