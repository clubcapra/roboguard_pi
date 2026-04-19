import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    ExecuteProcess,
    DeclareLaunchArgument,
    EmitEvent,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
    AndSubstitution,
    NotSubstitution,
)
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import UnlessCondition
from launch.events import Shutdown

handlers = []

def exit_on_error(actions=[]):
    def pred(event, context):
        return [*actions, EmitEvent(event=Shutdown())] if event.returncode != 0 else actions
    return pred

def exit_on_crash(node: Node, actions=[]):
    """Wraps a node so the launch file shuts down if it exits with a non-zero code."""
    handler = RegisterEventHandler(
        OnProcessExit(
            target_action=node,
            on_exit=exit_on_error(actions)
        )
    )
    handlers.append(handler)
    return node


def generate_launch_description():
    IN_DISTROBOX = os.path.exists("/run/host/etc")

    # Declare parameters
    use_mock_odrives_dec = DeclareLaunchArgument(
        "use_mock_odrives", default_value="false"
    )
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
    robot_state_publisher = exit_on_crash(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="both",
            parameters=[
                {"robot_description": robot_desc},
            ],
        )
    )

    # Can configuration
    can_prefix = "distrobox-host-exec " if IN_DISTROBOX else ""

    start_can_cmd = ExecuteProcess(
        cmd=[f"{can_prefix}sudo ip link set down can0 && "
            f"{can_prefix}sudo ip link set can0 type can bitrate 500000 && "
            f"{can_prefix}sudo ifconfig can0 txqueuelen 1000 && "
            f"{can_prefix}sudo ip link set up can0 && "
            "sleep 5"],
        shell=True,
        condition=UnlessCondition(use_mock_odrives),
    )

    stop_can_cmd = ExecuteProcess(
        cmd=[[f"{can_prefix}sudo ip link set down can0"]],
        shell=True,
    )


    # Controllers
    controller_nodes = ["odrive_controller", "diff_drive_controller", "ovis_controller"]
    controller_conditions = {
        "ovis_controller": IfCondition(real_ovis_present),
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

    control_node_start = RegisterEventHandler(
        OnProcessExit(
            target_action=start_can_cmd,
            on_exit=[
                control_node,
            ],
        ),
        condition=UnlessCondition(use_mock_odrives),
    )

    # On hardware: always tear down CAN when control_node exits; only emit
    # Shutdown if it exited non-zero (crash). Clean shutdowns fall through
    # to the normal launch termination.
    can_shutdown = RegisterEventHandler(
        OnProcessExit(
            target_action=control_node,
            on_exit=exit_on_error(actions=[stop_can_cmd]),
        ),
        condition=UnlessCondition(use_mock_odrives),
    )

    # Mock mode: no CAN teardown needed, but still fail the launch if the
    # mock control node crashes (exit_on_crash registers a handler that is
    # later spread into the LaunchDescription via *handlers).
    control_node_mock = exit_on_crash(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="both",
            parameters=[{"robot_description": robot_desc}, robot_controllers],
            condition=IfCondition(use_mock_odrives),
        )
    )

    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
            )
        ],
    )

    delay_joint_state_after_hardware_start = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_can_cmd,
            on_exit=[joint_state_broadcaster_spawner],
        ),
    )

    def create_controller(node_name):
        kwargs = {}
        if node_name in controller_conditions:
            kwargs = {"condition":controller_conditions[node_name]}
        return TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[node_name, "-c", "/controller_manager"],
                    **kwargs,
                )
            ],
        )

    # Twist mux
    twist_mux = exit_on_crash(
        Node(
            package="capra_twist_mux",
            executable="twist_mux",
            output="screen",
            parameters=[pkg_roboguard_bringup + "/config/twist_mux.yaml"],
            remappings={
                ("/cmd_vel_out", "/rove/robot/cmd_vel"),
                ("nav_vel", "/rove/robot/nav/cmd_vel"),
                ("remote_vel", "/rove/robot/comm/cmd_vel"),
            },
        )
    )

    # Relay messages from /rove/cmd_vel -> /diff_drive_controller/cmd_vel
    cmd_vel_relay = Node(
        package="topic_tools",
        executable="relay",
        name="diff_drive_remap",
        output="screen",
        arguments=("/rove/robot/cmd_vel", "/diff_drive_controller/cmd_vel"),
    )

    # Relay messages for odrive enables
    enable_relays = [
        Node(
            package="topic_tools",
            executable="relay",
            name=f"track_{track}_remap",
            output="screen",
            arguments=("/rove/robot/enable", f"/odrive_controller/enable/track_{track}_j"),
        )
        for track in ["rl", "rr", "fl", "fr"]
    ]
    
    # Demuxes
    demux_config_file = os.path.join(pkg_roboguard_bringup, "config", "robot_demux.yaml")
    demuxes = ["robot_cmd_vel_demux", "robot_enable_demux", "robot_estop_demux"]
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
    
    # Rosbag logging
    rosbag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_roboguard_bringup, "launch", "rosbag.launch.py"),
        ),
        condition=IfCondition(with_rosbag),
    )

    return LaunchDescription([
        use_mock_odrives_dec,
        use_mock_ovis_dec,
        with_ovis_dec,
        with_rosbag_dec,

        rosbag_launch,

        # CAN only in hardware mode
        start_can_cmd,

        # control node (mock OR hardware)
        control_node_mock,
        control_node_start,

        can_shutdown,

        robot_state_publisher,
        twist_mux,
        cmd_vel_relay,

        joint_state_broadcaster_spawner,
        *[create_controller(c) for c in controller_nodes],

        *enable_relays,
        
        *demux_nodes,

        *handlers,
    ])