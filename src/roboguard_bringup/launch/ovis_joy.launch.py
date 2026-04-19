from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    joy_config = PathJoinSubstitution([
        FindPackageShare("roboguard_bringup"), "config", "ovis_joy.yaml"
    ])
    # Arguments

    prefix_arg = DeclareLaunchArgument(
        "prefix",
        default_value="ovis",
        description="Xacro prefix used in joint/link names",
    )

    damping_arg = DeclareLaunchArgument(
        "damping",
        default_value="0.05",
        description="DLS damping factor for the IK solver",
    )

    max_joint_vel_arg = DeclareLaunchArgument(
        "max_joint_vel",
        default_value="1.0",
        description="Maximum joint velocity output (rad/s)",
    )

    # teleop_twist_joy
    # Publishes TwistStamped; remapped directly to the IK node's input topic.
    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="ovis_teleop_twist_joy_node",
        parameters=[joy_config],
        remappings=[
            ("/joy", "/rove/station/joy"),
            ("/cmd_vel", "/ovis/cmd_cartesian_vel"),
        ],
    )

    # Ovis velocity IK node
    ik_node = Node(
        package="ovis_ik",
        executable="velocity_node",
        name="ovis_velocity_ik",
        remappings=[
            ("/cmd_vel", "/ovis/cmd_cartesian_vel"),
            ("/joint_vel", "/ovis_controller/commands"),
        ],
        output="screen",
    )

    return LaunchDescription([
        prefix_arg,
        damping_arg,
        max_joint_vel_arg,
        teleop_node,
        ik_node,
    ])