# plotjuggler.launch.py
import subprocess
import tempfile
import os
import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicJointState

from launch import LaunchDescription
from launch.actions import OpaqueFunction, ExecuteProcess
from ament_index_python.packages import get_package_share_directory


# ─────────────────────────────────────────────
# Step 1: Spin a temporary node to capture indices
# ─────────────────────────────────────────────

TRACK_JOINTS = {"track_fl_j", "track_fr_j", "track_rl_j", "track_rr_j"}

def get_dynamic_joint_indices(timeout_sec=5.0):
    rclpy.init()
    node = Node("_index_probe")
    result = {}
    received = False

    def cb(msg):
        nonlocal received
        for global_idx, (joint_name, iface) in enumerate(
            zip(msg.joint_names, msg.interface_values)
        ):
            if joint_name not in TRACK_JOINTS:
                continue

            inames = list(iface.interface_names)
            result[joint_name] = {
                "joint_idx": global_idx,
                "signals": {name: i for i, name in enumerate(inames)},
            }
        received = True

    sub = node.create_subscription(
        DynamicJointState, "/dynamic_joint_states", cb, 1
    )

    import time
    deadline = time.time() + timeout_sec
    while not received and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()

    if not received:
        raise RuntimeError(
            f"No message on /dynamic_joint_states within {timeout_sec}s."
        )
    return result

# ─────────────────────────────────────────────
# Step 2: Run xacro with extracted indices
# ─────────────────────────────────────────────

def run_xacro(xacro_path, indices, output_path):
    def joint_idx(name):
        return str(indices[name]["joint_idx"])

    def signal_idx(name, signal):
        signals = indices[name]["signals"]
        if signal not in signals:
            available = list(signals.keys())
            raise KeyError(
                f"Signal '{signal}' not found for '{name}'. "
                f"Available: {available}"
            )
        return str(signals[signal])

    # Sample signal indices from fl — all track joints share the same interface layout
    sample = "track_fl_j"

    xacro_args = [
        f"track_fl_joint_idx:={joint_idx('track_fl_j')}",
        f"track_fr_joint_idx:={joint_idx('track_fr_j')}",
        f"track_rl_joint_idx:={joint_idx('track_rl_j')}",
        f"track_rr_joint_idx:={joint_idx('track_rr_j')}",
        f"track_current_idx:={signal_idx(sample, 'bus_current')}",
        f"track_voltage_idx:={signal_idx(sample, 'bus_voltage')}",
        f"track_temperature_idx:={signal_idx(sample, 'fet_temperature')}",
    ]

    cmd = ["xacro", xacro_path, *xacro_args, "-o", output_path]
    subprocess.run(cmd, check=True)

# ─────────────────────────────────────────────
# Step 3: OpaqueFunction — wires everything together
# ─────────────────────────────────────────────

def launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory("roboguard_plots")
    xacro_path = os.path.join(pkg_dir, "config", "layout.xml.xacro")

    # Temp file for the expanded config (survives the launch session)
    tmp = tempfile.NamedTemporaryFile(
        suffix=".xml", prefix="pj_layout_", delete=False
    )
    output_path = tmp.name
    tmp.close()

    # 1 — introspect topic
    indices = get_dynamic_joint_indices(timeout_sec=5.0)

    # 2 — expand xacro
    run_xacro(xacro_path, indices, output_path)

    # 3 — launch PlotJuggler (or your node) with the generated layout
    plotjuggler = ExecuteProcess(
        cmd=[
            "ros2", "run", "plotjuggler", "plotjuggler",
            "--layout", output_path,
        ],
        output="screen",
    )

    return [plotjuggler]


# ─────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
    
