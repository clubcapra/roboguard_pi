import sys
sys.path.append(__file__.removesuffix(f"/{__file__.split('/')[-1]}"))

# std imports
import asyncio
import os
from typing import Dict, Iterable, List

# 3rd party imports
import can

# ros imports
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.constants import S_TO_NS
from rclpy.duration import Duration

# messages and services imports
from odrive_types import ODriveAxisState, ODriveControlMode, ODriveInputMode, get_error_description
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_msgs.msg import Header, Bool

# local imports
from can_handler import CanError, CanHandler, CanStatus
from odrive_can_node import ODriveCanNode
from utils import dict2keyvalues, rad2rev, rev2rad, verifyLengthMatch, yesno, bounce


class ODrivePub(Node):
    def __init__(self):
        from rcl_interfaces.msg import ParameterDescriptor, ParameterType
        super().__init__('odrive_pub')
        
        self.jointTrajectoryPub = self.create_publisher(
            JointTrajectory, 'trajectory', 1)
        self.jointJogPub = self.create_publisher(
        JointJog, 'jog', 1)
        self.enablePub = self.create_publisher(Bool, 'enable', 1)
        self.estopPub = self.create_publisher(Bool, 'estop', 1)
        
        self.timer = self.create_timer(0.1, self.onTimer)
        self.i = 0
        self.cycle = 100
        
    def _getHeader(self) -> Header:
        return Header(frame_id=self.get_name(), stamp=self.get_clock().now().to_msg())
        
    def onTimer(self):
        self.enablePub.publish(Bool(data=True))
        self.estopPub.publish(Bool(data=False))
        
        vels = [rev2rad(bounce(10, 30, (self.i / self.cycle) + offset / 4)) for offset in range(4)]
        
        self.jointJogPub.publish(
            JointJog(
                header=self._getHeader(),
                joint_names=['track_fl_j', 'track_rl_j', 'track_fr_j', 'track_rr_j'],
                displacements=[0.0],
                velocities=[*vels]
            )
        )
        self.i = (self.i + 1) % self.cycle
        
if __name__ == "__main__":
    rclpy.init()
    node = ODrivePub()
    rclpy.spin(node)