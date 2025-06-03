import sys
sys.path.append(__file__.removesuffix(f"/{__file__.split('/')[-1]}"))

# std imports
import asyncio
import os
from typing import Dict, Iterable, List

# ros imports
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.constants import S_TO_NS
from rclpy.duration import Duration

# messages and services imports
from capra_control_msgs.msg import Flippers, Tracks
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Bool

# local imports
from utils import bounce, rad2rev, rev2rad


class ActionsPub(Node):
    def __init__(self):
        from rcl_interfaces.msg import ParameterDescriptor, ParameterType
        super().__init__('actions_pub')
        
        self.tracksPub = self.create_publisher(
            Tracks, 'tracks_cmd', 1)
        self.flippersPub = self.create_publisher(
            Flippers, 'flippers_cmd', 1)
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
        
        vel = bounce(-1, 1, (self.i / self.cycle))
        
        self.tracksPub.publish(Tracks(left=vel, right=-vel))
        
        self.i = (self.i + 1) % self.cycle
        
def main(args=None):
    rclpy.init()
    node = ActionsPub()
    rclpy.spin(node)
        
if __name__ == "__main__":
    main()