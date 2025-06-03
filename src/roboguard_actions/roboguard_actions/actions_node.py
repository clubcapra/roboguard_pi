import json
from statistics import mean
import sys
sys.path.append(__file__.removesuffix(f"/{__file__.split('/')[-1]}"))

# std imports
from typing import Dict, Generic, List, Optional, TypeVar
from pathlib import Path

# ros imports
import rclpy
from rclpy.node import Node
from rclpy.constants import S_TO_NS
from rclpy.duration import Duration
from rclpy.time import Time

# messages and services imports
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from std_msgs.msg import Header, Bool
from capra_control_msgs.msg import Flippers, Tracks

# local imports
from utils import clamp, rad2rev, rev2rad, sign

FLIPPER_MOVE_OFFSET = rev2rad(50)

class StateBool:
    """A boolean container that tracks latching and unlatching (off->on and on->off respectively) state changes
    """
    def __init__(self, value:bool = False):
        self._state = value
        self._lastState = value
    
    @property
    def state(self) -> bool:
        return self._state
    
    @state.setter
    def state(self, value:bool):
        self._lastState = self._state
        self._state = value
        
    @property
    def latched(self) -> bool:
        return not self._lastState and self._state
    
    @property
    def unlatched(self) -> bool:
        return self._lastState and not self._state
        
    def update(self):
        self._lastState = self._state

_K = TypeVar('_K')
_V = TypeVar('_V')

class ValueRef(Generic[_K, _V]):
    def __init__(self, _dict: Dict[_K, _V], key: _K):
        self._dict = _dict
        self._key = key
        
    def __call__(self) -> _V:
        return self._dict[self._key]

class Instruction:
    def __init__(self, offsets: List[ValueRef[str, float]], setOffsets: List[ValueRef[str, float]]):
        self.offsets = offsets
        self.setOffsets = setOffsets
        self.offset: float = 0.0
        self.setOffset: float = 0.0
        self.hasAction = False
        
    def update(self): ...
    
    def compute(self, command: float, enabled: bool): ...
    

class SingleInstruction(Instruction):
    def __init__(self, offset: ValueRef[str, float], setOffset: ValueRef[str, float]):
        super().__init__([offset], [setOffset])
        self.singleControl = StateBool()
        self.initial: float = 0.0
        
    def update(self):
        self.offset = self.initial + self.offsets[0]() - (self.setOffsets[0]() - self.setOffset)
        
    def compute(self, command: float, enabled: float):
        self.singleControl.state = command != 0 and enabled
        
        if self.singleControl.state:
            self.setOffset = self.offset + sign(command) * FLIPPER_MOVE_OFFSET
            self.hasAction = True
        elif self.singleControl.unlatched:
            self.setOffset = self.offset
            self.hasAction = True
        else:
            self.hasAction = False
        
class PairInstruction(Instruction):
    def __init__(self, offsets: List[ValueRef[str, float]], setOffsets: List[ValueRef[str, float]]):
        super().__init__(offsets, setOffsets)
        self.pairControl = StateBool()
        
    def update(self):
        self.offset = mean([o() - (s() - self.setOffset) for o, s in zip(self.offsets, self.setOffsets)])
        
    def compute(self, command: float, enabled: bool):
        self.pairControl.state = command != 0 and enabled
        
        if self.pairControl.state:
            self.setOffset = self.offset + sign(command) * FLIPPER_MOVE_OFFSET
            self.hasAction = True
        elif self.pairControl.unlatched:
            self.setOffset = self.offset
            self.hasAction = True
        else:
            self.hasAction = True
            
class AllInstruction(Instruction):
    def __init__(self, offsets: List[ValueRef[str, float]], setOffsets: List[ValueRef[str, float]]):
        super().__init__(offsets, setOffsets)
        self.allControl = StateBool()

    def update(self):
        self.offset = mean([o() - (s() - self.setOffset) for o, s in zip(self.offsets, self.setOffsets)])
        
    def compute(self, command: float, enabled: bool):
        self.allControl.state = command != 0 and enabled
        
        if self.allControl.state:
            self.setOffset = self.offset + sign(command) * FLIPPER_MOVE_OFFSET
            self.hasAction = True
        elif self.allControl.unlatched:
            self.setOffset = self.offset
            self.hasAction = True
        else:
            self.hasAction = True

class ActionsNode(Node):
    def __init__(self):
        from rcl_interfaces.msg import ParameterDescriptor, ParameterType
        super().__init__('roboguard_actions')
        
        # Declare variables
        self.enable = False
        self.flipperPos: Dict[str, float] = {}
        self.flipperSetPos: Dict[str, float] = {}
        
        # Declare parameters
        self.leftJointNames = self.declare_parameter(
            'tracks.left_joint_names',
            ['track_fl_j', 'track_rl_j'],
            ParameterDescriptor(description="Left track joints")
        )
        self.rightJointNames = self.declare_parameter(
            'tracks.right_joint_names',
            ['track_fr_j', 'track_rr_j'],
            ParameterDescriptor(description="Right track joints")
        )
        self.wheelRadius = self.declare_parameter(
            'tracks.wheel_radius',
            0.1075,
            ParameterDescriptor(description="Contact surface radius in meters")
        )
        self.wheelSeparation = self.declare_parameter(
            'tracks.wheel_separation',
            0.175,
            ParameterDescriptor(description="Distance between contact points")
        )
        self.tracksGearRatio = self.declare_parameter(
            'tracks.gear_ratio',
            1/30,
            ParameterDescriptor(description="Gearbox ratio for the tracks")
        )
        self.tracksMaxSpeed = self.declare_parameter(
            'tracks.max_speed',
            58,
            ParameterDescriptor(description="Max motor speed in rev/s")
        )
        
        self.flipperFrontLeft = self.declare_parameter('flippers.front_left_name', 'flipper_fl_j')
        self.flipperRearLeft = self.declare_parameter('flippers.rear_left_name', 'flipper_rl_j')
        self.flipperFrontRight = self.declare_parameter('flippers.front_right_name', 'flipper_fr_j')
        self.flipperRearRight = self.declare_parameter('flippers.rear_right_name', 'flipper_rr_j')
        self.flipperMaxSpeed = self.declare_parameter('flippers.max_speed', 50.0)
        self.flipperRatio = self.declare_parameter('flippers.gear_ratio', 540/1)
        
        # Create subscriptions
        self.enableSub = self.create_subscription(Bool, 'enable', self.onEnable)
        self.tracksSub = self.create_subscription(Tracks, 'tracks_cmd', self.onTracks)
        self.flipperSub = self.create_subscription(Flippers, 'flippers_cmd', self.onFlippers)
        self.jointStateSub = self.create_subscription(JointState, 'joint_states', self.onJointStates)
        
        # Create publishers
        self.jointTrajectoryPub = self.create_publisher(JointTrajectory, 'trajectory', 1)
        self.jointJogPub = self.create_publisher(JointJog, 'jog', 1)
        
        self.posToJoint = {
            'front_right': self.flipperFrontRight.value,
            'rear_right': self.flipperRearRight.value,
            'front_left': self.flipperFrontLeft.value,
            'rear_left': self.flipperRearLeft.value,
        }
        
        self.jointToPos = {pos: joint for joint, pos in self.posToJoint.items()}
        
        # Assign flippers
        self.flipperPos = {name: None for name in self.posToJoint.keys()}
        
        self.flipperSetPos = {name: None for name in self.posToJoint.keys()}
        
        # Assign instructions
        self.flipperSingleInstructions: Dict[str, SingleInstruction] = {
            name: SingleInstruction(
                ValueRef(self.flipperPos, name),
                ValueRef(self.flipperSetPos, name)) 
            for name in self.posToJoint.keys()}
        self.frontPairInstruction = PairInstruction(
            [
                ValueRef(self.flipperPos, 'front_left'),
                ValueRef(self.flipperPos, 'front_right'),
            ],
            [
                ValueRef(self.flipperSetPos, 'front_left'),
                ValueRef(self.flipperSetPos, 'front_right'),
            ]
        )
        self.rearPairInstruction = PairInstruction(
            [
                ValueRef(self.flipperPos, 'rear_left'),
                ValueRef(self.flipperPos, 'rear_right'),
            ],
            [
                ValueRef(self.flipperSetPos, 'rear_left'),
                ValueRef(self.flipperSetPos, 'rear_right'),
            ]
        )
        self.allPairInstruction = AllInstruction(
            [ValueRef(self.flipperPos, name) for name in self.posToJoint.keys()],
            [ValueRef(self.flipperSetPos, name) for name in self.posToJoint.keys()]
        )
        
    def _getHeader(self) -> Header:
        return Header(frame_id=self.get_name(), stamp=self.get_clock().now().to_msg())
        
    def onEnable(self, enable: Bool):
        # Here, enable is not time critical, roboguard_odrive will read the enable topic as well
        # It will determine if the enable command is valid
        self.enable = enable.data
    
    def onTracks(self, tracksCmd: Tracks):
        res = JointJog(header=self._getHeader())
        joints = []
        vels = []
        maxSpeed = rev2rad(self.tracksMaxSpeed.value)
        for name in self.leftJointNames.value:
            joints.append(name)
            vels.append(clamp(-1, 1, tracksCmd.left) * maxSpeed)
        for name in self.rightJointNames.value:
            joints.append(name)
            vels.append(clamp(-1, 1, tracksCmd.right) * maxSpeed)
            
        res.displacements = list([0 for _ in joints])
        res.joint_names = joints
        res.velocities = vels
        res.duration = Duration(seconds=1)
        
        self.jointJogPub.publish(res)
    
    def updateInstructions(self):
        instructions: List[Instruction] = [
            *self.flipperSingleInstructions.values(),
            self.frontPairInstruction,
            self.rearPairInstruction,
            self.allPairInstruction,
        ]
        for i in instructions:
            i.update()
            
    def onFlippers(self, flippersCmd: Flippers):
        self.updateInstructions()
        commands: Dict[str, float] = {
            'front_left': clamp(-1, 1, flippersCmd.front_left),
            'rear_left': clamp(-1, 1, flippersCmd.rear_left),
            'front_right': clamp(-1, 1, flippersCmd.front_right),
            'rear_right': clamp(-1, 1, flippersCmd.rear_right),
        }
        allSelected = all([v != 0.0 for v in commands.values()])
        frontSelected = not allSelected and all([v != 0 for k, v in filter(lambda x: 'front' in x[0], commands.items())])
        rearSelected = not allSelected and all([v != 0 for k, v in filter(lambda x: 'rear' in x[0], commands.items())])
        
        singleParams = {name: False for name in commands.keys()}
        
        if not allSelected:
            for name, cmd in commands.items():
                if 'front' in name and not frontSelected:
                    singleParams[name] = self.enable
                if 'rear' in name and not rearSelected:
                    singleParams[name] = self.enable
        
        for name, cmd in commands:
            self.flipperSingleInstructions[name].compute(cmd, singleParams[name])
        
        self.frontPairInstruction.compute(mean([commands['front_left'], commands['front_right']]), self.enable and frontSelected)
        self.rearPairInstruction.compute(mean([commands['rear_left'], commands['rear_right']]), self.enable and rearSelected)
        self.allPairInstruction.compute(mean(commands.values()), self.enable and allSelected)
        
        active = {
            name: (
                self.allPairInstruction.hasAction or
                ('front' in name and self.frontPairInstruction.hasAction) or
                ('rear' in name and self.rearPairInstruction.hasAction) or
                self.flipperSingleInstructions[name].hasAction
            ) 
            for name in commands.keys()
        }
        
        setPoints = {
            name: (
                self.flipperSingleInstructions[name].setOffset + 
                (self.frontPairInstruction.setOffset if 'front' in name else self.rearPairInstruction.setOffset) +
                self.allPairInstruction.setOffset
            )
        }
        
        traj = JointTrajectory(header=self._getHeader())
        point = JointTrajectoryPoint()
        
        for name, act in active.items():
            if act:
                traj.joint_names.append(self.posToJoint[name])
                point.positions.append(setPoints[name])
                point.velocities.append(rev2rad(self.flipperMaxSpeed.value * abs(commands[name])))
                point.accelerations.append(0)
                point.effort.append(0)
        
        traj.points.append(point)
        
        self.jointTrajectoryPub.publish(traj)
        
    def onJointStates(self, states: JointState):
        for joint, pos in zip(states.name, states.position):
            if joint in self.jointToPos.keys():
                # joint is a flipper
                
                name = self.jointToPos[joint]
                
                if self.flipperPos[name] is None:
                    # First feedback so we initialize offsets
                    self.flipperSetPos[name] = pos
                    self.flipperSingleInstructions[name].initial = pos
                self.flipperPos[name] = pos
                
        
def main(args=None):
    pass

if __name__ == '__main__':
    main()