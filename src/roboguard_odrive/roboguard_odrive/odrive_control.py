import sys
sys.path.append(__file__.removesuffix(f"/{__file__.split('/')[-1]}"))

# std imports
import asyncio
import os
from typing import Dict, Iterable, List, Optional, Tuple

# 3rd party imports
import can

# ros imports
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.constants import S_TO_NS
from rclpy.duration import Duration
from rclpy.time import Time

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
from utils import dict2keyvalues, rad2rev, rev2rad, verifyLengthMatch, yesno



class ODriveControl(Node):
    def __init__(self):
        from rcl_interfaces.msg import ParameterDescriptor, ParameterType
        super().__init__('odrive_control')
        
        # Declare variables
        self.shuttingDown = False
        self._lastEnable = self.get_clock().now()
        self._enable = False
        self._lastEStop = self.get_clock().now()
        self._estop = True
        
        # Declare parameters
        self.channel = self.declare_parameter(
            'channel', 'can0', ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='Can channel'))
        self.bitrate = self.declare_parameter(
            'bitrate', 500000, ParameterDescriptor(type=ParameterType.PARAMETER_STRING, description='Can bitrate'))

        # Default values are required otherwise the type defaults to byte array (even if marqued as PARAMETER_STRING_ARRAY)
        self.jointNames = self.declare_parameter(
            'joint_names', [''], ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY))
        self.jointCanIDs = self.declare_parameter(
            'joint_can_ids', [0], ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY))
        self.publishRate: rclpy.Parameter = self.declare_parameter(
            'publish_rate', 20.0, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, description='Publish rate in Hz'))
        self.canWriteRate: rclpy.Parameter = self.declare_parameter(
            'can_write_rate', 10.0, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, description='Can write rate in Hz'))

        # Create subscriptions
        self.jointTrajectorySub = self.create_subscription(
            JointTrajectory, 'trajectory', self.onJointTrajectoryMsg, 1)
        self.jointJogSub = self.create_subscription(
        JointJog, 'jog', self.onJointJogMsg, 1)
        self.enableSub = self.create_subscription(Bool, 'enable', self.onEnableMsg, 1)
        self.estopSub = self.create_subscription(Bool, 'estop', self.onEStopMsg, 1)

        # Create publishers
        self.jointStatePub = self.create_publisher(
            JointState, 'joint_states', 1)
        self.diagnosticsPub = self.create_publisher(
            DiagnosticArray, 'diagnostics', 1)

        # Create timers
        self.publishTimer = self.create_timer(
            1.0 / self.publishRate.value, self.onPublishTimer)
        self.diagnosticTimer = self.create_timer(1.0, self.onDiagnosticTimer)
        
        # Release resources on exit
        self.create_guard_condition(self.onExit)

        # Setup can nodes
        self.canHandler = CanHandler(self.get_logger(), self.channel.value, self.bitrate.value, self._onCanError)
        # self.reader = can.AsyncBufferedReader()
        # self.notifier = can.Notifier(self.canHandler, [
        #     self.reader
        # ], 0.005)
        self.nodes: Dict[str, ODriveCanNode] = {
            name: ODriveCanNode(self.canHandler, nodeID)
                for name, nodeID in zip(self.jointNames.value, self.jointCanIDs.value)
        }
        # self.readTask = asyncio.get_running_loop().create_task(self.readLoop())
        for node in self.nodes.values():
            node.__enter__()
            asyncio.get_running_loop().create_task(node.read_loop())
        
        self.writeTask = asyncio.get_running_loop().create_task(self.writeLoop())
        
        self.posActions: Dict[str, Optional[Tuple[Time, float, float, float]]] = {name: None for name in self.nodes.keys()}
        self.velActions: Dict[str, Optional[Tuple[Time, float]]] = {name: None for name in self.nodes.keys()}
        
    @property
    def enable(self) -> bool:
        if self.shuttingDown or self.estop:
            return False
        now = self.get_clock().now()
        if (now - self._lastEnable) > Duration(nanoseconds=int(0.5 * S_TO_NS)):
            # self.get_logger().info("Too long since enable")
            return False
        return self._enable
    
    @enable.setter
    def enable(self, value: bool):
        self._lastEnable = self.get_clock().now()
        self._enable = value
    
    @property
    def estop(self) -> bool:
        if self.shuttingDown:
            return True
        now = self.get_clock().now()
        if (now - self._lastEStop) > Duration(nanoseconds=int(0.5 * S_TO_NS)):
            # self.get_logger().info("Too long since estop")
            return True
        return self._estop
    
    @estop.setter
    def estop(self, value: bool):
        self._lastEStop = self.get_clock().now()
        self._estop = value

    def onJointTrajectoryMsg(self, trajectory: JointTrajectory):
        # Position control
        if len(trajectory.points) == 0:
            self.get_logger().warning('Invalid trajectory, size 0')
            return
        point: JointTrajectoryPoint = trajectory.points[-1]
        if not verifyLengthMatch(trajectory.joint_names, point.positions, point.velocities, point.effort):
            self.get_logger().error('Invalid trajectory, size mismatch')
            return
        for name, position, velocity, effort in zip(
            trajectory.joint_names,
            point.positions,
            point.velocities,
            point.effort
        ):
            position = rad2rev(position)
            velocity = rad2rev(velocity)
            # self.nodes[name].set_controller_mode(ODriveControlMode.MODE_POSITION_CONTROL, ODriveInputMode.INPUT_POS_FILTER)
            # self.nodes[name].set_position(position, velocity, effort)
            self.posActions[name] = (Time.from_msg(trajectory.header.stamp), position, velocity, effort)

    def onJointJogMsg(self, jog: JointJog):
        # Velocity control
        if len(jog.joint_names) == 0:
            self.get_logger().warning('Invalid jog, size 0')
            return
        if not verifyLengthMatch(jog.joint_names, jog.velocities):
            self.get_logger().error('Invalid jof, size mismatch')
            return

        for name, velocity in zip(jog.joint_names, jog.velocities):
            velocity = rad2rev(velocity)

            # self.nodes[name].set_controller_mode(ODriveControlMode.MODE_VELOCITY_CONTROL, ODriveInputMode.INPUT_VEL_RAMP)
            # self.nodes[name].set_velocity(velocity)
            self.velActions[name] = (Time.from_msg(jog.header.stamp), velocity)

    def onEnableMsg(self, enable: Bool):
        self.enable = enable.data
        
    def onEStopMsg(self, estop: Bool):
        self.estop = estop.data

    def canDiagnostic(self) -> DiagnosticStatus:
        res = DiagnosticStatus()
        res.hardware_id = self.channel.value
        res.name = 'Can status'
        if self.canHandler.error == CanError.FATAL_ERROR:
            res.level = DiagnosticStatus.ERROR
            res.message = 'Fatal error, check configuration'
        elif self.canHandler.error == CanError.INIT_ERROR:
            res.level = DiagnosticStatus.ERROR
            res.message = 'Initialization error, is the can interface up?'
        elif self.canHandler.error == CanError.OPERATION_ERROR:
            res.level = DiagnosticStatus.ERROR
            res.message = 'Operation error, read or write, failed. Is the EStop button pressed?'
        elif self.canHandler.error == CanError.TIMEOUT:
            res.level = DiagnosticStatus.WARN
            res.message = 'Timed out'
        elif self.canHandler.error == CanError.NONE:
            if self.canHandler.status == CanStatus.INITIALIZED:
                res.level = DiagnosticStatus.OK
                res.message = 'Ok'
            elif self.canHandler.status == CanStatus.UNINITIALIZED:
                res.level = DiagnosticStatus.WARN
                res.message = 'Uninitialized'
            elif self.canHandler.status == CanStatus.SHUTDOWN:
                res.level = DiagnosticStatus.ERROR
                res.message = 'Bus has been shutdown, maybe the node is stopping'
            elif self.canHandler.status == CanStatus.ERROR:
                res.level = DiagnosticStatus.ERROR
                res.message = "This shouldn't happen, errors should be managed outside this block"
            else:
                res.level = DiagnosticStatus.ERROR
                res.message = f'Unhandled status, status code: {self.canHandler.status}'
        else:
            res.level = DiagnosticStatus.ERROR
            res.message = f'Unhandled error, error code: {self.canHandler.error}'

        values: Dict[str, str] = {
            'status': self.canHandler.status.name,
            'inner_state': self.canHandler.state.name,
            'channel_info': self.canHandler.channel_info,
        }

        res.values = dict2keyvalues(values)
        return res

    def _driveErrorMessage(self, node: ODriveCanNode) -> DiagnosticStatus:
        if not node.connected:
            return DiagnosticStatus(
                level=DiagnosticStatus.ERROR,
                message='Disconnected'
            )
        if node.error != 0:
            return DiagnosticStatus(
                level=DiagnosticStatus.ERROR,
                message=get_error_description(node.error)
            )
        return DiagnosticStatus(
            level=DiagnosticStatus.OK,
            message=node.state.name
        )

    def odriveNodeDiagnostic(self) -> Iterable[DiagnosticArray]:
        for name, node in self.nodes.items():
            res: DiagnosticStatus = self._driveErrorMessage(node)
            res.hardware_id = f'odrive_node_{node.node_id}'
            res.name = name

            values: Dict[str, str] = {
                'connected': yesno(node.connected),
                'position': f'{round(node.position, 3)} rev',
                'velocity': f'{round(node.velocity, 3)} rev/s',
                'effort': f'{round(node.torque, 3)} Nm',
                'voltage': f'{round(node.voltage, 3)} V',
                'current': f'{round(node.current, 3)} A',
                'fet_temperature': f'{round(node.fetTemperature, 3)} °C',
                'motor_temperature': f'{round(node.motorTemperature, 3)} °C',
                'disarm_reason': get_error_description(node.disarmReason),
                'trajectory_done': yesno(node.trajectoryDone),
                'procedure_result': node.procedureResult.name,
                'procedure_done': yesno(node.procedureDone),
            }

            res.values = dict2keyvalues(values)

            yield res

    def _onCanError(self, error: CanError):
        self.sendDiagnostics()

    def _getHeader(self) -> Header:
        return Header(frame_id=self.get_name(), stamp=self.get_clock().now().to_msg())

    def sendDiagnostics(self):
        status: List[DiagnosticStatus] = [
            self.canDiagnostic(),
            *self.odriveNodeDiagnostic()
        ]
        diagnostics = DiagnosticArray(header=self._getHeader(), status=status)
        self.diagnosticsPub.publish(diagnostics)

    def onPublishTimer(self):
        names: List[str] = []
        pos: List[float] = []
        vels: List[float] = []
        effs: List[float] = []
        for name, node in self.nodes.items():
            names.append(name)
            pos.append(rev2rad(node.position))
            vels.append(rev2rad(node.velocity))
            effs.append(node.torque)
        state = JointState(
            header=self._getHeader(),
            name=names,
            position=pos,
            velocity=vels,
            effort=effs
        )
        self.jointStatePub.publish(state)

    def onDiagnosticTimer(self):
        self.sendDiagnostics()
        
    # async def readLoop(self):
    #     async for msg in self.reader:
    #         for node in self.nodes.values():
    #             node.read_msg(msg)
                
    async def writeLoop(self):
        while True:
            for name, node in self.nodes.items():
                if self.estop:
                    node.call_estop()
                    self.get_logger().info("estop")
                else:
                    if self.enable:
                        # node.clear_errors_msg()
                        now = self.get_clock().now()
                        if self.posActions[name] is not None and (now - self.posActions[name][0] < Duration(seconds=1)):
                            node.set_state_msg(ODriveAxisState.CLOSED_LOOP_CONTROL)
                            node.set_position(*self.posActions[name][1:])
                            
                        elif self.velActions[name] is not None and (now - self.velActions[name][0] < Duration(seconds=1)):
                            node.set_state_msg(ODriveAxisState.CLOSED_LOOP_CONTROL)
                            node.set_velocity(self.velActions[name][1])
                        else:
                            node.set_state_msg(ODriveAxisState.IDLE)
                    else:
                        node.set_state_msg(ODriveAxisState.IDLE)
            # await asyncio.sleep(1.0/self.canWriteRate.value)
            # self.get_logger().info("Sending")
            await asyncio.sleep(0.1)
                    

    async def onExit(self):
        self.get_logger().info("Shutting down")
        self.shuttingDown = True
        self.estop = True
        self.enable = False
        self.get_logger().info("Calling EStop")
        
        for node in self.nodes.values():
            node.call_estop()
            node.__exit__(None, None, None)
        await asyncio.sleep(0.1)
        self.get_logger().info("Stopping tasks")
        self.writeTask.cancel()
        # self.readTask.cancel()
        # while self.readTask.executing() or self.writeTask.executing():
        #     await asyncio.sleep(0.1)
        self.get_logger().info("Tasks stopped")
        self.canHandler.shutdown()
        self.get_logger().info("Stopped successfully")

async def spinning(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        await asyncio.sleep(0.01)


async def run(args: List[str], loop: asyncio.BaseEventLoop):
    # init ROS 2
    rclpy.init(args=args)
    odrive_control = ODriveControl()
    spin_task = loop.create_task(spinning(odrive_control))

    try:
        await spin_task
    except asyncio.exceptions.CancelledError:
        pass
    rclpy.shutdown()

def main(args=None):
    try:
        loop = asyncio.get_event_loop()
        loop.run_until_complete(run(args, loop=loop))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

def is_root():
    return os.getuid() == 0

if __name__ == '__main__':
    main()
    