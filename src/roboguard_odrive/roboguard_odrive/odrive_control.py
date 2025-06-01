from __future__ import annotations

# std imports
import asyncio
from typing import Dict, Iterable, List

# ros imports
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# messages and services imports
from odrive_types import get_error_description
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_msgs.msg import Header

# local imports
from can_handler import CanError, CanHandler, CanStatus
from odrive_can_node import ODriveCanNode
from utils import dict2keyvalues, rad2rev, verifyLengthMatch, yesno


class ODriveControl(Node):
    def __init__(self):
        from rcl_interfaces.msg import ParameterDescriptor, ParameterType
        super().__init__('odrive_control')

        # Declare parameters
        self.channel = self.declare_parameter(
            'channel', 'can0', ParameterDescriptor(description='Can channel'))
        self.bitrate = self.declare_parameter(
            'bitrate', 500000, ParameterDescriptor(description='Can bitrate'))

        self.jointNames = self.declare_parameter(
            'joint_names', [], ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY))
        self.jointCanIDs = self.declare_parameter(
            'joint_can_ids', [], ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY))
        self.frequency: rclpy.Parameter = self.declare_parameter(
            'publish_rate', 20.0, ParameterDescriptor(description='Publish rate in Hz'),
            type=ParameterType.PARAMETER_DOUBLE)

        # Create subscriptions
        self.jointTrajectorySub = self.create_subscription(
            JointTrajectory, 'trajectory', self.onJointTrajectoryMsg, 1)
        self.jointJogSub = self.create_subscription(
            JointJog, 'jog', self.onJointJogMsg, 1)

        # Create publishers
        self.jointStatePub = self.create_publisher(
            JointState, 'joint_states', 1)
        self.diagnosticsPub = self.create_publisher(
            DiagnosticArray, 'diagnostics', 1)

        # Create timer
        self.publishTimer = self.create_timer(
            1.0 / self.frequency.value, self.onPublishTimer)
        self.diagnosticTimer = self.create_timer(1.0, self.onDiagnosticTimer)

        # Setup can nodes
        self.canHandler = CanHandler(self.channel.value, self.bitrate.value)
        self.nodes: Dict[str, ODriveCanNode] = {
            name: ODriveCanNode(self.canHandler, nodeID)
                for name, nodeID in zip(self.jointNames.value, self.jointCanIDs.value)
        }

    async def onJointTrajectoryMsg(self, trajectory: JointTrajectory):
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
            self.nodes[name].set_position(position, velocity, effort)

    async def onJointJogMsg(self, jog: JointJog) -> ODriveControl:
        # Velocity control
        if len(jog.joint_names) == 0:
            self.get_logger().warning('Invalid jog, size 0')
            return
        if not verifyLengthMatch(jog.joint_names, jog.velocities):
            self.get_logger().error('Invalid jof, size mismatch')
            return

        for name, velocity in zip(jog.joint_names, jog.velocities):
            velocity = rad2rev(velocity)

            self.nodes[name].set_velocity(velocity)

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
        return Header(frame_id=self.get_name(), stamp=self.get_clock().now())

    def sendDiagnostics(self):
        status: List[DiagnosticStatus] = [
            self.canDiagnostic(),
            *self.odriveNodeDiagnostic()
        ]
        diagnostics = DiagnosticArray(header=self._getHeader(), status=status)
        self.diagnosticsPub.publish(diagnostics)

    async def onPublishTimer(self):
        pass

    async def onDiagnosticTimer(self):
        self.sendDiagnostics()

    def __enter__(self) -> ODriveControl:
        for n in self.nodes.values():
            n.__enter__()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        for n in self.nodes.values():
            n.__exit__(exc_type, exc_val, exc_tb)
        self.canHandler.shutdown()


async def spinning(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        await asyncio.sleep(0.001)


async def run(args: List[str], loop: asyncio.BaseEventLoop):
    # init ROS 2
    with rclpy.init(args=args):
        odrive_control = ODriveControl()
        spin_task = loop.create_task(spinning(odrive_control))

        try:
            await spin_task
        except asyncio.exceptions.CancelledError:
            pass


def main(args=None):
    try:
        loop = asyncio.get_event_loop()
        loop.run_until_complete(run(args, loop=loop))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
