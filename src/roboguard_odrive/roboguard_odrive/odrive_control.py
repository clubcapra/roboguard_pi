from datetime import datetime, timedelta
from statistics import covariance, mean, median
import statistics
import sys
sys.path.append(__file__.removesuffix(f"/{__file__.split('/')[-1]}"))

# std imports
from time import sleep
import asyncio
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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_msgs.msg import Header, Bool

# local imports
from odrive_types import ODriveAxisState, ODriveControlMode, ODriveInputMode, get_error_description
from can_handler import CanError, CanHandler, CanStatus
from odrive_can_node import ODriveCanNode
from utils import dict2keyvalues, rad2rev, rev2rad, verifyLengthMatch, yesno

class RollingBuffer:
    def __init__(self, capacity:int):
        self.capacity = capacity
        self._len = 0
        self.reset()
        self._index = 0
        
    def reset(self):
        self._data = list([0.0 for _ in range(self.capacity)])
        
    def push(self, value: float):
        self._data[self._index] = value
        self._len = max(self._index + 1, self._len)
        self._index = (self._index + 1) % self.capacity
        
    def mean(self) -> float:
        return mean(self._data[:self._len])
    
    def min(self) -> float:
        return min(self._data[:self._len])
    
    def max(self) -> float:
        return max(self._data[:self._len])
    
    def median(self) -> float:
        return median(self._data[:self._len])
    
    def variance(self) -> float:
        return statistics.variance(self._data[:self._len])

def niceFloat(value: float, numDigits: int = 3, just:int = 6) -> str:
    return str(round(value, numDigits)).ljust(just)

class ODriveControl(Node):
    def __init__(self, loop: asyncio.AbstractEventLoop):
        from rcl_interfaces.msg import ParameterDescriptor, ParameterType
        super().__init__('odrive_control')
        
        # Declare variables
        self.loop = loop
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

        joints = [
            "flipper_fl_j",
            "flipper_rl_j",
            "flipper_fr_j",
            "flipper_rr_j",
            "track_fl_j",
            "track_rl_j",
            "track_fr_j",
            "track_rr_j",
        ]
        joint_ids = [
            11,
            12,
            13,
            14,
            21,
            22,
            23,
            24,
        ]
        continuousCurrents = [
            6.0,
            6.0,
            6.0,
            6.0,
            8.0,
            8.0,
            8.0,
            8.0,
        ]
        self.jointNames = self.declare_parameter(
            'joint_names', joints, ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY))
        self.jointCanIDs = self.declare_parameter(
            'joint_can_ids', joint_ids, ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY))
        self.continuousCurrentLimits = self.declare_parameter(
            'continuous_current_limits', continuousCurrents, ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY))
        self.currentLimitTime: rclpy.Parameter = self.declare_parameter(
            'peak_current_time', 0.1, ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, description='Max time to be over continuous current'))
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
        
        # Setup can nodes
        self.canHandler = CanHandler(self.get_logger(), self.channel.value, self.bitrate.value, self._onCanError)
        self.nodes: Dict[str, ODriveCanNode] = {
            name: ODriveCanNode(
                    self.canHandler,
                    nodeID,
                    self.loop,
                    currentLimit,
                    self.currentLimitTime.value,
                    self._onDriveStatus
                )
                for name, nodeID, currentLimit in zip(self.jointNames.value, self.jointCanIDs.value, self.continuousCurrentLimits.value)
        }
        
        self.reader = can.AsyncBufferedReader()
            
        self.writeTask = self.loop.create_task(self.writeLoop())
        self.readTask = self.loop.create_task(self.readLoop())
        
        self.posActions: Dict[str, Optional[Tuple[Time, float, float, float]]] = {name: None for name in self.nodes.keys()}
        self.velActions: Dict[str, Optional[Tuple[Time, float]]] = {name: None for name in self.nodes.keys()}
        self.writeLoopStats = RollingBuffer(100)
        
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
            # self.posActions[name] = (Time.from_msg(trajectory.header.stamp), position, velocity, effort)
            self.posActions[name] = (Time.from_msg(self._getHeader().stamp), position, velocity, effort)

    def onJointJogMsg(self, jog: JointJog):
        # Velocity control
        if len(jog.joint_names) == 0:
            self.get_logger().warning('Invalid jog, size 0')
            return
        if not verifyLengthMatch(jog.joint_names, jog.velocities):
            self.get_logger().error('Invalid jog, size mismatch')
            return

        for name, velocity in zip(jog.joint_names, jog.velocities):
            velocity = rad2rev(velocity)

            # self.velActions[name] = (Time.from_msg(jog.header.stamp), velocity)
            self.velActions[name] = (Time.from_msg(self._getHeader().stamp), velocity)

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

    def writeLoopDiagnostic(self) -> DiagnosticStatus:
        res = DiagnosticStatus()
        res.hardware_id = 'write_loop'
        res.name = 'Write loop stats'
        errorThresh = 2 * (1 / self.canWriteRate.value)
        warnThresh = 1 / self.canWriteRate.value
        
        if self.writeLoopStats.max() > errorThresh:
            res.level = DiagnosticStatus.ERROR
        elif self.writeLoopStats.max() > warnThresh:
            res.level = DiagnosticStatus.WARN
        else:
            res.level = DiagnosticStatus.OK
            
        values = {
            'avg': self.writeLoopStats.mean(),
            'min': self.writeLoopStats.min(),
            'max': self.writeLoopStats.max(),
            'median': self.writeLoopStats.mean(),
            'var': self.writeLoopStats.variance(),
        }
        
        res.message = ' '.join([f'{name}: {niceFloat(value)}' for name, value in values.items()])
        res.values = dict2keyvalues({k: str(v) for k, v in values.items()})
        return res

    def _driveErrorMessage(self, node: ODriveCanNode) -> DiagnosticStatus:
        if not node.connected:
            return DiagnosticStatus(
                level=DiagnosticStatus.ERROR,
                message='Disconnected'
            )
        if node.currentPeakError:
            return DiagnosticStatus(
                level=DiagnosticStatus.ERROR,
                message="PEAK CURRENT REACHED"
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
                'state': node.state.name,
                'current_peak_error': yesno(node.currentPeakError),
                'error': get_error_description(node.error),
                'disarm_reason': get_error_description(node.disarmReason),
                'position': f'{round(node.position, 3)}',
                'velocity': f'{round(node.velocity, 3)}',
                'effort': f'{round(node.torque, 3)}',
                'voltage': f'{round(node.voltage, 3)}',
                'current': f'{round(node.current, 3)}',
                'fet_temperature': f'{round(node.fetTemperature, 3)}',
                'motor_temperature': f'{round(node.motorTemperature, 3)}',
                'trajectory_done': yesno(node.trajectoryDone),
                'procedure_result': node.procedureResult.name,
                'procedure_done': yesno(node.procedureDone),
            }

            res.values = dict2keyvalues(values)

            yield res

    def _onCanError(self, error: CanError):
        self.sendDiagnostics()
        
    def _onDriveStatus(self):
        self.sendDiagnostics()

    def _getHeader(self) -> Header:
        return Header(frame_id=self.get_name(), stamp=self.get_clock().now().to_msg())

    def sendDiagnostics(self):
        status: List[DiagnosticStatus] = [
            self.canDiagnostic(),
            self.writeLoopDiagnostic(),
            *self.odriveNodeDiagnostic(),
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
        
    async def _readLoop(self, node: ODriveCanNode):
        while True:
            async with node:
                await node.read_loop()
        
    async def readLoop(self):
        notifier = can.Notifier(
            self.canHandler,
            [self.reader],
            loop = asyncio.get_running_loop(),
        )
        async for msg in self.reader:
            for n in self.nodes.values():
                n.read_msg(msg)
        notifier.stop()
    
    async def writeLoop(self):
        posTimedOut = {name: True for name in self.nodes.keys()}
        velTimedOut = {name: True for name in self.nodes.keys()}
        lastOkTime = {name: datetime.now() for name in self.nodes.keys()}
        nextRun = datetime.now() + timedelta(seconds=1/self.canWriteRate.value)
        
        while True:
            try:
                startTime = datetime.now()
                for name, node in self.nodes.items():
                    if node.error != 0:
                        self.get_logger().info(f"Node: {name} {node.error.name}")
                        if lastOkTime[name] + timedelta(seconds=3) > startTime:
                            self.canHandler.restartCan()
                        node.clear_errors_msg()
                    else:
                        lastOkTime[name] = startTime
                    if self.estop or node.currentPeakError:
                        node.call_estop()
                        node.feed_watchdog_msg()
                    else:
                        if self.enable:
                            now = self.get_clock().now()
                            nowt = datetime.now()
                            posAction = self.posActions[name]
                            velAction = self.velActions[name]
                            posTO = False if posAction is None else (now - posAction[0] > Duration(nanoseconds=0.5*S_TO_NS))
                            velTO = False if velAction is None else (now - velAction[0] > Duration(nanoseconds=0.5*S_TO_NS))
                            if posTO and not posTimedOut[name]:
                                self.get_logger().warn(f"Node: {name} timed out pos")
                            if velTO and not velTimedOut[name]:
                                self.get_logger().warn(f"Node: {name} timed out vel")
                            posTimedOut[name] = posTO
                            velTimedOut[name] = velTO
                            if posAction is not None and not posTO and abs(posAction[1] - node.position) > 0.1:
                                if node.state != ODriveAxisState.CLOSED_LOOP_CONTROL:
                                    self.get_logger().info(f"Setting {name} to position control")
                                    node.set_controller_mode(ODriveControlMode.MODE_POSITION_CONTROL, ODriveInputMode.INPUT_POS_FILTER)
                                    node.set_state_msg(ODriveAxisState.CLOSED_LOOP_CONTROL)
                                node.set_position(posAction[1], 0, posAction[3])
                                    
                            elif velAction is not None and not velTO:
                                if node.state != ODriveAxisState.CLOSED_LOOP_CONTROL:
                                    self.get_logger().info(f"Setting {name} to velocity control")
                                    node.set_controller_mode(ODriveControlMode.MODE_VELOCITY_CONTROL, ODriveInputMode.INPUT_VEL_RAMP)
                                    node.set_state_msg(ODriveAxisState.CLOSED_LOOP_CONTROL)
                                node.set_velocity(velAction[1])
                            else:
                                if node.state != ODriveAxisState.IDLE:
                                    node.set_state_msg(ODriveAxisState.IDLE)
                                node.feed_watchdog_msg()
                        else:
                            if node.state != ODriveAxisState.IDLE:
                                node.set_state_msg(ODriveAxisState.IDLE)
                            node.feed_watchdog_msg()
                newNow = datetime.now()
                runtime = newNow - startTime
                self.writeLoopStats.push(runtime.total_seconds())
                if newNow >= nextRun:
                    self.get_logger().warning(f"Write loop is behind schedule by {round((nextRun - newNow).total_seconds(), 4)}s")
                    nextRun = newNow + timedelta(seconds=1/self.canWriteRate.value)
                else:
                    delay = nextRun - newNow
                    nextRun = nextRun + timedelta(seconds=1/self.canWriteRate.value)
                    await asyncio.sleep(delay.total_seconds())
            except asyncio.CancelledError:
                return
            except Exception as e:
                self.get_logger().error(f"Loop error: {e}")

    def stop(self):
        self.get_logger().info("Shutting down")
        self.shuttingDown = True
        self.estop = True
        self.enable = False
        self.get_logger().info("Calling EStop")
        
        for node in self.nodes.values():
            node.call_estop()
        sleep(0.1)
        self.get_logger().info("Stopping tasks")
        self.writeTask.cancel()
        self.get_logger().info("Tasks stopped")
        self.canHandler.shutdown()
        self.get_logger().info("Stopped successfully")

async def spinning(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        await asyncio.sleep(0.01)


async def run(odrive_control: ODriveControl, loop: asyncio.BaseEventLoop):
    spin_task = loop.create_task(spinning(odrive_control))

    try:
        await spin_task
    except asyncio.exceptions.CancelledError:
        pass

def main(args=None):
    try:
        rclpy.init(args=args)
        loop = asyncio.get_event_loop()
        odrive_control = ODriveControl(loop)
        loop.run_until_complete(run(odrive_control, loop=loop))
    except (KeyboardInterrupt, ExternalShutdownException):
        print("Stopped")
    finally:
        odrive_control.stop()
        loop.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
