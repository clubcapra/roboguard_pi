from __future__ import annotations
import asyncio
from datetime import datetime, timedelta
import can
import struct

from typing import Any, Callable, Optional
from odrive_types import ODriveAxisState, ODriveCommand, ODriveControlMode, ODriveEndpoints
from odrive_types import ODriveInputMode, ODriveProcedureResult, ODriveErrorCodes, get_error_description
import rclpy
import rclpy.logging
from can_handler import CanHandler


class ODriveCanNode:
    def __init__(
            self,
            handler: CanHandler,
            node_id: int,
            loop: asyncio.AbstractEventLoop,
            continuousCurrentLimit: float,
            peakCurrentTime: float,
            onStatusChangedCb: Optional[Callable]):
        self.handler: CanHandler = handler
        self.node_id: int = node_id
        self.loop = loop
        self.continuousCurrentLimit = continuousCurrentLimit
        self.peakCurrentTime = peakCurrentTime
        self.onStatusChangedCb = onStatusChangedCb
        self.reader: can.AsyncBufferedReader = can.AsyncBufferedReader()
        self.position = 0.0
        self.velocity = 0.0
        self.torque = 0.0
        self._error: ODriveErrorCodes = 0
        self._lastError: ODriveErrorCodes = 0
        self.disarmReason: ODriveErrorCodes = 0
        self._state: ODriveAxisState = ODriveAxisState.IDLE
        self.current = 0.0
        self.voltage = 0.0
        self.fetTemperature = 0.0
        self.motorTemperature = 0.0
        self.procedureResult: ODriveProcedureResult = ODriveProcedureResult.SUCCESS
        self.trajectoryDone = False
        self.procedureDone = False
        self._setpoint = 0.0
        self._nextPrint = datetime.now()
        self.logger = rclpy.logging.get_logger(
            'odrive_control').get_child(f'ODriveCanNode{self.node_id}')
        self._recursive = False
        self._lastMessage = datetime.min
        self._peakCurrentStart = datetime.min
        self.currentPeakError = False

    @property
    def connected(self) -> bool:
        return (datetime.now() - self._lastMessage).total_seconds() <= 1

    def onStatusChanged(self):
        recursing = self._recursive
        self._recursive = True
        if not recursing:
            if self.onStatusChangedCb is not None:
                self.onStatusChangedCb()
        
    @property
    def error(self) -> ODriveErrorCodes:
        return self._error
    
    @error.setter
    def error(self, error: ODriveErrorCodes):
        lastError = self.error
        self._error = error
        if self._error != lastError:
            self.onStatusChanged()

    @property
    def state(self) -> ODriveAxisState:
        return self._state
    
    @state.setter
    def state(self, state: ODriveAxisState):
        lastState = self._state
        self._state = state
        if self._state != lastState:
            self.onStatusChanged()

    @property
    def bus(self) -> can.BusABC:
        return self.handler

    async def __aenter__(self) -> ODriveCanNode:
        while self.bus is None:
            self.logger.error("Bus closed retrying in 1 second")
            await asyncio.sleep(1)
        self.notifier = can.Notifier(
            self.bus,
            [self.reader],
            loop=self.loop
        )
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        self.notifier.stop()

    def flush_rx(self) -> None:
        while not self.reader.buffer.empty():
            self.reader.buffer.get_nowait()

    async def read_loop(self):
        async for msg in self.reader:
            self.read_msg(msg)
        self.logger.error("Exitted read_loop")

    def read_msg(self, msg: can.Message):
        nid = ((msg.arbitration_id & (0b11111 << 5)) >> 5)
        if nid != self.node_id:
            return
        self._lastMessage = datetime.now()
        cmd_id = msg.arbitration_id & 0b11111
        if cmd_id == ODriveCommand.GET_ENCODER_ESTIMATES_CMD:
            self.position, self.velocity = struct.unpack('<ff', msg.data)
        elif cmd_id == ODriveCommand.GET_BUS_VOLTAGE_CURRENT_CMD:
            self.voltage, self.current = struct.unpack('<ff', msg.data)
            if self.current >= self.continuousCurrentLimit:
                if (datetime.now() - self._peakCurrentStart) >= timedelta(seconds=self.peakCurrentTime):
                    if not self.currentPeakError:
                        self.logger.fatal("PEAK CURRENT REACHED!!!")
                    self.currentPeakError = True
            else:
                self._peakCurrentStart = datetime.now()
        elif cmd_id == ODriveCommand.GET_TEMPERATURE_CMD:
            self.fetTemperature, self.motorTemperature = struct.unpack(
                '<ff', msg.data)
        elif cmd_id == ODriveCommand.GET_TORQUES_CMD:
            _, self.torque = struct.unpack('<ff', msg.data)
        elif cmd_id == ODriveCommand.GET_ERROR_CMD:
            error, disarmReason = struct.unpack('<II', msg.data)
            self.error = ODriveErrorCodes(error)
            self.disarmReason = ODriveErrorCodes(disarmReason)
            if self.error != 0:
                self.getErrorDescription(self.error)
        elif cmd_id == ODriveCommand.HEARTBEAT_CMD:
            error, state, procedureDone, trajectoryDone = struct.unpack(
                '<IBBBx', msg.data)
            self.error = ODriveErrorCodes(error)
            self.state = ODriveAxisState(state)
            self.procedureDone = procedureDone != 0
            self.trajectoryDone = trajectoryDone != 0

    def clear_errors_msg(self, identify: bool = False) -> None:
        data = b'\x01' if identify else b'\x00'
        self.bus.send(can.Message(
            arbitration_id=(
                self.node_id << 5) | ODriveCommand.CLEAR_ERRORS_CMD,
            data=data,
            is_extended_id=False
        ))

    def reboot_msg(self, action: int):
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5) | ODriveCommand.REBOOT_CMD,
            data=[action],
            is_extended_id=False
        ))

    def getErrorDescription(self, error_code: int):
        now = datetime.now()
        if self._lastError != self.error or self._nextPrint <= now:
            desc = get_error_description(error_code)
            self.logger.error(desc)
            self._nextPrint = now + timedelta(seconds=1)
        self._lastError = self.error
        # self.clear_errors_msg()

    def set_state_msg(self, state: ODriveAxisState):
        payload = struct.pack('<I', state)
        self.bus.send(can.Message(
            arbitration_id=(
                self.node_id << 5) | ODriveCommand.SET_AXIS_STATE_CMD,
            data=payload,
            is_extended_id=False
        ))

    def set_controller_mode(self, controlMode: ODriveControlMode, inputMode: ODriveInputMode):
        payload = struct.pack('<II', controlMode, inputMode)
        self.bus.send(can.Message(
            arbitration_id=(
                self.node_id << 5) | ODriveCommand.SET_CONTROLLER_MODE_CMD,
            data=payload,
            is_extended_id=False
        ))

    def _rxsdo(self, write: bool, endpointID: ODriveEndpoints, fmt: str, *data: Any):
        payload = struct.pack(
            f'<BHx{fmt}', 1 if write else 0, endpointID, *data)
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5) | ODriveCommand.RX_SDO_CMD,
            data=payload,
            is_extended_id=False
        ))

    def feed_watchdog_msg(self):
        self._rxsdo(True, ODriveEndpoints.AXIS_WATCHDOG_FEED, '')

    def set_inertia(self, inertia: float):
        # self._rxsdo(True, ODriveEndpoints.CONFIG_INERTIA, 'f', inertia)
        raise NotImplementedError()

    def set_input_filter_bandwidth(self, bandwidth: float):
        # self._rxsdo(
        #     True, ODriveEndpoints.CONFIG_INPUT_FILTER_BANDWIDTH, 'f', bandwidth)
        raise NotImplementedError()

    def set_velocity(self, vel: float) -> None:
        payload = struct.pack('<ff', vel, 0.0)
        self.bus.send(can.Message(
            arbitration_id=(
                self.node_id << 5) | ODriveCommand.SET_INPUT_VEL_CMD,
            data=payload,
            is_extended_id=False
        ))

    def set_traj_vel_limit(self, vel: float) -> None:
        payload = struct.pack('<f', vel)
        self.bus.send(can.Message(
            arbitration_id=(
                self.node_id << 5) | ODriveCommand.SET_TRAJ_VEL_LIMIT_CMD,
            data=payload,
            is_extended_id=False
        ))

    def set_trap_config(self, maxVel: float, accel: float, decel: float) -> None:
        self._rxsdo(True, ODriveEndpoints.TRAP_TRAJ_VEL_LIMIT, 'f', maxVel)
        self._rxsdo(True, ODriveEndpoints.TRAP_TRAJ_ACCEL_LIMIT, 'f', accel)
        self._rxsdo(True, ODriveEndpoints.TRAP_TRAJ_DECEL_LIMIT, 'f', decel)

    def set_position(
        self,
        pos: float,
        vel_feedforward: float = 0.0,
        torque_feedforward: float = 0.0
    ) -> None:
        """
        Set target position (revolutions) with optional velocity and torque feed-forward.

        Frame layout:
          Bytes 0-3: Input_Pos (float32, rev)
          Bytes 4-5: Vel_FF (int16, 0.001 rev/s)
          Bytes 6-7: Torque_FF (int16, 0.001 Nm)
        """
        vel_int = int(vel_feedforward * 1000)
        torque_int = int(torque_feedforward * 1000)
        data = struct.pack('<fhh', pos, vel_int, torque_int)
        self.bus.send(can.Message(
            arbitration_id=(
                self.node_id << 5) | ODriveCommand.SET_INPUT_POS_CMD,
            data=data,
            is_extended_id=False
        ))
        self._setpoint = pos

    def call_estop(self) -> None:
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5) | ODriveCommand.ESTOP_CMD,
            data=b'',
            is_extended_id=False
        ))

    def get_encoder_estimates_msg(self) -> None:
        """Request encoder position and velocity."""
        self.bus.send(can.Message(
            arbitration_id=(
                self.node_id << 5) | ODriveCommand.GET_ENCODER_ESTIMATES_CMD,
            is_extended_id=False,
            is_remote_frame=True
        ))

    def get_temperature_msg(self) -> None:
        """Request FET and motor temperatures."""
        self.bus.send(can.Message(
            arbitration_id=(
                self.node_id << 5) | ODriveCommand.GET_TEMPERATURE_CMD,
            is_extended_id=False,
            is_remote_frame=True
        ))

    def get_bus_voltage_current_msg(self):
        """Request bus voltage and current."""
        self.bus.send(can.Message(
            arbitration_id=(
                self.node_id << 5) | ODriveCommand.GET_BUS_VOLTAGE_CURRENT_CMD,
            is_extended_id=False,
            is_remote_frame=True
        ))

    def get_torques_msg(self):
        """Request torque setpoint and estimate."""
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5) | ODriveCommand.GET_TORQUES_CMD,
            is_extended_id=False,
            is_remote_frame=True
        ))

    def get_powers_msg(self):
        """Request electrical and mechanical power."""
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5) | ODriveCommand.GET_POWERS_CMD,
            is_extended_id=False,
            is_remote_frame=True
        ))
