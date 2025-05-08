from datetime import datetime, timedelta
from enum import IntEnum
from types import TracebackType
from typing import Any, Callable, Optional, Sequence, Tuple, Type, TypeVar, Union
import can
from rclpy.impl.rcutils_logger import RcutilsLogger


class CanStatus(IntEnum):
    UNINITIALIZED = 0
    INITIALIZED = 1
    ERROR = 2
    SHUTDOWN = 3


class CanError(IntEnum):
    NONE = 0
    INIT_ERROR = 1
    FATAL_ERROR = 2
    OPERATION_ERROR = 3
    TIMEOUT = 4


_Func = TypeVar('_Func', bound=Callable)

CanErrorCallback = Callable[[CanError], None]


class EmptyCyclicSendTask(can.CyclicSendTaskABC):
    """
    Message send task with defined period
    """

    def __init__(self) -> None:
        pass

    @staticmethod
    def _check_and_convert_messages(
        messages: Union[Sequence[can.Message], can.Message]
    ) -> Tuple[can.Message, ...]:
        return None


class CanHandler(can.BusABC):
    def __init__(
        self,
        logger: RcutilsLogger,
        channel: str = 'can0',
        bitrate: int = 500000,
        errorCallback: Optional[CanErrorCallback] = None
    ):
        self._error = CanError.NONE
        self.nextRetry = datetime.now()
        self._bus: Optional[can.BusABC] = None
        self._shutdown = False
        self.logger = logger.get_child("CanHandler")
        self.errorCallback = errorCallback
        self.channel = channel
        self.bitrate = bitrate

    @property
    def _is_shutdown(self) -> bool:
        if self.bus is None:
            return True
        return self.bus._is_shutdown

    @property
    def error(self) -> CanError:
        return self._error

    @error.setter
    def error(self, error: CanError):
        self._error = error
        # if self.errorCallback is not None:
        #     self.errorCallback(error)

    def _try(self, func: _Func, *args, **kwargs) -> Any:
        try:
            return func(*args, **kwargs)
        except can.exceptions.CanInterfaceNotImplementedError as e:
            self.logger.fatal(f"SometCanErrorCallbackhing is very wrong: {e}")
            self.logger.info("Restarting can bus in 5 sec")
            self.nextRetry = datetime.now() + timedelta(seconds=5)
            self.error = CanError.FATAL_ERROR
            return None
        except can.exceptions.CanInitializationError as e:
            self.logger.error(f"Initializing failed: {e}")
            self.logger.info("Restarting can bus in 5 sec")
            self.nextRetry = datetime.now() + timedelta(seconds=5)
            self.error = CanError.INIT_ERROR
            return None
        except can.exceptions.CanOperationError as e:
            self.logger.error(f"Operation failed: {e}")
            self.logger.info("Restarting can bus in 5 sec")
            self.nextRetry = datetime.now() + timedelta(seconds=5)
            self.error = CanError.OPERATION_ERROR
            return None
        except can.exceptions.CanTimeoutError as e:
            self.logger.warning(f"Timed out: {e}")
            return None

    @property
    def status(self) -> CanStatus:
        if self._shutdown:
            return CanStatus.SHUTDOWN
        if self.error != CanError.NONE:
            return CanStatus.ERROR
        if self._bus is None:
            return CanStatus.UNINITIALIZED
        if self._bus.state == can.BusState.ERROR:
            return CanStatus.ERROR
        return CanStatus.INITIALIZED

    def init_can_bus(self) -> can.BusABC:
        self.logger.info("Starting can bus")

        bus = can.interface.Bus(channel=self.channel,
                                interface='socketcan', bitrate=self.bitrate)
        # flush pending frames
        while bus.recv(timeout=0):
            pass

        self.logger.info("Done")
        self.error = CanError.NONE
        return bus

    @property
    def bus(self) -> Optional[can.BusABC]:
        if self._shutdown:
            return None
        if ((self._bus is None or self._bus.state == can.BusState.ERROR) and
                self.nextRetry < datetime.now()):
            if self._bus is not None:
                self._bus.shutdown()

            self._bus = self._try(self.init_can_bus)

        return self._bus

    def start(self):
        self._shutdown = False

    def __str__(self) -> str:
        if self.bus is not None:
            return self.bus.__str__()
        return ''

    def recv(self, timeout: Optional[float] = None) -> Optional[can.Message]:
        if self.bus is not None:
            return self._try(self.bus.recv, timeout)

    def send(self, msg: can.Message, timeout: Optional[float] = None) -> None:
        if self.bus is not None:
            self._try(self.bus.send, msg, timeout)

    def send_periodic(
        self,
        msgs: Union[can.Message, Sequence[can.Message]],
        period: float,
        duration: Optional[float] = None,
        store_task: bool = True,
        autostart: bool = True,
        modifier_callback: Optional[Callable[[can.Message], None]] = None,
    ) -> can.broadcastmanager.CyclicSendTaskABC:
        if self.bus is not None:
            res = self._try(self.bus.send_periodic, msgs, period,
                            duration, store_task, autostart, modifier_callback)
            if res is not None:
                return res
        return EmptyCyclicSendTask()

    def stop_all_periodic_tasks(self, remove_tasks: bool = True) -> None:
        if self.bus is not None:
            self.bus.stop_all_periodic_tasks(remove_tasks)

    @property
    def filters(self) -> Optional[can.typechecking.CanFilters]:
        if self.bus is not None:
            return self.bus.filters

    @filters.setter
    def filters(self, filters: Optional[can.typechecking.CanFilters]) -> None:
        self.set_filters(filters)

    def set_filters(
        self, filters: Optional[can.typechecking.CanFilters] = None
    ) -> None:
        if self.bus is not None:
            self._try(self.bus.set_filters, filters)

    def _apply_filters(self, filters: Optional[can.typechecking.CanFilters]) -> None:
        if self.bus is not None:
            self._try(self.bus._apply_filters, filters)

    def _matches_filters(self, msg: can.Message) -> bool:
        if self.bus is not None:
            res = self._try(self.bus._matches_filters, msg)
            if res is not None:
                return res
        return False

    def flush_tx_buffer(self) -> None:
        if self.bus is not None:
            self._try(self.bus.flush_tx_buffer)

    def shutdown(self):
        self._shutdown = True
        if self._bus is None:
            return
        self._bus.shutdown()
        self._bus = None
        self.logger.info("Can bus shutdown")

    def __enter__(self):
        self.start()
        return self

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc_value: Optional[BaseException],
        traceback: Optional[TracebackType],
    ) -> None:
        self.shutdown()

    def __del__(self) -> None:
        if self.bus is not None:
            self._try(self.bus.__del__)

    @property
    def state(self) -> can.BusState:
        if self.bus is not None:
            return self.bus.state
        return can.BusState.ACTIVE

    @state.setter
    def state(self, new_state: can.BusState) -> None:
        if self.bus is not None:
            self.bus.state = new_state

    @property
    def protocol(self) -> can.CanProtocol:
        if self.bus is not None:
            return self.bus.protocol
        return can.CanProtocol.CAN_20

    def fileno(self) -> int:
        if self.bus is not None:
            res = self._try(self.bus.fileno)
            if res is not None:
                return res
        return 0
