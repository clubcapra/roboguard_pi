from enum import IntEnum, IntFlag
from typing import Dict

# Non exhaustive list of odrive codes



# odrive commands
class ODriveCommand(IntEnum):
    GET_VERSION_CMD = 0x00
    HEARTBEAT_CMD = 0x01
    ESTOP_CMD = 0x02
    GET_ERROR_CMD = 0x03
    RX_SDO_CMD = 0x04
    TX_SDO_CMD = 0x05
    ADDRESS_CMD = 0x06
    SET_AXIS_STATE_CMD = 0x07
    GET_ENCODER_ESTIMATES_CMD = 0x09
    SET_CONTROLLER_MODE_CMD = 0x0B
    SET_INPUT_POS_CMD = 0x0C
    SET_INPUT_VEL_CMD = 0x0D
    SET_TRAJ_VEL_LIMIT_CMD = 0x11
    GET_TEMPERATURE_CMD = 0x15
    REBOOT_CMD = 0x16
    GET_BUS_VOLTAGE_CURRENT_CMD = 0x17
    CLEAR_ERRORS_CMD = 0x18
    GET_TORQUES_CMD = 0x1C
    GET_POWERS_CMD = 0x1D


class ODriveRebootAction(IntEnum):
    REBOOT_ACTION_REBOOT = 0
    REBOOT_ACTION_SAVE = 1
    REBOOT_ACTION_ERASE = 2

# odrive axis state
class ODriveAxisState(IntEnum):
    UNDEFINED = 0
    IDLE = 1
    STARTUP_SEQUENCE = 2
    FULL_CALIBRATION_SEQUENCE = 3
    MOTOR_CALIBRATION = 4
    ENCODER_INDEX_SEARCH = 6
    ENCODER_OFFSET_CALIBRATION = 7
    CLOSED_LOOP_CONTROL = 8

# odrive control mode
class ODriveControlMode(IntEnum):
    MODE_VOLTAGE_CONTROL = 0
    MODE_TORQUE_CONTROL = 1
    MODE_VELOCITY_CONTROL = 2
    MODE_POSITION_CONTROL = 3

# odrive input mode
class ODriveInputMode(IntEnum):
    INPUT_INACTIVE = 0
    INPUT_PASSTHROUGH = 1
    INPUT_VEL_RAMP = 2
    INPUT_POS_FILTER = 3
    INPUT_MIX_CHANNELS = 4
    INPUT_TRAP_TRAJ = 5
    INPUT_TORQUE_RAMP = 6
    INPUT_MIRROR = 7
    INPUT_TUNING = 8


class ODriveEndpoints(IntEnum):
    CONFIG_INERTIA = 413  # float
    CONFIG_INPUT_FILTER_BANDWIDTH = 414  # float
    TRAP_TRAJ_VEL_LIMIT = 429  # float
    TRAP_TRAJ_ACCEL_LIMIT = 430  # float
    TRAP_TRAJ_DECEL_LIMIT = 431  # float


class ODriveProcedureResult(IntEnum):
    SUCCESS = 0
    BUSY = 1
    CANCELED = 2
    DISARMED = 3
    NO_RESPONSE = 4
    POLE_PAIR_CPR_MISMATCH = 5
    PHASE_RESISTANCE_OUT_OF_RANGE = 6
    PHASE_INDUCTANCE_OUT_OF_RANGE = 7
    UNBALANCED_PHASES = 8
    INVALID_MOTOR_TYPE = 9
    ILLEGAL_HALL_STATE = 10
    TIMEOUT = 11
    HOMING_WITHOUT_ENDSTOP = 12
    INVALID_STATE = 13
    NOT_CALIBRATED = 14
    NOT_CONVERGING = 15


class ODriveErrorCodes(IntFlag):
    INITIALIZING = 1
    SYSTEM_LEVEL = 2
    TIMING_ERROR = 4
    MISSING_ESTIMATE = 8
    BAD_CONFIG = 16
    DRV_FAULT = 32
    MISSING_INPUT = 64
    DC_BUS_OVER_VOLTAGE = 256
    DC_BUS_UNDER_VOLTAGE = 512
    DC_BUS_OVER_CURRENT = 1024
    DC_BUS_OVER_REGEN_CURRENT = 2048
    CURRENT_LIMIT_VIOLATION = 4096
    MOTOR_OVER_TEMP = 8192
    INVERTER_OVER_TEMP = 16384
    VELOCITY_LIMIT_VIOLATION = 32768
    POSITION_LIMIT_VIOLATION = 65536
    WATCHDOG_TIMER_EXPIRED = 16777216
    ESTOP_REQUESTED = 33554432
    SPINOUT_DETECTED = 67108864
    BRAKE_RESISTOR_DISARMED = 134217728
    THERMISTOR_DISCONNECTED = 268435456
    CALIBRATION_ERROR = 1073741824


ERROR_CODES: Dict[ODriveErrorCodes, str] = {
    ODriveErrorCodes.INITIALIZING: "INITIALIZING - The system is initializing or reconfiguring.",
    ODriveErrorCodes.SYSTEM_LEVEL: "SYSTEM_LEVEL - Unexpected system error such as memory corruption, stack overflow, frozen thread, assert fail, etc.",
    ODriveErrorCodes.TIMING_ERROR: "TIMING_ERROR - An internal hard timing requirement was violated. Likely due to system overload.",
    ODriveErrorCodes.MISSING_ESTIMATE: "MISSING_ESTIMATE - The position/velocity/phase estimate was invalid.",
    ODriveErrorCodes.BAD_CONFIG: "BAD_CONFIG - The ODrive configuration is invalid or incomplete.",
    ODriveErrorCodes.DRV_FAULT: "DRV_FAULT - The gate driver chip reported an error.",
    ODriveErrorCodes.MISSING_INPUT: "MISSING_INPUT - No value was provided for input_pos, input_vel, or input_torque.",
    ODriveErrorCodes.DC_BUS_OVER_VOLTAGE: "DC_BUS_OVER_VOLTAGE - The DC voltage exceeded the configured overvoltage trip level.",
    ODriveErrorCodes.DC_BUS_UNDER_VOLTAGE: "DC_BUS_UNDER_VOLTAGE - The DC voltage fell below the configured undervoltage trip level.",
    ODriveErrorCodes.DC_BUS_OVER_CURRENT: "DC_BUS_OVER_CURRENT - Too much DC current was pulled.",
    ODriveErrorCodes.DC_BUS_OVER_REGEN_CURRENT: "DC_BUS_OVER_REGEN_CURRENT - Too much DC current was regenerated.",
    ODriveErrorCodes.CURRENT_LIMIT_VIOLATION: "CURRENT_LIMIT_VIOLATION - The motor current exceeded the specified hard max current.",
    ODriveErrorCodes.MOTOR_OVER_TEMP: "MOTOR_OVER_TEMP - The motor temperature exceeded the specified upper limit.",
    ODriveErrorCodes.INVERTER_OVER_TEMP: "INVERTER_OVER_TEMP - The inverter temperature exceeded the specified upper limit.",
    ODriveErrorCodes.VELOCITY_LIMIT_VIOLATION: "VELOCITY_LIMIT_VIOLATION - The velocity exceeds the velocity limit.",
    ODriveErrorCodes.POSITION_LIMIT_VIOLATION: "POSITION_LIMIT_VIOLATION - The position exceeded the position limit.",
    ODriveErrorCodes.WATCHDOG_TIMER_EXPIRED: "WATCHDOG_TIMER_EXPIRED - The axis watchdog timer expired.",
    ODriveErrorCodes.ESTOP_REQUESTED: "ESTOP_REQUESTED - An emergency stop was requested.",
    ODriveErrorCodes.SPINOUT_DETECTED: "SPINOUT_DETECTED - A spinout situation was detected.",
    ODriveErrorCodes.BRAKE_RESISTOR_DISARMED: "BRAKE_RESISTOR_DISARMED - The brake resistor was disarmed.",
    ODriveErrorCodes.THERMISTOR_DISCONNECTED: "THERMISTOR_DISCONNECTED - The motor thermistor is disconnected.",
    ODriveErrorCodes.CALIBRATION_ERROR: "CALIBRATION_ERROR - A calibration procedure failed."
}


def get_error_description(error_code: ODriveErrorCodes):
    """
    Returns a human-readable description for a given error code.
    This handles both individual errors and combined error bitmasks.
    """
    description = []
    for code, desc in ERROR_CODES.items():
        if error_code & code:
            description.append(desc)
    return "\n".join(description) if description else "No error."
