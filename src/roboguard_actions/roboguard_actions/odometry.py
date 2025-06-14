# Ported from https://github.com/ros-controls/ros2_controllers/blob/master/diff_drive_controller/src/odometry.cpp
import math
from typing import List
from rclpy.time import Time

import sys
sys.path.append(__file__.removesuffix(f"/{__file__.split('/')[-1]}"))
from utils import Meters, Radians


class RollingMeanAccumulator:
    def __init__(self, window_size: int) -> None:
        self.window_size: int = window_size
        self.values: List[float] = []

    def accumulate(self, value: float) -> None:
        self.values.append(value)
        if len(self.values) > self.window_size:
            self.values.pop(0)

    def getRollingMean(self) -> float:
        if not self.values:
            return 0.0
        return sum(self.values) / len(self.values)


class DiffOdometry:
    def __init__(self, time: Time, velocity_rolling_window_size: int = 10) -> None:
        #Current timestamp:
        self.timestamp: Time = time
        
        # Current pose:
        self.x: Meters = 0.0
        self.y: Meters = 0.0
        self.heading: Radians = 0.0
        self.linear: Meters = 0.0
        self.angular: Radians = 0.0
        self.wheel_separation: Meters = 0.0
        self.left_wheel_radius: Meters = 0.0
        self.right_wheel_radius: Meters = 0.0
        self.left_wheel_old_pos: Radians = 0.0
        self.right_wheel_old_pos: Radians = 0.0
        self.velocity_rolling_window_size: int = velocity_rolling_window_size
        self.linear_accumulator: RollingMeanAccumulator = RollingMeanAccumulator(velocity_rolling_window_size)
        self.angular_accumulator: RollingMeanAccumulator = RollingMeanAccumulator(velocity_rolling_window_size)

    def init(self, time: Time) -> None:
        self.resetAccumulators()
        self.timestamp = time

    def update(self, left_pos: Radians, right_pos: Radians, time: Time) -> bool:
        dt: float = (time - self.timestamp).nanoseconds / 1e9
        if dt < 0.0001:
            return False

        left_wheel_cur_pos: Meters = left_pos * self.left_wheel_radius
        right_wheel_cur_pos: Meters = right_pos * self.right_wheel_radius

        left_wheel_est_vel: Meters = left_wheel_cur_pos - self.left_wheel_old_pos
        right_wheel_est_vel: Meters = right_wheel_cur_pos - self.right_wheel_old_pos

        self.left_wheel_old_pos = left_wheel_cur_pos
        self.right_wheel_old_pos = right_wheel_cur_pos

        return self.updateFromVelocity(left_wheel_est_vel, right_wheel_est_vel, time)

    def updateFromVelocity(self, left_vel: Meters, right_vel: Meters, time: Time) -> bool:
        dt: float = (time - self.timestamp).nanoseconds / 1e9
        if dt < 0.0001:
            return False

        linear: Meters = (left_vel + right_vel) * 0.5
        angular: Radians = (right_vel - left_vel) / self.wheel_separation

        self.integrateExact(linear, angular)

        self.timestamp = time
        self.linear_accumulator.accumulate(linear / dt)
        self.angular_accumulator.accumulate(angular / dt)

        self.linear = self.linear_accumulator.getRollingMean()
        self.angular = self.angular_accumulator.getRollingMean()

        return True

    def updateOpenLoop(self, linear: Meters, angular: Radians, time: Time) -> None:
        self.linear = linear
        self.angular = angular

        dt: float = (time - self.timestamp).nanoseconds / 1e9
        self.timestamp = time
        self.integrateExact(linear * dt, angular * dt)

    def resetOdometry(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0

    def setWheelParams(self, wheel_separation: Meters, left_wheel_radius: Meters, right_wheel_radius: Meters) -> None:
        self.wheel_separation = wheel_separation
        self.left_wheel_radius = left_wheel_radius
        self.right_wheel_radius = right_wheel_radius

    def setVelocityRollingWindowSize(self, velocity_rolling_window_size: int) -> None:
        self.velocity_rolling_window_size = velocity_rolling_window_size
        self.resetAccumulators()

    def integrateRungeKutta2(self, linear: Meters, angular: Radians) -> None:
        direction: Radians = self.heading + angular * 0.5
        self.x += linear * math.cos(direction)
        self.y += linear * math.sin(direction)
        self.heading += angular

    def integrateExact(self, linear: Meters, angular: Radians) -> None:
        if abs(angular) < 1e-6:
            self.integrateRungeKutta2(linear, angular)
        else:
            heading_old: Radians = self.heading
            r: float = linear / angular
            self.heading += angular
            self.x += r * (math.sin(self.heading) - math.sin(heading_old))
            self.y += -r * (math.cos(self.heading) - math.cos(heading_old))

    def resetAccumulators(self) -> None:
        self.linear_accumulator = RollingMeanAccumulator(self.velocity_rolling_window_size)
        self.angular_accumulator = RollingMeanAccumulator(self.velocity_rolling_window_size)
