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


class Odometry:
    def __init__(self, velocity_rolling_window_size: int) -> None:
        #Current timestamp:
        self.timestamp_: Time = Time()
        
        # Current pose:
        self.x_: Meters = 0.0
        self.y_: Meters = 0.0
        self.heading_: Radians = 0.0
        self.linear_: Meters = 0.0
        self.angular_: Radians = 0.0
        self.wheel_separation_: float = 0.0
        self.left_wheel_radius_: float = 0.0
        self.right_wheel_radius_: float = 0.0
        self.left_wheel_old_pos_: float = 0.0
        self.right_wheel_old_pos_: float = 0.0
        self.velocity_rolling_window_size_: int = velocity_rolling_window_size
        self.linear_accumulator_: RollingMeanAccumulator = RollingMeanAccumulator(velocity_rolling_window_size)
        self.angular_accumulator_: RollingMeanAccumulator = RollingMeanAccumulator(velocity_rolling_window_size)

    def init(self, time: Time) -> None:
        self.resetAccumulators()
        self.timestamp_ = time

    def update(self, left_pos: float, right_pos: float, time: Time) -> bool:
        dt: float = (time - self.timestamp_).nanoseconds / 1e9
        if dt < 0.0001:
            return False

        left_wheel_cur_pos: float = left_pos * self.left_wheel_radius_
        right_wheel_cur_pos: float = right_pos * self.right_wheel_radius_

        left_wheel_est_vel: float = left_wheel_cur_pos - self.left_wheel_old_pos_
        right_wheel_est_vel: float = right_wheel_cur_pos - self.right_wheel_old_pos_

        self.left_wheel_old_pos_ = left_wheel_cur_pos
        self.right_wheel_old_pos_ = right_wheel_cur_pos

        return self.updateFromVelocity(left_wheel_est_vel, right_wheel_est_vel, time)

    def updateFromVelocity(self, left_vel: float, right_vel: float, time: Time) -> bool:
        dt: float = (time - self.timestamp_).nanoseconds / 1e9
        if dt < 0.0001:
            return False

        linear: float = (left_vel + right_vel) * 0.5
        angular: float = (right_vel - left_vel) / self.wheel_separation_

        self.integrateExact(linear, angular)

        self.timestamp_ = time
        self.linear_accumulator_.accumulate(linear / dt)
        self.angular_accumulator_.accumulate(angular / dt)

        self.linear_ = self.linear_accumulator_.getRollingMean()
        self.angular_ = self.angular_accumulator_.getRollingMean()

        return True

    def updateOpenLoop(self, linear: float, angular: float, time: Time) -> None:
        self.linear_ = linear
        self.angular_ = angular

        dt: float = (time - self.timestamp_).nanoseconds / 1e9
        self.timestamp_ = time
        self.integrateExact(linear * dt, angular * dt)

    def resetOdometry(self) -> None:
        self.x_ = 0.0
        self.y_ = 0.0
        self.heading_ = 0.0

    def setWheelParams(self, wheel_separation: float, left_wheel_radius: float, right_wheel_radius: float) -> None:
        self.wheel_separation_ = wheel_separation
        self.left_wheel_radius_ = left_wheel_radius
        self.right_wheel_radius_ = right_wheel_radius

    def setVelocityRollingWindowSize(self, velocity_rolling_window_size: int) -> None:
        self.velocity_rolling_window_size_ = velocity_rolling_window_size
        self.resetAccumulators()

    def integrateRungeKutta2(self, linear: float, angular: float) -> None:
        direction: float = self.heading_ + angular * 0.5
        self.x_ += linear * math.cos(direction)
        self.y_ += linear * math.sin(direction)
        self.heading_ += angular

    def integrateExact(self, linear: float, angular: float) -> None:
        if abs(angular) < 1e-6:
            self.integrateRungeKutta2(linear, angular)
        else:
            heading_old: float = self.heading_
            r: float = linear / angular
            self.heading_ += angular
            self.x_ += r * (math.sin(self.heading_) - math.sin(heading_old))
            self.y_ += -r * (math.cos(self.heading_) - math.cos(heading_old))

    def resetAccumulators(self) -> None:
        self.linear_accumulator_ = RollingMeanAccumulator(self.velocity_rolling_window_size_)
        self.angular_accumulator_ = RollingMeanAccumulator(self.velocity_rolling_window_size_)
