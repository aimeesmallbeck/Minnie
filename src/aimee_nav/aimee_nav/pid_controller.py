#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (C) ARDUINO SRL
#
# SPDX-License-Identifier: MPL-2.0

"""
Simple PID controllers for heading and velocity.
"""

import time
from typing import Tuple


class PIDController:
    """Generic PID controller with output clamping and integral windup limit."""

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        output_min: float = -float('inf'),
        output_max: float = float('inf'),
        integral_limit: float = float('inf'),
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_limit = integral_limit

        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = time.time()

    def reset(self) -> None:
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = time.time()

    def compute(self, setpoint: float, measurement: float) -> float:
        """Compute PID output for one timestep."""
        now = time.time()
        dt = now - self._last_time
        self._last_time = now

        if dt <= 0:
            dt = 0.001

        error = setpoint - measurement

        # Proportional
        p = self.kp * error

        # Integral with windup limit
        self._integral += error * dt
        self._integral = max(-self.integral_limit, min(self.integral_limit, self._integral))
        i = self.ki * self._integral

        # Derivative (on measurement to avoid derivative kick)
        d = 0.0
        if dt > 0:
            d = self.kd * (self._last_error - error) / dt
        self._last_error = error

        output = p + i + d
        return max(self.output_min, min(self.output_max, output))


class HeadingVelocityController:
    """
    Combined heading + velocity controller for differential-drive robots.
    """

    def __init__(
        self,
        heading_kp: float = 2.0,
        heading_ki: float = 0.0,
        heading_kd: float = 0.5,
        velocity_kp: float = 1.0,
        velocity_ki: float = 0.0,
        velocity_kd: float = 0.0,
        max_linear: float = 0.5,
        max_angular: float = 1.5,
    ) -> None:
        self.heading_pid = PIDController(
            kp=heading_kp,
            ki=heading_ki,
            kd=heading_kd,
            output_min=-max_angular,
            output_max=max_angular,
            integral_limit=1.0,
        )
        self.velocity_pid = PIDController(
            kp=velocity_kp,
            ki=velocity_ki,
            kd=velocity_kd,
            output_min=0.0,
            output_max=max_linear,
            integral_limit=1.0,
        )
        self.max_linear = max_linear
        self.max_angular = max_angular

    def reset(self) -> None:
        self.heading_pid.reset()
        self.velocity_pid.reset()

    def compute(
        self,
        target_heading: float,
        current_heading: float,
        target_speed: float,
        current_speed: float,
    ) -> Tuple[float, float]:
        """
        Compute velocity commands.

        Args:
            target_heading: desired heading in radians
            current_heading: current heading in radians
            target_speed: desired linear speed in m/s
            current_speed: current linear speed in m/s

        Returns:
            (linear_x, angular_z)
        """
        # Normalize heading error to [-pi, pi]
        heading_error = target_heading - current_heading
        while heading_error > 3.14159:
            heading_error -= 6.28318
        while heading_error < -3.14159:
            heading_error += 6.28318

        angular_z = self.heading_pid.compute(0.0, -heading_error)
        linear_x = self.velocity_pid.compute(target_speed, current_speed)

        # Reduce speed when turning sharply; stop forward motion entirely
        # if heading error exceeds 90° so the robot can turn in place.
        turn_penalty = max(0.0, 1.0 - abs(heading_error) / 1.57)
        linear_x *= turn_penalty

        return linear_x, angular_z
