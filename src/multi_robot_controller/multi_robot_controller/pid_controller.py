#!/usr/bin/env python3

import math


class PIDController:
    def __init__(self, kp: float, ki: float, kd: float, dt: float, max_value: float, integral_limit: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = max(dt, 1e-3)
        self.max_value = max_value
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error: float) -> float:
        self.integral = max(
            -self.integral_limit,
            min(self.integral_limit, self.integral + error * self.dt),
        )
        derivative = (error - self.prev_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return max(-self.max_value, min(self.max_value, output))

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = 0.0


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))
