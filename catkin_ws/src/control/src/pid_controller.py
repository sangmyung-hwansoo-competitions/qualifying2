#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np


class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0

    def get_control_output(self, error_from_mid):
        self.integral += error_from_mid
        self.derivative = error_from_mid - self.prev_error
        self.prev_error = error_from_mid

        # if abs(error_from_mid) > 100:
        #     return 0.3 * error_from_mid + self.Ki * self.integral + self.Kd * self.derivative

        # if abs(error_from_mid) > 120:
        #     return 0.6 * error_from_mid + self.Ki * self.integral + self.Kd * self.derivative

        return self.Kp * error_from_mid + self.Ki * self.integral + self.Kd * self.derivative