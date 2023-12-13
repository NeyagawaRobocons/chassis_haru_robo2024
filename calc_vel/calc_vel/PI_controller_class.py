# description: PI制御を行うクラス

import numpy as np

class PI_controller:
    def __init__(self) -> None:
        self.Kp = 0.0
        self.Ki = 0.0
        self.dt = 0.0
        self.error = 0.0
        self.error_sum = 0.0
        self.output = 0.0

    def set_params(self, Kp, Ki, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.dt = dt

    def calc_output(self, error):
        self.error = error
        self.error_sum += error * self.dt
        self.output = self.Kp * self.error + self.Ki * self.error_sum
        return self.output