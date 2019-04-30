import time
import numpy as np

class PID:

    def __init__(self, target, p_gain, i_gain, d_gain, i_cap=None, o_cap=None):

        self.target = target
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.d_gain = d_gain
        self.i_cap = i_cap
        self.o_cap = o_cap

        self.p_comp = 0
        self.i_comp = 0
        self.d_comp = 0

        self.last_value = None
        self.last_error = None
        self.last_timestamp = None
        self.output = 0

        self.debug = []

    def update(self, value, timestamp, target=None):

        if target is not None:
            self.target = target

        if self.last_timestamp is None:
            self.last_timestamp = timestamp

        time_difference = timestamp - self.last_timestamp

        if time_difference != 0:

            error = self.target - value

            self.p_comp = error * self.p_gain

            self.i_comp += error * time_difference  * self.i_gain
            if self.i_cap is not None:  # Limiter
                self.i_comp = np.clip(self.i_comp, -self.i_cap, self.i_cap)

            if self.last_error is not None:
                error_difference = error - self.last_error
                self.d_comp = (error_difference/time_difference) * self.d_gain

            self.last_value = value
            self.last_error = error
            self.last_timestamp = timestamp
            output = self.p_comp + self.i_comp + self.d_comp

            if self.o_cap is not None:
                self.output = np.clip(output, -self.o_cap, self.o_cap)
            else:
                self.output = output

        return self.output
