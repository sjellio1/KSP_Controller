import cvxpy as cv
import math
import numpy as np

class GLOBALS:

    DISPLAY_PERIOD = 0.6  # seconds

    KERBIN_RADIUS = 600000  # m


class Dynamics:

    def __init__(self, m, Ip, Iy):
        self.g = 9.81
        self.m = m
        self.Ip = Ip
        self.Iy = Iy

    def fx(self, states, inputs):
        x, x_dot, y, y_dot, z, z_dot, t, t_dot, p, p_dot = states
        thrust, torque_t, torque_p = inputs

        f = np.zeros(10)
        # x
        f[0] = x_dot
        # x_dot
        f[1] = thrust / self.m * math.sin(t)
        # y
        f[2] = y_dot
        # y_dot
        f[3] = thrust / self.m * math.sin(p)
        # z
        f[4] = z_dot
        # z_dot
        f[5] = thrust / self.m * math.cos(t) + thrust + thrust / self.m * math.cos(t) - self.m*self.g
        # t
        f[6] = t_dot
        # t_dot
        f[7] = torque_t/self.Ip
        # p
        f[8] = p_dot
        # p_dot
        f[9] = torque_p/self.Iy

        return f

    def dynamics_jacobian(self, states, inputs):
        x, x_dot, y, y_dot, z, z_dot, t, t_dot, p, p_dot = states
        thrust, torque_t, torque_p = inputs

        ja = np.array([
            # x
            [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
            # x dot
            [0, 0, 0, 0, 0, 0, thrust/self.m*t_dot*math.cos(t), 0, 0, 0],
            # y
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
            # y dot
            [0, 0, 0, 0, 0, 0, 0, 0, thrust/self.m*p_dot*math.cos(p), 0],
            # z
            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
            # z dot
            [0, 0, 0, 0, 0, 0, -thrust/self.m*t_dot*math.sin(t), 0, -thrust/self.m*p_dot*math.sin(p), 0],
            # theta
            [0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
            # theta dot
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            # phi
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
            # phi dot
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            ])

        jb = np.array([
            # x
            [0, 0, 0],
            # x dot
            [1 / self.m * math.sin(t), 0, 0],
            # y
            [0, 0, 0],
            # y dot
            [1 / self.m * math.sin(p), 0, 0],
            # z
            [0, 0, 0],
            # z dot
            [0, 0, 0],
            # theta
            [0, 0, 0],
            # theta dot
            [0, 1/self.Ip, 0],
            # phi
            [0, 0, 0],
            # phi dot
            [0, 0, 1/self.Iy],
            ])

        return ja, jb