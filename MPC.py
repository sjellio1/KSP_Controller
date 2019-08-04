import cvxpy as cp
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D

class MPC:
    def __init__(self):

        self.Q_weight = [1, 0.1, 1, 0.1, 1, 0.1, 0, 0, 0, 0]
        self.R_weight = [0, 0, 0.1]

        self.g = 9.81

        self.mass = 1
        self.i_pitch = 1
        self.i_yaw = 1
        self.max_pitch = 0
        self.max_yaw = 0
        self.max_thrust = 0

    def calculate_inputs(self, x0, u0, target, time_steps):

        x = cp.Variable((len(x0), time_steps+1), name='states')
        u = cp.Variable((len(u0), time_steps), name='inputs')

        A, B, D = self.dynamics_jacobian(x0, u0)

        Q = np.eye(len(x0)) * self.Q_weight
        R = np.eye(len(u0)) * self.R_weight

        cost = 0
        constraints = []
        for t in range(time_steps):
            cost += cp.quad_form(target[:, t + 1] - x[:, t + 1], Q)  # + cp.quad_form(u[:, t], R)

            constraints += [
                x[:, t+1]-x[:, t] == A@x[:, t] + B@u[:, t] + D,
                u[0, t] >= -self.max_pitch,
                u[0, t] <= self.max_pitch,
                u[1, t] >= -self.max_yaw,
                u[1, t] <= self.max_yaw,
                u[2, t] >= 0.1*self.max_thrust,
                u[2, t] <= self.max_thrust]

        cost = cp.quad_form(target[:, time_steps] - x[:, time_steps], Q)

        constraints += [x[:, 0] == x0,
                        ]#cp.norm(target[:, time_steps] - x[:, time_steps], 1) <= 0.001]

        problem = cp.Problem(cp.Minimize(cost), constraints=constraints)
        problem.solve(verbose=False, solver=cp.SCS, max_iters=100000)

        error = target - x.value

        plot = False
        if plot:
            fig = plt.figure()
            ax = Axes3D(fig)
            ax.scatter3D(target[0, :], target[2, :], target[4, :])
            ax.plot3D(x.value[0, :], x.value[2, :], x.value[4, :])
            ax.axis('equal')

            fig.show()

        return u.value[0, 0], u.value[1, 0], u.value[2, 0]

    def update_constraints(self, pitch, yaw, thrust):
        self.max_pitch = pitch
        self.max_yaw = yaw
        self.max_thrust = thrust

    def update_vehicle_properties(self, vehicle):
        self.mass = vehicle.mass
        self.i_pitch = vehicle.moment_of_inertia[0]
        self.i_yaw = vehicle.moment_of_inertia[2]

        self.max_pitch = np.abs(vehicle.available_torque[0][0])
        self.max_yaw = np.abs(vehicle.available_torque[0][2])
        self.max_thrust = vehicle.max_thrust

    def dynamics_func(self, states, inputs):
        x, x_dot, y, y_dot, z, z_dot, t, t_dot, p, p_dot = states
        torque_t, torque_p, thrust = inputs

        f = np.array([
            # x
            x_dot,
            # x dot
            thrust / self.mass * math.sin(t),
            # y
            y_dot,
            # y dot
            thrust / self.mass * math.sin(p),
            # z
            z_dot,
            # z dot
            thrust / self.mass * math.cos(t) + thrust / self.mass * math.cos(p),
            # theta
            t_dot,
            # theta dot
            torque_t / self.i_pitch,
            # phi
            p_dot,
            # phi dot
            torque_p / self.i_yaw,
            ])

        return f

    def dynamics_jacobian(self, states, inputs):
        x, x_dot, y, y_dot, z, z_dot, t, t_dot, p, p_dot = states
        torque_t, torque_p, thrust = inputs

        ja = np.array([

            #x dx  y dy  z dz  t dt  p dp
            # x
            [0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
            # x dot
            [0, 0, 0, 0, 0, 0, thrust / self.mass * t_dot * math.cos(t), 0, 0, 0],
            # y
            [0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
            # y dot
            [0, 0, 0, 0, 0, 0, 0, 0, thrust / self.mass * p_dot * math.cos(p), 0],
            # z
            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
            # z dot
            [0, 0, 0, 0, 0, 0, -thrust / self.mass * t_dot * math.sin(t), 0, -thrust / self.mass * p_dot * math.sin(p), 0],
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
            [0, 0, 1 / self.mass * math.sin(t)],
            # y
            [0, 0, 0],
            # y dot
            [0, 0, 1 / self.mass * math.sin(p)],
            # z
            [0, 0, 0],
            # z dot
            [0, 0, 1 / self.mass * math.cos(t) + 1 / self.mass * math.cos(p)],
            # theta
            [0, 0, 0],
            # theta dot
            [1/self.i_pitch, 0, 0],
            # phi
            [0, 0, 0],
            # phi dot
            [0, 1/self.i_yaw, 0],
            ])

        d = np.array([
            # x
            0,
            # x dot
            0,
            # y
            0,
            # y dot
            0,
            # z
            0,
            # z dot
            -self.g,
            # theta
            0,
            # theta dot
            0,
            # phi
            0,
            # phi dot
            0,
        ])

        return ja, jb, d

    def descent_profile(self, x, y, z, time):
        pass
