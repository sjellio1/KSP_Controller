import numpy as np
from AttitudeControl import PID
import cvxpy as cp
import matplotlib.pyplot as plt

class SuicideBurnControl:

    def __init__(self, connection, vehicle, telemetry, frames):

        self.connection = connection
        self.vehicle = vehicle
        self.telemetry = telemetry
        self.frames = frames

        self.z = 0
        self.dz = 0

        self.p_cont = 0
        self.d_cont = 0

        self.thrust_pid = PID(0, 0, 0, 0, 1, 1)

    def update(self):

        self.update_vehicle_properties()
        if self.telemetry.pos.v[-1][2] < 5:
            return 0

        flight = self.vehicle.flight()

        self.z = flight.surface_altitude - 15
        self.dz = self.telemetry.vel.v[-1][2]

        x0 = np.array([self.z, self.dz])

        gains = [899.9, 10386]

        self.p_cont = -x0[0]*gains[0]
        self.d_cont = -x0[1]*gains[1]

        lqr_thrust = self.p_cont + self.d_cont
        gravity_thrust = self.vehicle.mass * 9.81

        print(flight.surface_altitude, lqr_thrust, gravity_thrust)

        thrust = lqr_thrust + gravity_thrust

        throttle = thrust / self.vehicle.available_thrust

        throttle = np.clip(throttle, 0, 1)

        return throttle

    def update_vehicle_properties(self):

        self.max_accel = self.vehicle.available_thrust / self.vehicle.mass
