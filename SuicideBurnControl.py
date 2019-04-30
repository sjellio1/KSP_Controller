import numpy as np
from PID import PID

class SuicideBurnControl:

    def __init__(self, connection, vehicle, telemetry, launch_frame):

        self.connection = connection
        self.vehicle = vehicle
        self.telemetry = telemetry
        self.launch_frame = launch_frame

        self.ready_to_land = False

        self.max_acceleration = self.vehicle.available_thrust/self.vehicle.mass - 9.82

    def update(self):

        altitude = self.vehicle.flight().surface_altitude
        if altitude < 10000 and not self.ready_to_land:
            self.prepare_to_land()

        throttle = 0

        # Calculate Desired Thrust Vector
        velocity = self.telemetry.vel.v[-1]
        vertical_velocity = velocity[1]
        y_velocity = velocity[0]
        z_velocity = -velocity[2]

        desired_pitch_angle = np.degrees(np.arctan2(-z_velocity, np.abs(vertical_velocity)))
        desired_yaw_angle = np.degrees(np.arctan2(-y_velocity, np.abs(vertical_velocity)))

        # Calculate Throttle
        time_to_zero = np.linalg.norm(velocity) / self.max_acceleration
        stop_distance = np.linalg.norm(velocity) * time_to_zero + 0.5 * self.max_acceleration * time_to_zero * time_to_zero
        print(stop_distance, altitude)

        if stop_distance > altitude and altitude > 3:
            throttle = 1
        else:
            throttle = 0

        return throttle, desired_pitch_angle, desired_yaw_angle

    def prepare_to_land(self):
        for leg in self.vehicle.parts.legs:
            leg.deployed = True
            self.ready_to_land = True
