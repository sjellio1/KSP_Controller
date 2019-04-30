import numpy as np
import matplotlib.pyplot as plt
from GLOBALS import GLOBALS as G
from PID import PID
import time
from Telemetry import Telemetry
from Display import VehicleDisplay
from AscentTrajectory import AscentTrajectory


class AscentController:

    def __init__(self, mission_config, connection, vehicle, launch_frame):

        self.mission_config = mission_config
        self.connection = connection
        self.vehicle = vehicle
        self.launch_frame = launch_frame

        self.ascent_trajectory = AscentTrajectory()

        self.display = VehicleDisplay(self.ascent_trajectory)
        self.telemetry = Telemetry(self.connection, self.vehicle, self.launch_frame)

    def mission(self):

        roll_pid = PID(0, 0.005, 0, 0.001)
        # roll_pid = PID(0, 0, 0, 0)
        pitch_pid = PID(0, 0.01, 0, 0.01, 0.03)
        yaw_pid = PID(0, 0.01, 0, 0.01)
        # yaw_pid = PID(0, 0, 0, 0)

        self.vehicle.control.activate_next_stage()  # Start Main Engines

        # Step 1: Gather Data Before Controlling
        while self.vehicle.orbit.apoapsis - G.KERBIN_RADIUS < np.int(self.mission_config.get('PARAMETERS', 'target_altitude')):

            self.telemetry.update(self.vehicle.control)
            self.display.update(self.telemetry, roll_pid, pitch_pid, yaw_pid)

            # TODO: Implement AttitudeControl Class
            roll_current    = -self.telemetry.q2e(self.telemetry.rot.v[-1])[1]
            pitch_current   = -self.telemetry.q2e(self.telemetry.rot.v[-1])[0]
            yaw_current     = -self.telemetry.q2e(self.telemetry.rot.v[-1])[2]
            timestamp       = self.telemetry.rot.t[-1]

            self.vehicle.control.roll = roll_pid.update(roll_current,
                                                        timestamp)

            # pitch is commanded to dr/a ratio plus an error to optimal trajectory
            desired_pitch = np.degrees(np.arctan(self.ascent_trajectory.get_dr_dot(self.telemetry.alt(self.telemetry.pos.v[-1]))))
            pitch_offset_due_to_error = 0.015 * (self.ascent_trajectory.get_dr(self.telemetry.alt(self.telemetry.pos.v[-1])) - self.telemetry.dr(self.telemetry.pos.v[-1]))

            self.vehicle.control.pitch = pitch_pid.update(pitch_current,
                                                          timestamp,
                                                          desired_pitch + pitch_offset_due_to_error)

            self.vehicle.control.yaw = yaw_pid.update(yaw_current,
                                                      timestamp)

            self.vehicle.control.throttle = 0.7

        self.vehicle.control.throttle = 0
        time.sleep(3)
        self.vehicle.control.activate_next_stage()
