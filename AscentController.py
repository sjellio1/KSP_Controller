import numpy as np
import matplotlib.pyplot as plt
from GLOBALS import GLOBALS as G
import time
from Telemetry import Telemetry
from Commanding import Commanding
from AttitudeControl import AttitudeControl
from Display import VehicleDisplay
from ReferenceTrajectory import ReferenceAscentTrajectory


class AscentController:

    def __init__(self, mission_config, connection, vehicle, frames):

        self.mission_config = mission_config
        self.connection = connection
        self.vehicle = vehicle
        self.frames = frames

        self.ascent_trajectory = ReferenceAscentTrajectory()

        self.telemetry = Telemetry(self.connection, self.vehicle, self.frames, 'launch')
        self.commanding = Commanding(self.vehicle)
        self.display = VehicleDisplay('ascent', self.telemetry, self.commanding, self.connection, self.frames, self.ascent_trajectory)

    def mission(self):

        attitude_control = AttitudeControl(self.connection,
                                           self.vehicle,
                                           self.telemetry,
                                           self.frames,
                                           target_euler=(0, -90, 0))

        self.commanding.update(0, 0, 0, throttle=1.0)

        self.display.update(attitude_control)

        start = time.time()
        while time.time() - start<5:
            self.display.update(attitude_control)
            pass

        self.commanding.stage()  # Start Main Engines

        desired_pitch = 0
        pitch_offset_due_to_error = 0

        self.vehicle.control.legs = False

        time.sleep(2)  # Give Time for Rocket to Leave Pad Before Initializing Attitude Control

        # Step 1: Ascent
        while self.vehicle.orbit.apoapsis - G.KERBIN_RADIUS < np.int(self.mission_config.get('PARAMETERS', 'target_altitude')):

            self.telemetry.update()
            self.display.update(attitude_control)

            #  Pitch is commanded to dr/a ratio plus an error to optimal trajectory
            if self.telemetry.pos.v[-1][2] < 36000:  # This is the end of the specified trajectory
                desired_pitch = np.degrees(np.arctan(self.ascent_trajectory.get_dr_dot(self.telemetry.pos.v[-1][2])))
                pitch_offset_due_to_error = 0.015 * (self.ascent_trajectory.get_dr(self.telemetry.pos.v[-1][2]) - self.telemetry.pos.v[-1][0])

            roll, pitch, yaw = attitude_control.update(target_euler=(0, -90 + desired_pitch + pitch_offset_due_to_error, 0))

            self.commanding.update(roll, pitch, yaw, throttle=1.0)

        self.commanding.update(0, 0, 0, 0)

        time.sleep(3)
