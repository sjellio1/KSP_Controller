import numpy as np
from GLOBALS import GLOBALS as G
import time
from Telemetry import Telemetry
from Commanding import Commanding
from AttitudeControl import AttitudeControl
from Display import VehicleDisplay
import math


class CircularizationController:

    def __init__(self, mission_config, connection, vehicle, frames):

        self.mission_config = mission_config
        self.connection = connection
        self.vehicle = vehicle
        self.frames = frames

        self.telemetry = Telemetry(self.connection, self.vehicle, self.frames, 'launch')
        self.commanding = Commanding(self.vehicle)
        self.display = VehicleDisplay('ascent', self.telemetry, self.commanding, self.connection, self.frames)

    def mission(self):

        attitude_control = AttitudeControl(self.connection,
                                           self.vehicle,
                                           self.telemetry,
                                           self.frames,
                                           target_euler=(0, 0, 0))

        self.commanding.update(0, 0, 0, throttle=0)

        self.display.update(attitude_control)

        self.commanding.stage()  # Eject Fairings
        self.commanding.stage()  # Start Engines

        time.sleep(1)

        # Step 1: Point Prograde
        pointing_error = 1
        attitude_control.multiplier = 3
        while pointing_error > 0.05 or np.linalg.norm(self.telemetry.ome.v[-1]) > 0.05:

            self.telemetry.update()
            self.display.update(attitude_control)

            pointing_error = attitude_control.pointing_error

            tangent = self.connection.space_center.transform_direction((0, 0, 1), self.frames['launch'].relative, self.vehicle.surface_reference_frame)
            pitch = np.degrees(math.atan2(-tangent[2], tangent[0]))
            roll, pitch, yaw = attitude_control.update(target_euler=(0, pitch, 0))

            self.commanding.update(roll, pitch, yaw, throttle=0.05)

        # Step 2: Wait Until Burn
        while self.telemetry.pos.v[-1][2] < 90000:
            self.telemetry.update()
            self.display.update(attitude_control)

            tangent = self.connection.space_center.transform_direction((0, 0, 1), self.frames['launch'].relative, self.vehicle.surface_reference_frame)
            pitch = np.degrees(math.atan2(-tangent[2], tangent[0]))
            roll, pitch, yaw = attitude_control.update(target_euler=(0, pitch, 0))

            self.commanding.update(roll, pitch, yaw, throttle=0.05)

        # Step 2: Burn
        self.commanding.update(0, 0, 0, 0)
        time.sleep(1)
        self.commanding.update(0, 0, 0, throttle=1)
        time.sleep(2)
        attitude_control.multiplier = 0.5
        attitude_control.pitch_gains = (0.3, 0.05, 0)
        attitude_control.yaw_gains = (0.3, 0.05, 0)
        while self.vehicle.orbit.periapsis < self.vehicle.orbit.body.equatorial_radius + 95000:

            self.telemetry.update()
            self.display.update(attitude_control)

            tangent = self.connection.space_center.transform_direction((0, 0, 1), self.frames['launch'].relative, self.vehicle.surface_reference_frame)
            pitch = np.degrees(math.atan2(-tangent[2], tangent[0]))
            roll, pitch, yaw = attitude_control.update(target_euler=(0, pitch, 0))

            self.commanding.update(roll, pitch, yaw, throttle=1.0)

        self.commanding.update(0, 0, 0, 0)

        time.sleep(3)