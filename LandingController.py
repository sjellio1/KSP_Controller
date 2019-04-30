import time
import numpy as np
import matplotlib.pyplot as plt
from GLOBALS import GLOBALS as G
from pyquaternion import Quaternion
from PID import PID
from Telemetry import Telemetry
from Display import VehicleDisplay
from AttitudeControl import AttitudeControl
from SuicideBurnControl import SuicideBurnControl



class LandingController:

    def __init__(self, mission_config, connection, vehicle, launch_frame):

        self.mission_config = mission_config
        self.connection = connection
        self.vehicle = vehicle
        self.launch_frame = launch_frame

        self.display = VehicleDisplay()
        self.telemetry = Telemetry(self.connection, self.vehicle, self.launch_frame)

    def mission(self):

        target_euler = (0, -90, np.degrees(np.arctan2(-self.telemetry.pos.v[-1][0], -self.telemetry.pos.v[-1][2])))  # This points back to base

        time.sleep(5)

        self.vehicle.control.rcs = True
        attitude_control = AttitudeControl(self.connection,
                                           self.vehicle,
                                           self.telemetry,
                                           self.launch_frame,
                                           target_euler=target_euler)

        # Step 1: Flip to Target Thrust Vector
        while np.linalg.norm([attitude_control.q_error.imaginary[0], attitude_control.q_error.imaginary[1]]) > 0.05 or np.linalg.norm(self.telemetry.ome.v[-1]) > 0.02:

            target_euler = (0, -90, np.degrees(np.arctan2(-self.telemetry.pos.v[-1][0], -self.telemetry.pos.v[-1][2])))

            self.telemetry.update(self.vehicle.control)
            self.display.update(self.telemetry,
                                attitude_control.roll_pid,
                                attitude_control.pitch_pid,
                                attitude_control.yaw_pid)

            roll, pitch, yaw = attitude_control.update(target_euler=target_euler)

            self.vehicle.control.roll = roll
            self.vehicle.control.pitch = pitch
            self.vehicle.control.yaw = yaw

        # Step 2: Fire to KSP Launch Pad
        # TODO: Make this a function of predicted end point
        burn_back_start = time.time()
        while time.time() - burn_back_start < 25:

            target_euler = (0, -90, np.degrees(np.arctan2(-self.telemetry.pos.v[-1][0], -self.telemetry.pos.v[-1][2])))

            self.telemetry.update(self.vehicle.control)
            self.display.update(self.telemetry,
                                attitude_control.roll_pid,
                                attitude_control.pitch_pid,
                                attitude_control.yaw_pid)

            roll, pitch, yaw = attitude_control.update(target_euler=target_euler)

            self.vehicle.control.roll = roll
            self.vehicle.control.pitch = pitch
            self.vehicle.control.yaw = yaw
            if time.time() - burn_back_start < 3:  # This lets attitude control stabilize
                self.vehicle.control.throttle = 0.05
            else:
                self.vehicle.control.throttle = 0.4
        self.vehicle.control.throttle = 0

        # Step 3: Wait Until In Atmosphere for Corrections
        while self.vehicle.flight().surface_altitude > 50000:

            self.telemetry.update(self.vehicle.control)
            self.display.update(self.telemetry,
                                attitude_control.roll_pid,
                                attitude_control.pitch_pid,
                                attitude_control.yaw_pid)

            roll, pitch, yaw = attitude_control.update(target_euler=(0, 0, 0))

            self.vehicle.control.roll = roll
            self.vehicle.control.pitch = pitch
            self.vehicle.control.yaw = yaw

        # Step 4: Suicide Burn
        throttle_control = SuicideBurnControl(self.connection,
                                              self.vehicle,
                                              self.telemetry,
                                              self.launch_frame)
        for brake in self.vehicle.parts.control_surfaces:
            brake.deployed = True
        while self.vehicle.flight().surface_altitude > 0:

            self.telemetry.update(self.vehicle.control)
            self.display.update(self.telemetry,
                                attitude_control.roll_pid,
                                attitude_control.pitch_pid,
                                attitude_control.yaw_pid)

            throttle, desired_pitch_angle, desired_yaw_angle = throttle_control.update()
            roll, pitch, yaw = attitude_control.update(target_euler=(0, desired_pitch_angle, desired_yaw_angle))

            self.vehicle.control.roll = roll
            self.vehicle.control.pitch = pitch
            self.vehicle.control.yaw = yaw
            self.vehicle.control.throttle = throttle




        self.vehicle.control.throttle = 0
        print("Landed")


    def get_landing_location(self):
        self.vehicle = self.connection.space_center.active_vessel
        radius = self.vehicle.orbit.body.equatorial_radius
        TA = self.vehicle.orbit.true_anomaly_at_radius(radius)
        TA = TA-np.pi  # look on the negative (descending) side of the orbit
        impact_time = self.vehicle.orbit.ut_at_true_anomaly(TA)
        impact_place = self.vehicle.orbit.position_at(impact_time, self.launch_frame)