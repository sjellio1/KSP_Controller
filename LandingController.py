import numpy as np
import math
from Telemetry import Telemetry
from Commanding import Commanding
from Display import *
from AttitudeControl import AttitudeControl
from ReferenceTrajectory import ReferenceLandingTrajectory
from SuicideBurnControl import SuicideBurnControl
from MPC import MPC
import krpc


class LandingController:

    def __init__(self, mission_config, connection, vehicle, frames):

        self.mission_config = mission_config
        self.connection = connection
        self.vehicle = vehicle
        self.frames = frames

        self.telemetry = Telemetry(self.connection, self.vehicle, self.frames, 'landing')
        self.commanding = Commanding(self.vehicle)
        #  self.display = VehicleDisplay('landing', self.telemetry, self.commanding, self.connection, self.frames)
        self.land_display = None

        self.time_steps = 5

        self.predicted_landing = connection.drawing.add_line((0, 0, 0), (0, 0, 0), frames['landing'].relative)
        self.predicted_landing.color = (255, 255, 0)
        self.predicted_landing.thickness = 20

    def mission(self):

        attitude_control = self.vehicle.auto_pilot

        # Step 1: Flip to Target Thrust Vector
        self.vehicle.control.rcs = True
        predicted_landing = self.get_predicted_landing_location()
        attitude_control.reference_frame = self.frames['landing'].relative
        attitude_control.disengage()
        attitude_control.engage()
        attitude_control.target_direction = (-1, 0, 0)
        attitude_control.stopping_time = (5, 5, 5)

        time.sleep(0.5)

        while attitude_control.error > 1:

            self.telemetry.update()

            predicted_landing = self.get_predicted_landing_location()

            x_dir = -predicted_landing[0] / np.linalg.norm(predicted_landing)
            y_dir = -predicted_landing[1] / np.linalg.norm(predicted_landing)

            attitude_control.target_direction = (x_dir, y_dir, 0)

            self.commanding.update(throttle=0)

        # Step 2: Fire to Zero Out Landing
        print("Slew Achieved")
        self.commanding.update(0, 0, 0, throttle=0.3)
        time.sleep(1)
        while np.linalg.norm(predicted_landing) > 100 and predicted_landing[0] > 0:

            self.telemetry.update()

            predicted_landing = self.get_predicted_landing_location()

            x_dir = -predicted_landing[0] / np.linalg.norm(predicted_landing)
            y_dir = -predicted_landing[1] / np.linalg.norm(predicted_landing)

            attitude_control.target_direction = (x_dir, y_dir, 0)

            if np.linalg.norm(predicted_landing) > 5000:
                self.commanding.update(throttle=0.3)
            else:
                self.commanding.update(throttle=0.05)

        # Step 3: Wait Until Landing: Point Retrograde
        self.commanding.update(throttle=0)
        self.vehicle.control.rcs = False
        self.predicted_landing.visible = False
        while self.telemetry.pos.v[-1][2] > 60000:  # Wait to turn on RCS
            self.telemetry.update()
        self.vehicle.control.rcs = True

        while self.telemetry.pos.v[-1][2] > 5000:
            self.telemetry.update()

            retrograde = self.get_retrograde()
            attitude_control.target_direction = retrograde

            self.commanding.update(throttle=0)

        # Step 4: Landing Time
        self.commanding.update(0, 0, 0, throttle=0)
        landing_control = SuicideBurnControl(self.connection, self.vehicle, self.telemetry, self.frames)
        self.land_display = LandingDisplay(self.telemetry, self.commanding, self.connection, self.frames)
        while self.telemetry.pos.v[-1][2] > 300:

            self.telemetry.update()
            self.land_display.update(landing_control)

            throttle = landing_control.update()

            retrograde = self.get_retrograde()
            attitude_control.target_direction = retrograde

            self.commanding.update(throttle=throttle)

        self.commanding.update(0, 0, 0, throttle=0)

        while self.vehicle.situation.name is not 'landed':

            self.telemetry.update()
            self.land_display.update(landing_control)

            throttle = landing_control.update()

            retrograde = self.get_retrograde()
            attitude_control.target_direction = (0, 0, 1)

            self.commanding.update(throttle=throttle)

        self.commanding.update(0, 0, 0, throttle=0)
        self.vehicle.control.rcs = False
        print(self.telemetry.pos.v[-1][2])

        print("Landed")

        input("Press Enter")

    def get_predicted_landing_location(self):
        x0 = self.telemetry.pos.v[-1][0]
        y0 = self.telemetry.pos.v[-1][1]
        z0 = self.telemetry.pos.v[-1][2]

        dx0 = self.telemetry.vel.v[-1][0]
        dy0 = self.telemetry.vel.v[-1][1]
        dz0 = self.telemetry.vel.v[-1][2]

        g = -9.81

        t = np.roots([0.5*g, dz0, z0])
        dt = np.max(t)

        aerodynamic_correction_constant = 1.85
        # Higher is shorter

        x1 = x0 + dx0*dt * aerodynamic_correction_constant
        y1 = y0 + dy0*dt * aerodynamic_correction_constant

        self.predicted_landing.start = (x1, y1, -30000)
        self.predicted_landing.end = (x1, y1, 300000)

        return x1, y1, 0

    def get_retrograde(self):
        velocity = self.telemetry.vel.v[-1]
        retrograde = (-velocity[0], -velocity[1], np.abs(velocity[2])) / np.linalg.norm(velocity)

        return retrograde

    def get_mpc_states_targets(self, compensated_trajectory):

        pointing_vector = self.telemetry.rot.v[-1].rotate((1, 0, 0))

        pitch = math.atan2(pointing_vector[0], pointing_vector[2])
        yaw = math.atan2(pointing_vector[1], pointing_vector[2])

        states = np.transpose(np.array([
            self.telemetry.pos.v[-1][0],
            self.telemetry.vel.v[-1][0],
            self.telemetry.pos.v[-1][1],
            self.telemetry.vel.v[-1][1],
            self.telemetry.pos.v[-1][2],
            self.telemetry.vel.v[-1][2],
            pitch,
            self.telemetry.ome.v[-1][1],
            yaw,
            self.telemetry.ome.v[-1][2],
        ]))

        targets = np.array([
            compensated_trajectory[0, :],
            compensated_trajectory[1, :],
            compensated_trajectory[2, :],
            compensated_trajectory[3, :],
            compensated_trajectory[4, :],
            compensated_trajectory[5, :],
            np.zeros((self.time_steps+1,)),
            np.zeros((self.time_steps+1,)),
            np.zeros((self.time_steps+1,)),
            np.zeros((self.time_steps+1,)),
        ])

        return states, targets




