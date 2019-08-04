import numpy as np
from pyquaternion import Quaternion


class AttitudeControl:

    def __init__(self, connection, vehicle, telemetry, frames, target_quaternion=None, target_euler=None):

        self.connection = connection
        self.vehicle = vehicle
        self.telemetry = telemetry
        self.frames = frames

        self.pitch_max_torque = 1
        self.yaw_max_torque = 1

        self.roll_max_alpha = 0
        self.pitch_max_alpha = 0
        self.yaw_max_alpha = 0
        self.update_properties()

        self.multiplier = 2
        self.roll_multiplier = 0.5

        self.roll_zero = False

        self.stop_time = 5

        self.error = 0
        self.pointing_error = 0
        if target_quaternion is not None:

            self.target_quaternion = target_quaternion
        else:
            self.target_quaternion = Quaternion(self.telemetry.e2q(target_euler))

        self.roll_pid = PID(0, 0, 0, 0, 1, 1)
        self.pitch_pid = PID(0, 0, 0, 0, 1, 1)
        self.yaw_pid = PID(0, 0, 0, 0, 1, 1)

        self.roll_gains = (0.6, 0.01, 0)
        self.pitch_gains = (0.8, 0.01, 0)
        self.yaw_gains = (0.8, 0.01, 0)

        self.update()

    def update(self, target_quaternion=None, target_euler=None):

        # Update target_quaternion if New is Specified
        if target_quaternion is not None:
            self.target_quaternion = target_quaternion
        if target_euler is not None:
            self.target_quaternion = Quaternion(self.telemetry.e2q(target_euler))

        # Update SC Properties
        self.update_properties()

        # Get the Error
        q_error = self.telemetry.rot.v[-1].conjugate * self.target_quaternion
        error = q_error.axis * q_error.angle

        roll_error = error[0]
        pitch_error = error[1]
        yaw_error = error[2]

        self.error = error
        self.pointing_error = np.linalg.norm((pitch_error, yaw_error))

        roll_alpha = np.min([self.roll_max_alpha, 2])
        pitch_alpha = np.min([self.pitch_max_alpha, 2])
        yaw_alpha = np.min([self.yaw_max_alpha, 2])

        # Create constant rate profile
        desired_roll_dot = np.sign(roll_error) * np.min([self.roll_max_alpha * self.stop_time, self.roll_multiplier * roll_alpha * np.abs(roll_error)])
        desired_pitch_dot = np.sign(pitch_error) * np.min([self.pitch_max_alpha * self.stop_time, self.multiplier * pitch_alpha * np.abs(pitch_error)])
        desired_yaw_dot = np.sign(yaw_error) * np.min([self.yaw_max_alpha * self.stop_time,  self.multiplier * yaw_alpha * np.abs(yaw_error)])

        if self.roll_zero:
            desired_roll_dot = 0

        # Update Gains
        self.roll_pid.p_gain = self.roll_gains[0] / self.roll_max_alpha
        self.roll_pid.i_gain = self.roll_gains[1] / self.roll_max_alpha
        self.roll_pid.d_gain = self.roll_gains[2] / self.roll_max_alpha

        self.pitch_pid.p_gain = self.pitch_gains[0] / self.pitch_max_alpha
        self.pitch_pid.i_gain = self.pitch_gains[1] / self.pitch_max_alpha
        self.pitch_pid.d_gain = self.pitch_gains[2] / self.pitch_max_alpha

        self.yaw_pid.p_gain = self.yaw_gains[0] / self.yaw_max_alpha
        self.yaw_pid.i_gain = self.yaw_gains[1] / self.yaw_max_alpha
        self.yaw_pid.d_gain = self.yaw_gains[2] / self.yaw_max_alpha

        angular_velocity = self.telemetry.ome.v[-1]
        timestamp = self.telemetry.ome.t[-1]

        # Update Controllers
        roll_command = self.roll_pid.update(angular_velocity[0],
                                            timestamp,
                                            desired_roll_dot)

        pitch_command = self.pitch_pid.update(angular_velocity[1],
                                              timestamp,
                                              desired_pitch_dot)

        yaw_command = self.yaw_pid.update(angular_velocity[2],
                                          timestamp,
                                          desired_yaw_dot)

        # Aerodynamic Force Rejection
        flight = self.vehicle.flight()
        pitch_comp = -0.14 * np.rad2deg(flight.angle_of_attack * flight.dynamic_pressure) / self.pitch_max_torque
        yaw_comp = -0.14 * np.rad2deg(flight.sideslip_angle * flight.dynamic_pressure) / self.yaw_max_torque

        pitch_command += pitch_comp
        yaw_command += yaw_comp

        return roll_command, pitch_command, yaw_command

    def update_properties(self):

        spacecraft_inertia = self.vehicle.moment_of_inertia
        spacecraft_inertia = [spacecraft_inertia[1], spacecraft_inertia[0], spacecraft_inertia[2]]  # Convert to Proper Frame

        available_torque = self.vehicle.available_torque
        available_torque = [available_torque[0][1], available_torque[0][0], available_torque[0][2]]  # Convert to Proper Frame

        self.pitch_max_torque = available_torque[1]
        self.yaw_max_torque = available_torque[2]

        self.roll_max_alpha = available_torque[0] / spacecraft_inertia[0]
        self.pitch_max_alpha = available_torque[1] / spacecraft_inertia[1]
        self.yaw_max_alpha = available_torque[2] / spacecraft_inertia[2]


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

            self.i_comp += error * time_difference * self.i_gain
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

