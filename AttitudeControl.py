import numpy as np
from PID import PID
from pyquaternion import Quaternion


class AttitudeControl:

    def __init__(self, connection, vehicle, telemetry, launch_frame, target_quaternion=None, target_euler=None):

        self.connection = connection
        self.vehicle = vehicle
        self.telemetry = telemetry
        self.launch_frame = launch_frame

        self.roll_max_alpha = 0
        self.pitch_max_alpha = 0
        self.yaw_max_alpha = 0
        self.update_properties()

        if target_quaternion is not None:
            self.target_quaternion = target_quaternion
        else:
            roll_krpc_frame = -np.radians(target_euler[1])
            pitch_krpc_frame = -np.radians(target_euler[0])
            yaw_krpc_frame = np.radians(-target_euler[2])
            self.target_quaternion = Quaternion(self.telemetry.e2q([roll_krpc_frame, pitch_krpc_frame, yaw_krpc_frame]))

        self.roll_pid = PID(0, 0, 0, 0, 1, 1)
        self.pitch_pid = PID(0, 0, 0, 0, 1, 1)
        self.yaw_pid = PID(0, 0, 0, 0, 1, 1)

        self.q_current = None
        self.q_error = None

        self.update()

    def update(self, target_quaternion=None, target_euler=None):

        # Update target_quaternion if New is Specified
        if target_quaternion is not None:
            self.target_quaternion = target_quaternion
        if target_euler is not None:
            roll_krpc_frame = -np.radians(target_euler[1])
            pitch_krpc_frame = -np.radians(target_euler[0])
            yaw_krpc_frame = np.radians(-target_euler[2])
            self.target_quaternion = Quaternion(self.telemetry.e2q([roll_krpc_frame, pitch_krpc_frame, yaw_krpc_frame]))

        # Get Current q and Update the Error
        self.q_current = Quaternion(self.telemetry.rot.v[-1])
        self.q_error = self.q_current * self.target_quaternion.conjugate

        # Update SC Properties
        self.update_properties()

        # Create constant rate profile
        self.q_error.angle = self.q_error.angle if self.q_error.angle < np.pi else self.q_error.angle - 2*np.pi
        errors = self.q_error.axis * self.q_error.angle

        roll_error = errors[2]
        desired_roll_dot = 0.10 * np.sign(roll_error) * np.min([1, np.abs(roll_error)])

        pitch_error = errors[0]
        desired_pitch_dot = 0.10 * np.sign(pitch_error) * np.min([1, np.abs(pitch_error)])

        yaw_error = -errors[1]
        desired_yaw_dot = 0.10 * np.sign(yaw_error) * np.min([1, np.abs(yaw_error)])

        # Compute Angular Velocity
        angular_velocity = self.telemetry.ome.v[-1]
        timestamp = self.telemetry.ome.t[-1]
        angular_velocity = self.connection.space_center.transform_direction(angular_velocity, self.launch_frame, self.vehicle.reference_frame)
        angular_velocity = [-angular_velocity[1], -angular_velocity[0], -angular_velocity[2]]

        p_gain = 0.5
        i_gain = 0
        d_gain = 0

        # Update Gains
        self.roll_pid.p_gain = p_gain / self.roll_max_alpha
        self.roll_pid.i_gain = i_gain / self.roll_max_alpha
        self.roll_pid.d_gain = d_gain / self.roll_max_alpha

        self.pitch_pid.p_gain = p_gain / self.pitch_max_alpha
        self.pitch_pid.i_gain = i_gain / self.pitch_max_alpha
        self.pitch_pid.d_gain = d_gain / self.pitch_max_alpha

        self.yaw_pid.p_gain = p_gain / self.yaw_max_alpha
        self.yaw_pid.i_gain = i_gain / self.yaw_max_alpha
        self.yaw_pid.d_gain = d_gain / self.yaw_max_alpha

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

        return roll_command, pitch_command, yaw_command

    def update_properties(self):

        # roll_previous = self.vehicle.control.roll
        # pitch_previous = self.vehicle.control.pitch
        # yaw_previous = self.vehicle.control.yaw
        # self.vehicle.control.roll = 0
        # self.vehicle.control.pitch = 0
        # self.vehicle.control.yaw = 0

        spacecraft_inertia = self.vehicle.moment_of_inertia
        spacecraft_inertia = [spacecraft_inertia[1], spacecraft_inertia[0], spacecraft_inertia[2]]  # Convert to Proper Frame

        available_torque = self.vehicle.available_torque
        available_torque = [available_torque[0][1], available_torque[0][0], available_torque[0][2]]  # Convert to Proper Frame

        # self.vehicle.control.roll = roll_previous
        # self.vehicle.control.pitch = pitch_previous
        # self.vehicle.control.yaw = yaw_previous

        self.roll_max_alpha = available_torque[0] / spacecraft_inertia[0]
        self.pitch_max_alpha = available_torque[1] / spacecraft_inertia[1]
        self.yaw_max_alpha = available_torque[2] / spacecraft_inertia[2]

    # Giving up on this, thinking angular acceleration retrieved doesnt match actual
    # @staticmethod
    # def get_target_angular_velocity(error, max_alpha):
    #
    #     # stop_time = 5
    #     # max_velocity = stop_time * max_alpha
    #     # slow_threshold = 1
    #     # target_angular_velocity = np.sign(error) * np.min([max_velocity, slow_threshold * np.abs(error) * max_velocity])
    #
    #     target_angular_velocity = np.sign(error) * np.min([0.1, np.abs(error) * 0.1])
    #
    #     return target_angular_velocity

