from GLOBALS import GLOBALS as G
import time
import numpy as np
import math
from pyquaternion import Quaternion

class Telemetry:
    def __init__(self, connection, vehicle, frames, reference):

        self.connection = connection
        self.vehicle = vehicle
        self.frames = frames
        self.reference = reference

        # Launch Frame
        self.pos = TelemetryPoint()  # Position m
        self.vel = TelemetryPoint()  # Velocity m/s
        self.rot = TelemetryPoint()  # Rotation Quaternion
        self.ome = TelemetryPoint()  # Rotation Rate rad/s

        self.update()

    def update(self):

        # If position is fresh, append it and timestamp it
        position = self.vehicle.position(self.frames[self.reference].relative)
        if not self.pos.v or position != self.pos.v[-1]:
            timestamp = time.time()
            self.pos.v.append(position)
            self.pos.t.append(timestamp)

        velocity = self.vehicle.velocity(self.frames[self.reference].relative)
        if not self.vel.v or velocity != self.vel.v[-1]:
            timestamp = time.time()
            self.vel.v.append(velocity)
            self.vel.t.append(timestamp)

        # If rotation is fresh, append it and timestamp it ( this in the vehicle frame)
        rotation = self.vehicle.rotation(self.frames[self.reference].relative)
        rotation = Quaternion(rotation[3], rotation[0], rotation[1], rotation[2])
        rotation = rotation * self.frames['vehicle'].rotation
        if not self.rot.v or rotation != self.rot.v[-1]:
            timestamp = time.time()
            self.rot.v.append(rotation)
            self.rot.t.append(timestamp)

        # If rotational rate is fresh, append it and timestamp it ( this is in the vehicle frame)
        omega = self.vehicle.angular_velocity(self.frames[self.reference].relative)
        omega = self.connection.space_center.transform_direction(omega, self.frames[self.reference].relative, self.frames['vehicle'].relative)
        if not self.ome.v or omega != self.ome.v[-1]:
            timestamp = time.time()
            self.ome.v.append(omega)
            self.ome.t.append(timestamp)

    @staticmethod
    def q2e(q):

        t0 = +2.0 * (q[0] * q[1] + q[2] * q[3])
        t1 = +1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])
        roll = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (q[0] * q[2] - q[3] * q[1])
        pitch = math.degrees(math.asin(t2))

        t3 = +2.0 * (q[0] * q[3] + q[1] * q[2])
        t4 = +1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])
        yaw = math.degrees(math.atan2(t3, t4))

        return [roll, pitch, yaw]

    @staticmethod
    def e2q(e):

        roll = math.radians(e[0])
        pitch = math.radians(e[1])
        yaw = math.radians(e[2])

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q0 = cy * cp * cr + sy * sp * sr
        q1 = cy * cp * sr - sy * sp * cr
        q2 = sy * cp * sr + cy * sp * cr
        q3 = sy * cp * cr - cy * sp * sr

        return [q0, q1, q2, q3]

    @staticmethod
    def alt(position):
        return position[2]

    @staticmethod
    def dr(position):
        return np.sign(position[0]) * np.linalg.norm([position[0], position[1]])


class TelemetryPoint:

    def __init__(self):
        self.v = []         # Value
        self.t = []         # Timestamp
