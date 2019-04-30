from GLOBALS import GLOBALS as G
import time
import numpy as np
import math
from pyquaternion import Quaternion

class Telemetry:
    def __init__(self, connection, vehicle, launch_frame):

        self.connection = connection
        self.vehicle = vehicle
        self.launch_frame = launch_frame

        t0 = time.time()
        self.pos = TelemetryPoint()
        self.vel = TelemetryPoint()
        self.rot = TelemetryPoint()
        self.ome = TelemetryPoint()

        self.roll_command = []
        self.pitch_command = []
        self.yaw_command = []
        self.throttle_command = []

        self.update(self.vehicle.control)

    def update(self, commanding):

        self.roll_command.append(commanding.roll)
        self.pitch_command.append(commanding.pitch)
        self.yaw_command.append(commanding.yaw)
        self.throttle_command.append(commanding.throttle)

        # If position is fresh, append it and timestamp it
        position = self.vehicle.position(self.launch_frame)
        if not self.pos.v or position != self.pos.v[-1]:
            timestamp = time.time()
            self.pos.v.append(position)
            self.pos.t.append(timestamp)

        velocity = self.vehicle.velocity(self.launch_frame)
        if not self.vel.v or velocity != self.vel.v[-1]:
            timestamp = time.time()
            self.vel.v.append(velocity)
            self.vel.t.append(timestamp)

        # If rotation is fresh, append it and timestamp it
        rotation = self.vehicle.rotation(self.launch_frame)
        rotation = [rotation[3], rotation[0], rotation[1], rotation[2]]  # Who the fucks idea was this
        #rotation = Quaternion(rotation) * Quaternion(matrix=np.matrix('[0, 1.0, 0; 1.0, 0, 0; 0, 0, -1.0]'))  # Convert to vehicle frame
        if not self.rot.v or rotation != self.rot.v[-1]:
            timestamp = time.time()
            self.rot.v.append(rotation)
            self.rot.t.append(timestamp)

        # If rotational rate is fresh, append it and timestamp it
        omega = self.vehicle.angular_velocity(self.launch_frame)  # ( Thanks a fucking lot krpc for this one (left handed? tf))
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

        roll = e[0]
        pitch = e[1]
        yaw = e[2]

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
        return position[1]

    @staticmethod
    def dr(position):
        return -np.sign(position[2]) * np.linalg.norm([position[0], position[2]])


class TelemetryPoint:

    def __init__(self):
        self.v = []         # Value
        self.t = []         # Timestamp
