from GLOBALS import GLOBALS as G
import time
import matplotlib.pyplot as plt
import numpy as np


class VehicleDisplay:

    def __init__(self, phase, telemetry, commanding, connection, frames, ascent_trajectory=None):

        self.phase = phase
        self.telemetry = telemetry
        self.commanding = commanding
        self.connection = connection
        self.frames = frames

        self.ascent_trajectory = ascent_trajectory

        # Plots
        self.fig = plt.figure(figsize=(7, 8))
        vertical = 5
        horizontal = 2
        self.r_disp = self.fig.add_subplot(vertical, horizontal, 1)
        self.q_disp = self.fig.add_subplot(vertical, horizontal, 2)
        self.t_disp = self.fig.add_subplot(vertical, horizontal, 3)
        self.c_disp = self.fig.add_subplot(vertical, horizontal, 4)

        self.rc_disp = self.fig.add_subplot(vertical, horizontal, 5)
        self.rg_disp = self.fig.add_subplot(vertical, horizontal, 6)
        self.pc_disp = self.fig.add_subplot(vertical, horizontal, 7)
        self.pg_disp = self.fig.add_subplot(vertical, horizontal, 8)
        self.yc_disp = self.fig.add_subplot(vertical, horizontal, 9)
        self.yg_disp = self.fig.add_subplot(vertical, horizontal, 10)

        self.data = lambda: 0
        self.data.e = []
        self.data.rc = []
        self.data.rg = []
        self.data.pc = []
        self.data.pg = []
        self.data.yc = []
        self.data.yg = []

        # Annotations

        # Attitude Reference
        self.attitude_reference = connection.drawing.add_line((0, 0, 0), (0, 0, 0), self.frames['vehicle'].relative)

        if phase is 'landing':
            # Landing Zone
            scale = 3000.0
            vertices = [(scale, scale, 0.0),
                        (scale, -scale, 0.0),
                        (-scale, -scale, 0.0),
                        (-scale, scale, 0.0)]
            landing_zone = self.connection.drawing.add_polygon(vertices, self.frames['landing'].relative)
            landing_zone.color = (250, 0, 0)
            landing_zone.thickness = 40

            # Predicted Landing
            self.predicted_landing = connection.drawing.add_line((0, 0, 0), (0, 0, 0), frames['landing'].relative)
            self.predicted_landing.color = (255, 255, 0)
            self.predicted_landing.thickness = 20

        self.t0 = time.time()
        self.last_update = 0
        plt.show(block=False)

        self.count = 0

    def update(self, attitude_control, predicted_landing=None):

        if time.time()-self.last_update > G.DISPLAY_PERIOD:

            roll_pid = attitude_control.roll_pid
            pitch_pid = attitude_control.pitch_pid
            yaw_pid = attitude_control.yaw_pid

            # Plots
            self.data.e.append(attitude_control.error)
            self.data.rc.append([roll_pid.last_value, roll_pid.target])
            self.data.rg.append([roll_pid.p_comp, roll_pid.i_comp, roll_pid.d_comp])
            self.data.pc.append([pitch_pid.last_value, pitch_pid.target])
            self.data.pg.append([pitch_pid.p_comp, pitch_pid.i_comp, pitch_pid.d_comp])
            self.data.yc.append([yaw_pid.last_value, yaw_pid.target])
            self.data.yg.append([yaw_pid.p_comp, yaw_pid.i_comp, yaw_pid.d_comp])

            # Position Plot
            self.r_disp.clear()
            self.r_disp.plot([t - self.t0 for t in self.telemetry.pos.t], self.telemetry.pos.v)
            self.r_disp.legend(['x', 'y', 'z'])

            # Roll Control Plots
            self.q_disp.clear()
            self.q_disp.plot(self.data.e)
            self.q_disp.legend(['r', 'p', 'y'])

            # Downrange Plot
            if self.ascent_trajectory is not None:
                self.t_disp.clear()
                self.t_disp.plot(self.ascent_trajectory.downrange, self.ascent_trajectory.altitude)
                self.t_disp.plot([self.telemetry.dr(p) for p in self.telemetry.pos.v], [self.telemetry.alt(p) for p in self.telemetry.pos.v])
                self.t_disp.legend(['Target', 'Actual'])
                self.t_disp.axis('equal')

            # Command Plot
            self.c_disp.clear()
            self.c_disp.plot(self.commanding.roll)
            self.c_disp.plot(self.commanding.pitch)
            self.c_disp.plot(self.commanding.yaw)
            self.c_disp.plot(self.commanding.throttle)
            self.c_disp.legend(['Roll', 'Pitch', 'Yaw', 'Thrust'])
            self.c_disp.set_ylim((-1, 1))

            # Roll Control Plots
            self.rc_disp.clear()
            self.rg_disp.clear()
            self.rc_disp.plot(self.data.rc)
            self.rc_disp.legend(['value', 'target'])
            self.rc_disp.set_ylabel('Roll Control')
            self.rg_disp.plot(self.data.rg)
            self.rg_disp.legend(['P', 'I', 'D'])

            # Pitch Control Plots
            self.pc_disp.clear()
            self.pg_disp.clear()
            self.pc_disp.plot(self.data.pc)
            self.pc_disp.legend(['value', 'target'])
            self.pc_disp.set_ylabel('Pitch Control')
            self.pg_disp.plot(self.data.pg)
            self.pg_disp.legend(['P', 'I', 'D'])

            # Yaw Control Plots
            self.yc_disp.clear()
            self.yg_disp.clear()
            self.yc_disp.plot(self.data.yc)
            self.yc_disp.legend(['value', 'target'])
            self.yc_disp.set_ylabel('Yaw Control')
            self.yg_disp.plot(self.data.yg)
            self.yg_disp.legend(['P', 'I', 'D'])

            # Annotations

            # Attitude Reference
            reference_in_landing_frame = self.telemetry.rot.v[-1].inverse * attitude_control.target_quaternion
            self.attitude_reference.end = reference_in_landing_frame.rotate((100, 0, 0))

            if self.phase is 'landing':
                # Predicted Landing
                if predicted_landing is not None:
                    self.predicted_landing.start = (predicted_landing[0], predicted_landing[1], -10000)
                    self.predicted_landing.end = (predicted_landing[0], predicted_landing[1], 300000)
                else:
                    self.predicted_landing.visible = False

            self.last_update = time.time()
            plt.pause(0.05)


class LandingDisplay:

    def __init__(self, telemetry, commanding, connection, frames):

        self.telemetry = telemetry
        self.commanding = commanding
        self.connection = connection
        self.frames = frames

        # Plots
        self.fig = plt.figure(figsize=(7, 4))
        vertical = 1
        horizontal = 2
        self.dzc_disp = self.fig.add_subplot(vertical, horizontal, 1)
        self.dzg_disp = self.fig.add_subplot(vertical, horizontal, 2)

        self.data = lambda: 0
        self.data.dzc = []
        self.data.dzg = []

        self.t0 = time.time()
        self.last_update = 0
        plt.show(block=False)

    def update(self, landing_control):

        if time.time() - self.last_update > G.DISPLAY_PERIOD:

            self.data.dzc.append([landing_control.z, landing_control.dz])
            self.data.dzg.append([landing_control.p_cont, landing_control.d_cont])

            # Thrust Control Plots
            self.dzc_disp.clear()
            self.dzg_disp.clear()
            self.dzc_disp.plot(self.data.dzc)
            self.dzc_disp.legend(['z', 'dz'])
            self.dzc_disp.set_ylabel('Throttle Control')
            self.dzg_disp.plot(self.data.dzg)
            self.dzg_disp.legend(['P', 'D'])

            self.last_update = time.time()
            plt.pause(0.05)