from GLOBALS import GLOBALS as G
import time
import matplotlib.pyplot as plt
import numpy as np
from pyquaternion import Quaternion


class VehicleDisplay:

    def __init__(self, ascent_trajectory=None):

        self.ascent_trajectory = ascent_trajectory

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
        self.data.rc = []
        self.data.rg = []
        self.data.pc = []
        self.data.pg = []
        self.data.yc = []
        self.data.yg = []

        self.t0 = time.time()
        self.last_update = 0
        plt.show(block=False)

        self.count = 0

    def update(self, telemetry, roll_PID, pitch_PID, yaw_PID):
        if time.time()-self.last_update > G.DISPLAY_PERIOD:

            self.data.rc.append([roll_PID.last_value, roll_PID.target])
            self.data.rg.append([roll_PID.p_comp, roll_PID.i_comp, roll_PID.d_comp])
            self.data.pc.append([pitch_PID.last_value, pitch_PID.target])
            self.data.pg.append([pitch_PID.p_comp, pitch_PID.i_comp, pitch_PID.d_comp])
            self.data.yc.append([yaw_PID.last_value, yaw_PID.target])
            self.data.yg.append([yaw_PID.p_comp, yaw_PID.i_comp, yaw_PID.d_comp])


            # Position Plot
            self.r_disp.clear()
            self.r_disp.plot([t - self.t0 for t in telemetry.pos.t], telemetry.pos.v)
            self.r_disp.legend(['x', 'y', 'z'])

            # Rotation Plot
            self.q_disp.clear()
            self.q_disp.plot([t - self.t0 for t in telemetry.rot.t], telemetry.rot.v)
            self.q_disp.legend(['q0', 'q1', 'q2', 'q3'])

            # Downrange Plot
            if self.ascent_trajectory is not None:
                self.t_disp.clear()
                self.t_disp.plot(self.ascent_trajectory.downrange, self.ascent_trajectory.altitude)
                self.t_disp.plot([telemetry.dr(p) for p in telemetry.pos.v], [telemetry.alt(p) for p in telemetry.pos.v])
                self.t_disp.legend(['Target', 'Actual'])
                self.t_disp.axis('equal')

            # Command Plot
            self.c_disp.clear()
            self.c_disp.plot(telemetry.roll_command)
            self.c_disp.plot(telemetry.pitch_command)
            self.c_disp.plot(telemetry.yaw_command)
            self.c_disp.plot(telemetry.throttle_command)
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

            self.last_update = time.time()
            plt.pause(0.05)


class BoosterDisplay:

    def __init__(self):
        pass

    def update(self, telemetry, roll_PID, pitch_PID, yaw_PID):
        pass
