import numpy as np
import matplotlib.pyplot as plt
from GLOBALS import GLOBALS as G
from PID import PID
from Telemetry import Telemetry
from Display import VehicleDisplay


class CircularizationController:

    def __init__(self, mission_config, connection, vehicle, launch_frame):

        self.mission_config = mission_config
        self.connection = connection
        self.vehicle = vehicle
        self.launch_frame = launch_frame

        self.display = VehicleDisplay()
        self.telemetry = Telemetry(self.connection, self.vehicle, self.launch_frame)

    def mission(self):

        roll_pid = PID(0, 0.005, 0, 0.00015)
        pitch_pid = PID(0, 0.01, 0, 0.001, 0.03)
        yaw_pid = PID(0, 0.01, 0, 0.001)

        self.vehicle.control.throttle = 0.0

        while True:
            pass
