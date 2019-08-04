import krpc
from AscentController import AscentController
from CircularizationController import CircularizationController
from LandingController import LandingController
import configparser
from sys import argv
from pyquaternion import Quaternion
import numpy as np
from Frames import *
import time


def main():

    # Get Mission Configuration
    mission_file = "Mission1.ini"
    mission_config = configparser.ConfigParser()
    mission_config.sections()
    mission_config.read(mission_file)

    mission_phase = argv[1]

    # Get Connection
    connection = krpc.connect()

    # Revert to Start
    # connection.space_center.launch_vessel_from_vab('New Glenn v2')
    # time.sleep(3)

    # Define vehicle
    vehicle = connection.space_center.active_vessel

    # Establish frames
    vehicle_relative_frame = vehicle.reference_frame
    launch_relative_frame = vehicle.orbit.body.reference_frame

    frames = Frames(connection)

    frames.add_frame(vehicle_relative_frame,
                     'vehicle',
                     (0, 0, 0),
                     (0.7071068, 0, 0, 0.7071068),
                     (0, 0, 0),
                     (0, 0, 0))

    frames.add_frame(launch_relative_frame,
                     'launch',
                     (159783.82381776502, -1018.4263901739021, -578426.6428425296),  # These are hardcoded to the launchpad values
                     (0.00169008, 0.99115833, 0.02235677, 0.13077702),
                     (0, 0, 0),
                     (0, 0, 0))

    # ## This is used for landing barge coordinates ##
    l_pos = vehicle.position(launch_relative_frame)
    l_rot = vehicle.rotation(launch_relative_frame)
    l_rot = [l_rot[3], l_rot[0], l_rot[1], l_rot[2]]
    l_rot = Quaternion(l_rot) * Quaternion([0.5, -0.5, 0.5, 0.5])
    # ################################################

    # frames.add_frame(launch_relative_frame,
    #                  'landing',
    #                  (272167.4350489719, 4319.0206644535065, -534704.2939413232),
    #                  (-0.00346168, 0.97241164, 0.00098668, 0.23324373),
    #                  (0, 0, 0),
    #                  (0, 0, 0))

    frames.add_frame(launch_relative_frame,
                     'landing',
                     (159783.82381776502, -1018.4263901739021, -578426.6428425296),
                     (0.00169008, 0.99115833, 0.02235677, 0.13077702),
                     (0, 0, 0),
                     (0, 0, 0))

    # x = connection.drawing.add_line((0, 0, 0), (100000, 0, 0), frames.frames['landing'].relative)
    # x.color = (255, 0, 0)
    # x.thickness = 10
    # y = connection.drawing.add_line((0, 0, 0), (0, 100000, 0), frames.frames['landing'].relative)
    # y.color = (0, 255, 0)
    # y.thickness = 10
    # z = connection.drawing.add_line((0, 0, 0), (0, 0, 1000000), frames.frames['landing'].relative)
    # z.color = (0, 0, 255)
    # z.thickness = 10
    #
    # land = connection.drawing.add_line((0, 0, 0), (0, 0, 100000), frames.frames['landing'].relative)
    # land.thickness = 10

    if mission_phase == 'ascent':
        vehicle_controller = AscentController(mission_config, connection, vehicle, frames.frames)
        vehicle_controller.mission()

    if mission_phase == 'circularization':
        vehicle_controller = CircularizationController(mission_config, connection, vehicle, frames.frames)
        vehicle_controller.mission()

    if mission_phase == 'landing':
        vehicle_controller = LandingController(mission_config, connection, vehicle, frames.frames)
        vehicle_controller.mission()


if __name__ == "__main__":
    main()




