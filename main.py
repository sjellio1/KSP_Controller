import krpc
from AscentController import AscentController
from CircularizationController import CircularizationController
from LandingController import LandingController
from ConfigParser import ConfigParser
from sys import argv
from pyquaternion import Quaternion
import numpy as np


def main():

    # Get Mission Configuration
    mission_file = "Mission1.ini"
    mission_config = ConfigParser()
    mission_config.sections()
    mission_config.read(mission_file)

    mission_phase = argv[1]

    # Get Connection
    connection = krpc.connect()

    # Define Vehicle
    vehicle = connection.space_center.active_vessel

    # Define Reference Frame
    vehicle_frame = vehicle.orbit.body.reference_frame
    # These are hardcoded to the launchpad values
    launch_frame = connection.space_center.ReferenceFrame.create_relative(vehicle_frame,
                                                                          (159783.82381776502, -1018.4263901739021, -578420.1601801577),
                                                                          (0.5641519723244943, 0.5607155213968196, -0.42753416929035815, -0.4295873485013165),
                                                                          (0, 0, 0),
                                                                          (0, 0, 0))

    if mission_phase == 'ascent':
        vehicle_controller = AscentController(mission_config, connection, vehicle, launch_frame)
        vehicle_controller.mission()

    if mission_phase == 'circularization':
        vehicle_controller = CircularizationController(mission_config, connection, vehicle, launch_frame)
        vehicle_controller.mission()

    if mission_phase == 'landing':
        vehicle_controller = LandingController(mission_config, connection, vehicle, launch_frame)
        vehicle_controller.mission()


if __name__ == "__main__":
    main()




