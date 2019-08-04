from pyquaternion import Quaternion


class Frames:
    def __init__(self, connection):
        self.connection = connection
        self.frames = {}

    def add_frame(self, relative, name, position, rotation, velocity, angular_velocity):
        new_frame = Frame(self.connection, relative, position, rotation, velocity, angular_velocity)
        self.frames[name] = new_frame


class Frame:
    def __init__(self, connection, relative, position, rotation, velocity, angular_velocity):
        self.relative = connection.space_center.ReferenceFrame.create_relative(relative,
                                                                               position,
                                                                               [rotation[1], rotation[2], rotation[3], rotation[0]],  # Using KRPC quaternion format
                                                                               velocity,
                                                                               angular_velocity)
        self.position = position
        self.rotation = Quaternion(rotation)
        self.velocity = velocity
        self.angular_velocity = angular_velocity
