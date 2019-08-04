class Commanding:

    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.roll = []
        self.pitch = []
        self.yaw = []
        self.throttle = []

    def update(self, roll=None, pitch=None, yaw=None, throttle=0):

        self.roll.append(roll)
        self.pitch.append(pitch)
        self.yaw.append(yaw)
        self.throttle.append(throttle)

        if roll is not None:
            self.vehicle.control.roll = -roll

        if pitch is not None:
            self.vehicle.control.pitch = pitch

        if yaw is not None:
            self.vehicle.control.yaw = -yaw

        self.vehicle.control.throttle = throttle

    def stage(self):
        self.vehicle.control.activate_next_stage()

    def landing_legs(self, deploy):
        for leg in self.vehicle.parts.legs:
            leg.deployed = deploy
