import krpc
import time

connection = krpc.connect()
vehicle = connection.space_center.active_vessel

vehicle_relative_frame = vehicle.reference_frame
launch_relative_frame = vehicle.orbit.body.reference_frame

ref_frame = connection.space_center.ReferenceFrame.create_relative(
    vehicle.orbit.body.reference_frame,
    (159783.82381776502, -1018.4263901739021, -578426.6428425296),
    (0.99115833, 0.02235677, 0.13077702, 0.00169008),
    (0, 0, 0),
    (0, 0, 0))

start = time.time()
vehicle.control.activate_next_stage()
vehicle.control.throttle = 0.8

while True:
    time.sleep(0.5)
    print(time.time()-start, vehicle.position(ref_frame))
