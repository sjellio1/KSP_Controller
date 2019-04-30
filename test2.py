from AscentTrajectory import Trajectory
import math


traj = Trajectory()

for i in range(0, 30000, 100):
    print(i, traj.get_dr_dot(i))