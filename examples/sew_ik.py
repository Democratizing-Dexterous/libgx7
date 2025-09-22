import numpy as np
import time
import sys

sys.path.append("../")
from robot import GX7
from hardware.can import VCICAN


can = VCICAN()
can.init_can()
FREQ = 200  # Hz
robot = GX7(can, can_channel=1, freq=FREQ, control_mode="pvt")
robot.setup()
robot.run()  # Start the robot thread

xyz = [0.3, 0, 0.3]
orientation = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])

psi = np.pi / 2
v = 0.1

solutions = robot.ik(xyz, orientation, psi=psi)

robot.setJPVTs(solutions[0], [0.2] * 7, [0.5] * 7)

time.sleep(5)

while True:
    solutions = robot.ik(xyz, orientation, psi=psi)
    if len(solutions) > 0:
        robot.setJPVTs(solutions[0], [0.1] * 7, [0.5] * 7)

    time.sleep(1 / FREQ)

    psi += v / FREQ

    if psi > np.pi * 0.8:
        v = -v

    if psi < np.pi / 2:
        v = -v
