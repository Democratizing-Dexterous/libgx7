import numpy as np
from robot import GX7
from hardware.can import VCICAN


can = VCICAN()
can.init_can()
FREQ = 200  # Hz
robot = GX7(can, can_channel=1, freq=FREQ, control_mode="pvt")
robot.setup()
robot.run()  # Start the robot thread
