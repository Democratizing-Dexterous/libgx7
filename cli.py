import numpy as np
from libgx7 import VCICAN, GX7


can = VCICAN()
can.init_can()
FREQ = 100  # Hz
robot = GX7(can, can_channel=1, freq=FREQ, control_mode="pvt", soft_limit=True)
robot.setup()
robot.run()  # Start the robot thread
