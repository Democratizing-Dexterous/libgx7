import threading
import time

import numpy as np

from robot.robot import Robot
from utils.utils import MotorStatus

FREQ = 300  # Hz
robot = Robot(FREQ)

init_motors_status_params = [[0]*robot.num_dof]*8
global_motors_status = MotorStatus(*init_motors_status_params)
robot.load_global_motors_status(global_motors_status)

threading_robot_run = threading.Thread(target=robot.run)
threading_robot_run.daemon = True
threading_robot_run.start()

dt = 1 / FREQ


time.sleep(1)

# robot.setJPVT([0.3, 1.2, 0.6], [0.5]*3, [0.1]*3)
# start_t = time.time()

# robot.switch_pv()

# while True:
#     robot.setPV([-0.3, 0.2, -0.6], [0.5]*3)
#     time.sleep(3)
#     robot.setPV([0.3, 1.2, 0.6], [0.5]*3)

#     time.sleep(3)

# start_t = time.time()
# while True:
#     t = time.time() - start_t
#     robot.setPVT([1.5*np.sin(2*t), 1.2*np.sin(2*t) + 0.9, 1.2*np.sin(t)], [0.5, 0.5, 0.5], [0.1, 0.1, 0.1])
#     time.sleep(dt)

# robot.switch_mit()
# print(robot.getP())


# time.sleep(3)
# print(robot.getP())
