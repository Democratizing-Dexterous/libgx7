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

start_t = time.time()


while True:
    robot.setPVT([-0.6, 0.1, -0.3], [1, 1, 1], [0.1, 0.1, 0.1])
    time.sleep(3)
    
    robot.setPVT([0.6, 1.2, 1.3], [1, 1, 1], [0.1, 0.1, 0.1])
    time.sleep(3)

# robot.switch_mit()
# print(robot.getP())


# time.sleep(3)
# print(robot.getP())
