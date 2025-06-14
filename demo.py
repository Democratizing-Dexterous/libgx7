import threading
import time
import numpy as np

from robot.robot import Robot
from utils.utils import MotorStatus

FREQ = 300 # Hz
robot = Robot(FREQ)
global_motors_status = [MotorStatus()]*robot.robot_motors.num_motors
robot.load_global_motors_status(global_motors_status)

threading_robot_run = threading.Thread(target=robot.run)
threading_robot_run.daemon = True
threading_robot_run.start()

dt = 1/FREQ

start_t = time.time()
while True:
    robot.setPVT([0.5*np.sin(3*(time.time()-start_t))], [1], [0.3])
    time.sleep(dt)

    if time.time() - start_t > 3:
        break

# robot.switch_mit()
# print(robot.getP())



# time.sleep(3)
# print(robot.getP())




