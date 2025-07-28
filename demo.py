import threading
import time

import numpy as np

from robot.robot import Robot
from utils.utils import MotorStatus

FREQ = 300  # Hz
robot = Robot(FREQ, 'linux')

init_motors_status_params = [[0] * robot.num_dof] * 8
global_motors_status = MotorStatus(*init_motors_status_params)
robot.load_global_motors_status(global_motors_status)

threading_robot_run = threading.Thread(target=robot.run)
threading_robot_run.daemon = True
threading_robot_run.start()

dt = 1 / FREQ


# # time.sleep(1)

# # # robot.setJPVT([0.3, 1.2, 0.6], [0.5]*3, [0.1]*3)
# # # start_t = time.time()

# # # robot.switch_pv()

# # # while True:
# # #     robot.setPV([-0.3, 0.2, -0.6], [0.5]*3)
# # #     time.sleep(3)
# # #     robot.setPV([0.3, 1.2, 0.6], [0.5]*3)

# # #     time.sleep(3)

# j_ps1 = [0.014686808575570254, 0.7303349355306317, 0.14248111696040233, 0.9458686198214696, -0.04482337682154558, 0.478942549782559]

# j_ps = j_ps1
# robot.setJPVT(j_ps, [0.3]*len(j_ps), [0.35]*len(j_ps))
# # # robot.setJPVT([0], [0.5], [0.1])
# time.sleep(5)

# while True:
#     j_ps2 = [0.3587777523460751, 1.1137178606851297, -0.06008239871824195, 0.6528953994048976, 1.6088731212329286, 1.177424277103837]
#     j_ps = j_ps2
#     robot.setJPVT(j_ps, [0.3]*len(j_ps), [0.35]*len(j_ps))
#     time.sleep(5)

#     j_ps3 = [0.31834134431982974, 1.8560692759594115, -0.5296787975890744, 1.059929808499275, -0.7291905088883794, 1.1297398336766609]
#     j_ps = j_ps3
#     robot.setJPVT(j_ps, [0.3]*len(j_ps), [0.35]*len(j_ps))
#     time.sleep(5)


j_ps = [
    0.014686808575570254,
    0.7303349355306317,
    0.14248111696040233,
    0.9458686198214696,
    # -0.04482337682154558,
    # 0.478942549782559,
]
robot.setJPVT(j_ps, [0.3] * len(j_ps), [0.35] * len(j_ps))
# # robot.setJPVT([0], [0.5], [0.1])
time.sleep(5)

start_t = time.time()
while True:
    t = time.time() - start_t
    j = [
        np.sin(0.6 * t),
        np.pi / 2 + np.sin(0.6 * t - np.pi),
        np.sin(0.6 * t + np.pi),
        j_ps[3] + 0.5*np.sin(0.6 * t - np.pi),
        # j_ps[4] + 0.5*np.sin(0.6 * t - np.pi),
        # j_ps[5] + 0.5*np.sin(0.6 * t - np.pi),
    ]
    robot.setJPVT(j, [1]*len(j), [0.45]*len(j))
    # robot.setJPVT([np.pi/2*np.sin(1*t)], [1], [0.1])
    time.sleep(dt)

# robot.switch_mit()
# print(robot.getP())


# time.sleep(3)
# print(robot.getP())
