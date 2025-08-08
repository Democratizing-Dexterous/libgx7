import threading
import time

import numpy as np

from utils.libgx7.robot.robot import Robot
from utils.libgx7.utils.utils import MotorStatus

from call_regressor import compute_regressor

FREQ = 100  # Hz
robot = Robot(FREQ, 'linux')

urdf_sign = np.array([-1, 1, 1, 1, -1, -1])

base_idxs = np.array([5, 11, 12, 13, 14, 15, 16, 18, 19, 20, 
        21, 25, 26, 27, 28, 29, 30, 32, 33, 34, 
        35, 38, 39, 40, 41, 42, 43, 44, 46, 47, 
        48, 49, 52, 53, 54, 55, 56, 57, 58, 60, 
        61, 62, 63, 66, 67, 68, 69, 70, 71, 72, 
        74, 75, 76, 77, 80, 81, 82, 83])

init_motors_status_params = [[0] * robot.num_dof] * 8
global_motors_status = MotorStatus(*init_motors_status_params)
robot.load_global_motors_status(global_motors_status)

threading_robot_run = threading.Thread(target=robot.run)
threading_robot_run.daemon = True
threading_robot_run.start()

dt = 1 / FREQ

beta_ols = np.load('data/beta_ols.npy')
print(beta_ols.shape)


robot.switch_mit()

time.sleep(0.5)

qd_zero = np.zeros((1, 6))
qdd_zero = np.zeros((1, 6))



# for i in range(int(1e5)):
while True:
    
    pos_i = np.array(robot.global_motors_status.positions)[None]*urdf_sign[None]

    
    regressor = compute_regressor(pos_i, qd_zero, qdd_zero).reshape(6, -1)[:, base_idxs]
    pred_ols_i = regressor @ beta_ols

    tor_apply = pred_ols_i*urdf_sign[:, None]
    
    # tor_apply = np.clip(tor_apply, -5, 5)


    # if i %100 == 0:
    #     print(robot.getJT())
    robot.setJT(tor_apply.ravel().tolist())

    time.sleep(dt)
