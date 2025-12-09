import numpy as np
import time
import sys
import csv
import cv2
from datetime import datetime
import numpy as np

sys.path.append("../../")
from robot import GX7
from robot.dynamics.gx7.calc_dynamics import CalcDynamics
from hardware.can import VCICAN

# ===============================
# 初始化 CAN 和机器人
# ===============================
can = VCICAN()
can.init_can()
FREQ = 100  # Hz
robot = GX7(can, can_channel=1, freq=FREQ, control_mode="pvt", soft_limit=False)
robot.setup()
robot.switch_mit()  # 切换到MIT模式
robot.run()


beta = np.load("ls_id_beta.npy")
dynamics_regressor = CalcDynamics()

# press s to save current position into data/points.csv

csv_filename = f"data/points.csv"


with open(csv_filename, mode="w", newline="") as file:
    writer = csv.writer(file)

    while True:
        positions = robot.getJP()
        velocities = robot.getJV()

        regressor = dynamics_regressor.calc(positions, velocities * 0, np.zeros(7))

        tau = regressor @ beta

        robot.setJTs(tau.ravel().tolist())

        # check keybard
        key = cv2.waitKey(1)
        print(f'key: {key}')
        if cv2.waitKey(1) != -1:
            writer.writerow(positions)
            print(f"save {positions}")
