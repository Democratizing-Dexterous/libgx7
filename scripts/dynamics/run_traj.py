import numpy as np
import time
import sys
import csv
from datetime import datetime

sys.path.append("../../")
from robot import GX7
from hardware.can import VCICAN

# ===============================
# 初始化 CAN 和机器人
# ===============================
can = VCICAN()
can.init_can()

FREQ = 100  # Hz
PERIOD = 1.0 / FREQ

robot = GX7(can, can_channel=1, freq=FREQ, control_mode="pvt")
robot.setup()
robot.switch_pvt()
robot.run()


points = np.loadtxt("data/traj_p.csv", delimiter=",")

points = points[6*FREQ : -6*FREQ , :]

# ===============================
# 先运动到第一个点（安全过渡）
# ===============================
first_point = points[0]
robot.setJPVTs(first_point, [0.3] * len(first_point), [0.6] * len(first_point))
time.sleep(5)  # 给一些时间到达第一个点

# ===============================
# 新 CSV 文件用于记录实际执行数据
# ===============================
new_csv_filename = f"data/executed_traj_pvt.csv"

with open(new_csv_filename, mode="w", newline="") as file:
    writer = csv.writer(file)
    # ===============================
    # 执行轨迹 + 采集反馈
    # ===============================
    print("开始执行轨迹并采集数据...")
    for point in points:
        # 发送目标点（这里只设置位置，速度和力矩限制可以调整）
        robot.setJPVTs(point, [0.4] * len(point), [0.6] * len(point))

        # 提取数据
        timestamps = robot.getJTimestamp().tolist()
        positions = robot.getJP().tolist()
        velocities = robot.getJV().tolist()
        currents = robot.getJT().tolist()  # torque 通常和电流相关

        # 保存到 CSV
        writer.writerow(timestamps + positions + velocities + currents)
        time.sleep(PERIOD)

print(f"轨迹执行完成，已保存到 {new_csv_filename}")


robot.setJPVTs([0] * 7, [0.2] * 7, [0.6] * 7)

time.sleep(5)
# # 停止机器人
# robot.stop()
# robot.disable()
