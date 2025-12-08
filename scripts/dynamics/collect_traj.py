import numpy as np
import time
import sys
import csv

sys.path.append("../../")  # 根据你的目录结构修改
from robot import GX7
from hardware.can import VCICAN

# 初始化 CAN
can = VCICAN()
can.init_can()

FREQ = 100  # Hz
robot = GX7(can, can_channel=1, freq=FREQ, control_mode="pvt")
robot.setup()
robot.run()  # Start the robot thread

# 采集时长（秒）
DURATION = 2  # 采集10秒，可自行修改
PERIOD = 1.0 / FREQ

# CSV 文件名
csv_filename = f"data/traj_p.csv"

# 写 CSV 文件头
with open(csv_filename, mode="w", newline="") as file:
    writer = csv.writer(file)
    print("开始采集数据...")
    start_time = time.perf_counter()
    while True:
        positions = robot.getJP().tolist()
        # 保存到 CSV
        writer.writerow(positions)

        time.sleep(PERIOD)

# print(f"采集完成，数据保存到 {csv_filename}")
