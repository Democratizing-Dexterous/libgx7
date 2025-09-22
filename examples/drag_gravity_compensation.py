import numpy as np
import time

import sys

sys.path.append('../')
from robot import GX7
from robot.dynamics import Dynamics
from hardware.can import VCICAN


can = VCICAN()
can.init_can()
FREQ = 100  # Hz
robot =  GX7(can, can_channel=1, freq=FREQ, control_mode='mit', soft_limit=False) # 初始化机器人，设置为MIT模式，不开启软关节限位检测

dt = 1/FREQ
robot.setup()
robot.run()  

dynamics = Dynamics()
urdf_sign = np.array([-1, 1, 1, 1, -1, -1])

qd_zero = np.zeros((1, 6))
qdd_zero = np.zeros((1, 6))

while True:
    q = robot.getJP()
    
    # 计算inverse dynamics，计算补偿重力所需要的力矩
    tau = dynamics.calc(np.array(q), qd_zero, qdd_zero) 
    # 设置关节力矩，以抵消机械臂重力，从而实现重力补偿
    robot.setJTs(tau.ravel().tolist())
    
    time.sleep(dt) # 等待dt秒，注意要限制发送指令频次，否则会导致CAN通道通信拥堵，机械臂关节会报通信丢失错误
    
