import numpy as np
import time

import sys

sys.path.append('../')
from robot import GX7
from robot.dynamics import Dynamics


# 初始化参数
FREQ = 100  # Hz
dt = 1/FREQ

# 初始化机器人，设置为MIT模式，不开启软关节限位检测
robot = GX7(FREQ, 'pvt', soft_limit=False)
robot.setup()
robot.run()


# 初始化动力学模型
dynamics = Dynamics()

# 阻抗控制参数
# lambda: 刚度系数，决定了系统对位置偏差的响应强度
# 可以根据需要调整不同方向的刚度
lambda_x = 100  # x方向刚度
lambda_y = 100  # y方向刚度
lambda_z = 100  # z方向刚度
lambda_roll = 100
lambda_pitch = 100
lambda_yaw = 100

# 初始化零向量，用于动力学计算
qd_zero = np.zeros((1, 6))
qdd_zero = np.zeros((1, 6))

# 获取初始末端位置作为目标位置
target_position = np.array([-0.02034015953540802, 0.0690930187702179, 0.3825889527797699])
target_ori = np.array([-1.6441860973730307, 1.50605140812909, -0.11808874284118874])



qs = [-0.08144502937361686, 0.3122377355611512, 0.09212634470130432, 0.6666285191119243, -0.024605172808422893, -0.3572518501564055]

robot.setJPVTs(qs, [1]*len(qs), [0.3]*len(qs))

time.sleep(5)

robot.switch_mit()

# print("初始末端位置:", target_position)
# print("阻抗控制已启动，按Ctrl+C退出...")


while True:
    # 获取当前关节角度
    q = robot.getJP()
    
    # 计算当前末端位置和姿态
    current_pos, current_rpy = robot.fk(q)
    current_z = current_pos[-1]
    
    # 计算雅可比矩阵
    J = robot.jac(q)
    
    
    # 阻抗控制力计算: F = λ(Pd - T(q))
    # 只考虑位置控制，不考虑姿态控制
    impedance_force = np.zeros(6)
    # impedance_force[] = lambda_y * (target_position[1] - current_pos[1])
    # print(impedance_force[1])
    impedance_force[:3] = np.array([lambda_x, lambda_y, lambda_z]) * (target_position - current_pos)
    
    impedance_force[3:] = np.array([lambda_roll, lambda_pitch, lambda_yaw]) * (target_ori - current_rpy)
    
    # 计算关节力矩: τ = J^T(q)F + M(q)q̈ + C(q,q̇) + G(q)
    # 1. 计算重力和动力学补偿项
    dynamics_torque = dynamics.calc(np.array(q), qd_zero, qdd_zero)
    
    # 2. 计算阻抗控制力矩: J^T(q)F
    impedance_torque = J.T @ impedance_force
    
    # 3. 合并力矩
    total_torque = dynamics_torque.ravel()  + impedance_torque
    
    # 设置关节力矩
    robot.setJTs(total_torque.tolist())
    
    
    
    # 等待下一个控制周期
    time.sleep(dt)
        
