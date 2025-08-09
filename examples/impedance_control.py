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
robot = GX7(FREQ, 'mit', soft_limit=False)
robot.setup()
robot.run()

# 初始化动力学模型
dynamics = Dynamics()

# 阻抗控制参数
# lambda: 刚度系数，决定了系统对位置偏差的响应强度
# 可以根据需要调整不同方向的刚度
lambda_x = 10  # x方向刚度
lambda_y = 10  # y方向刚度
lambda_z = 10  # z方向刚度

# 初始化零向量，用于动力学计算
qd_zero = np.zeros((1, 6))
qdd_zero = np.zeros((1, 6))

# 获取初始末端位置作为目标位置
target_position = np.array([0, 0, 0.5])

print("初始末端位置:", target_position)
print("阻抗控制已启动，按Ctrl+C退出...")

try:
    while True:
        # 获取当前关节角度
        q = robot.getJP()
        q = q+[0]
        
        # 计算当前末端位置和姿态
        current_pos, current_rot = robot.fk(q)
        
        # 计算位置误差
        pos_error = target_position - current_pos
        
        # 计算雅可比矩阵
        J = robot.jac(q)
        
        # 阻抗控制力计算: F = λ(Pd - T(q))
        # 只考虑位置控制，不考虑姿态控制
        impedance_force = np.zeros(6)
        impedance_force[:3] = np.array([lambda_x, lambda_y, lambda_z]) * pos_error
        
        # 计算关节力矩: τ = J^T(q)F + M(q)q̈ + C(q,q̇) + G(q)
        # 1. 计算重力和动力学补偿项
        dynamics_torque = dynamics.calc(np.array(q), qd_zero, qdd_zero)
        
        # 2. 计算阻抗控制力矩: J^T(q)F
        impedance_torque = J.T @ impedance_force
        
        # 3. 合并力矩
        total_torque = dynamics_torque.ravel() + impedance_torque[:-1]
        
        # 设置关节力矩
        robot.setJTs(total_torque.tolist())
        
        # 打印当前状态信息（可选，用于调试）
        if int(time.time() * FREQ) % FREQ == 0:  # 每秒打印一次
            print(f"当前位置: {current_pos}, 目标位置: {target_position}, 误差: {pos_error}")
            print(f"阻抗力: {impedance_force[:3]}, 总力矩: {total_torque}")
        
        # 等待下一个控制周期
        time.sleep(dt)
        
except KeyboardInterrupt:
    print("程序已停止")
    # 在退出前，设置零力矩以确保安全
    robot.setJTs([0, 0, 0, 0, 0, 0])
    time.sleep(0.1)
    # 停止机器人控制线程
    robot.stop()
