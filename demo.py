import numpy as np
from robot import GX7
import time

FREQ = 100  # Hz
robot = GX7(FREQ, 'win', 'pv')


axis = 1
pk = 80
acc = 0.2

robot.can.write_acc(axis, acc)
robot.can.write_dec(axis, -acc)
robot.can.write_pos_kp(axis, pk)

pos_kp = robot.can.read_pos_kp(axis)
pos_ki = robot.can.read_pos_ki(axis)
vel_kp = robot.can.read_vel_kp(axis)
vel_ki = robot.can.read_vel_ki(axis)
acc = robot.can.read_acc(axis)
dec = robot.can.read_dec(axis)

axis = 2
pk = 80
acc = 0.1

robot.can.write_acc(axis, acc)
robot.can.write_dec(axis, -acc)
robot.can.write_pos_kp(axis, pk)

pos_kp = robot.can.read_pos_kp(axis)
pos_ki = robot.can.read_pos_ki(axis)
vel_kp = robot.can.read_vel_kp(axis)
vel_ki = robot.can.read_vel_ki(axis)
acc = robot.can.read_acc(axis)
dec = robot.can.read_dec(axis)

axis = 4
pk = 80
acc = 0.1

robot.can.write_acc(axis, acc)
robot.can.write_dec(axis, -acc)
robot.can.write_pos_kp(axis, pk)

pos_kp = robot.can.read_pos_kp(axis)
pos_ki = robot.can.read_pos_ki(axis)
vel_kp = robot.can.read_vel_kp(axis)
vel_ki = robot.can.read_vel_ki(axis)
acc = robot.can.read_acc(axis)
dec = robot.can.read_dec(axis)

print(f'位置控制参数 - KP: {pos_kp}, KI: {pos_ki}')
print(f'速度控制参数 - KP: {vel_kp}, KI: {vel_ki}')
print(f'加速度: {acc}, 减速度: {dec}')

robot.run()

while True:
    robot.setJPV(1, 0, 3)
    robot.setJPV(2, 0.3, 3)
    robot.setJPV(3, 0, 3)
    robot.setJPV(4, 0.3, 3)
    robot.setJPV(5, 0, 3)
    robot.setJPV(6, 0, 3)
    time.sleep(3)
    
    robot.setJPVs([-0.0993743801022351, 2.2795071335927375, 0.09746700236514805, 1.2998779278248271, -0.08449683375295614, 0.5930037384603644], [1]*6)
    # robot.setJPV(1, 0, 3)
    # robot.setJPV(2, 0, 3)
    # robot.setJPV(4, 0, 3)
    time.sleep(3)