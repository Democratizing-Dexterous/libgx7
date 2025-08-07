import numpy as np
from robot import GX7
import time
from matplotlib import pyplot as plt
import argparse

plt.rcParams['font.sans-serif'] = ['SimHei']  # 使用黑体
plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

# 命令行参数解析
parser = argparse.ArgumentParser(description='电机阶跃响应测试与可视化')
parser.add_argument('--axis', type=int, default=2, help='要测试的轴号 (1-7)')
parser.add_argument('--target', type=float, default=1.0, help='目标位置 (rad)')
parser.add_argument('--speed_limit', type=float, default=2.0, help='速度限制 (rad/s)')
parser.add_argument('--duration', type=int, default=300, help='测试持续时间 (帧数)')
parser.add_argument('--mode', type=str, default='pv', choices=['pv', 'pvt', 'mit'], help='控制模式')
parser.add_argument('--pk', type=float, default=56, help='位置环P')
parser.add_argument('--acc', type=float, default=5, help='加速度')
args = parser.parse_args()

# 设置参数
axis = args.axis
target = args.target
speed_limit = args.speed_limit
acc = args.acc
pk = args.pk
N = args.duration
mode = args.mode

FREQ = 100  # Hz
robot = GX7(FREQ, 'win', mode)

robot.can.write_acc(axis, args.acc)
robot.can.write_dec(axis, -args.acc)
robot.can.write_pos_kp(axis, pk)

# 读取电机控制参数
pos_kp = robot.can.read_pos_kp(axis)
pos_ki = robot.can.read_pos_ki(axis)
vel_kp = robot.can.read_vel_kp(axis)
vel_ki = robot.can.read_vel_ki(axis)
acc = robot.can.read_acc(axis)
dec = robot.can.read_dec(axis)

print(f'位置控制参数 - KP: {pos_kp}, KI: {pos_ki}')
print(f'速度控制参数 - KP: {vel_kp}, KI: {vel_ki}')
print(f'加速度: {acc}, 减速度: {dec}')

robot.run()  # 启动机器人线程

time.sleep(1)

dt = 1/FREQ
N = 200

# 创建数组存储多个参数的轨迹
pos_traj = np.zeros(N)
vel_traj = np.zeros(N)
torque_traj = np.zeros(N)
time_traj = np.zeros(N)
rotor_temp_traj = np.zeros(N)
mos_temp_traj = np.zeros(N)

# 设置目标位置和速度限制
if mode == 'pv':
    robot.setJPV(axis, target, speed_limit)
elif mode == 'pvt':
    robot.setJPVT(axis, target, speed_limit, 0.6)  # 默认力矩限制为0.6
elif mode == 'mit':
    robot.setJP(axis, target)

# 采集数据
print(f"开始测试轴 {axis} 的阶跃响应，目标位置: {target} rad，持续时间: {N/FREQ:.1f} 秒")
for i in range(N):
    # 获取当前状态
    pos = robot.getJP()[axis-1]
    vel = robot.getJV()[axis-1]
    torque = robot.getJT()[axis-1]
    rotor_temp = robot.getRotorTemp()[axis-1]
    mos_temp = robot.getMossTemp()[axis-1]
    time_stamp = time.perf_counter()
    
    # 存储数据
    pos_traj[i] = pos
    vel_traj[i] = vel
    torque_traj[i] = torque
    rotor_temp_traj[i] = rotor_temp
    mos_temp_traj[i] = mos_temp
    time_traj[i] = time_stamp
    
    # 显示进度
    if i % 50 == 0:
        print(f"进度: {i}/{N} ({i/N*100:.1f}%)")
    
    time.sleep(dt)

print("测试完成，正在生成图表...")

# 计算相对时间（从0开始）
rel_time = time_traj - time_traj[0]

# 创建多子图显示
fig, axs = plt.subplots(3, 1, figsize=(6, 8), sharex=True)

# 位置轨迹图
axs[0].plot(rel_time, pos_traj, 'b-', linewidth=2)
axs[0].hlines(target, 0, rel_time[-1], 'r', 'dashed', label='目标位置')
axs[0].set_ylabel('位置 (rad)')
axs[0].set_title(f'轴 {axis} - 位置响应 (KP: {pos_kp:.2f}, KI: {pos_ki:.2f})')
axs[0].grid(True)
axs[0].legend()

# 速度轨迹图
axs[1].plot(rel_time, vel_traj, 'g-', linewidth=2)
axs[1].hlines(speed_limit, 0, rel_time[-1], 'r', 'dashed', label='速度限制')
axs[1].set_ylabel('速度 (rad/s)')
axs[1].set_title(f'轴 {axis} - 速度响应 (KP: {vel_kp:.6f}, KI: {vel_ki:.6f})')
axs[1].grid(True)
axs[1].legend()

# 力矩轨迹图
axs[2].plot(rel_time, torque_traj, 'm-', linewidth=2)
axs[2].set_ylabel('力矩 (Nm)')
axs[2].set_title(f'轴 {axis} - 力矩响应')
axs[2].grid(True)


# 添加总标题
plt.suptitle(f'轴 {axis} 阶跃响应 - 模式: {mode.upper()}\n 加速度: {acc:.2f}, 减速度: {dec:.2f}\n',
             fontsize=12, color='red')

# 调整子图间距
plt.tight_layout()
plt.subplots_adjust(top=0.9)

# 保存图形到文件
# filename = f'step_response_axis{axis}_{mode}_target{target}.png'
# plt.savefig(filename)
# print(f"图表已保存到文件: {filename}")



# 打印关键指标
settling_time = None
overshoot = None
rise_time = None

# 计算上升时间 (10% 到 90% 的时间)
target_diff = target - pos_traj[0]
if abs(target_diff) > 0.001:  # 确保目标变化足够大
    rise_start_idx = next((i for i, p in enumerate(pos_traj) if abs(p - pos_traj[0]) >= 0.1 * abs(target_diff)), None)
    rise_end_idx = next((i for i, p in enumerate(pos_traj) if abs(p - pos_traj[0]) >= 0.9 * abs(target_diff)), None)
    
    if rise_start_idx is not None and rise_end_idx is not None:
        rise_time = rel_time[rise_end_idx] - rel_time[rise_start_idx]
        print(f"上升时间 (10% 到 90%): {rise_time:.3f} 秒")

# 计算超调量
if abs(target_diff) > 0.001:
    # 找到第一个接近目标值的点之后的最大偏差
    first_approach_idx = next((i for i, p in enumerate(pos_traj) if abs(p - target) <= 0.05 * abs(target_diff)), None)
    
    if first_approach_idx is not None:
        max_overshoot = 0
        for i in range(first_approach_idx, len(pos_traj)):
            overshoot_val = abs(pos_traj[i] - target)
            if overshoot_val > max_overshoot:
                max_overshoot = overshoot_val
        
        if max_overshoot > 0:
            overshoot_percent = (max_overshoot / abs(target_diff)) * 100
            print(f"超调量: {overshoot_percent:.2f}%")

# 计算稳定时间 (误差在 ±5% 以内)
if abs(target_diff) > 0.001:
    error_threshold = 0.05 * abs(target_diff)
    for i in range(len(pos_traj) - 1, 0, -1):
        if abs(pos_traj[i] - target) > error_threshold:
            settling_time = rel_time[i+1] if i+1 < len(rel_time) else rel_time[-1]
            print(f"稳定时间 (±5%): {settling_time:.3f} 秒")
            break

# 显示图形
plt.show()