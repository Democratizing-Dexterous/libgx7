import numpy as np
import sys
from tqdm import tqdm
import matplotlib.pyplot as plt

sys.path.append("../../")
from robot.dynamics.gx7.calc_dynamics import CalcDynamics

# 初始化动力学计算器
calc = CalcDynamics()

# 读取执行轨迹数据
points = np.loadtxt("data/executed_traj_pvt.csv", delimiter=",")

# 你存的格式不确定 列数可能有偏差，这里要改成你的实际格式
timestamps = points[:, :7]
positions = points[:, 7:14]  # 接下来 7 列是关节位置
velocities = points[:, 14:21]  # 7 列是关节速度
taus = points[:, 21:28]  # 7 列是关节力矩


regressors = np.zeros((len(positions), 7, 69))

for i in range(len(positions)):
    regressor = calc.calc(
        positions[i], velocities[i], np.zeros_like(velocities[i])
    )  # shape (7, 69)
    regressors[i] = regressor

# tau = regressor@beta
beta = np.linalg.pinv(regressors.reshape(len(positions) * 7, -1)) @ taus.reshape(
    len(positions) * 7, -1
)

np.save("ls_id_beta.npy", beta)

pred_taus = regressors @ beta

# plot, 对比真实值和预测值, 真实值为实线(蓝色), 预测值为虚线(红色)
# 使用subplot，设置颜色
fig, axs = plt.subplots(7, 1, figsize=(12, 12))

for p in range(7):
    axs[p].plot(taus[:, p], label=f"tau_{p+1}", color="blue")
    axs[p].plot(
        pred_taus[:, p], "--", label=f"pred_tau_{p+1}", color="red"
    )  # dashed line
    axs[p].set_xlabel("Time step")
    axs[p].set_ylabel("Joint Torque")
    axs[p].set_title(f"LS Tau Prediction for tau_{p+1}")
    axs[p].legend(loc="upper right", fontsize="small", ncol=2)
plt.tight_layout()

plt.savefig("data/ls_id_results.png", dpi=300)
plt.show()
