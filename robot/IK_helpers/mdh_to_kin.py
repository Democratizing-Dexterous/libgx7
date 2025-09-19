import numpy as np
from IK_helpers.subproblem import rot
from robot_kin.kuka import kuka


def mdh_to_kin(alpha_vec, a_vec, d_vec, theta_vec):
    """
    将基于(修正)DH参数的机械臂描述转换为 POE 形式的 kinematics 字典。
    计算顺序与维度约定：
      - kin['joint_type'] : (N,)  全零（转动关节）
      - kin['H']          : (3,N) 关节轴(unit)，列向量为每个关节轴
      - kin['P']          : (3,N+1) 相邻坐标系原点间的位移列向量
      - kin['RT']         : (3,3) 末端的固定旋转

    参数：符号参考 John. J. Craig 的 《机器人学导论（第4版）》
    ----
    alpha_vec : array_like, shape (N+1,)
        MDH 中的 α_{i-1}
    a_vec     : array_like, shape (N+1,)
        MDH 中的 a_{i-1}
    d_vec     : array_like, shape (N+1,)
        MDH 中的 d_i
    theta_vec : array_like, shape (N+1,)
        MDH 中的 theta_i ，初始位置默认都是 0
    之所以是 N+1 ，是留了一排从最后一轴到工具坐标系的参数，有人可能坐标系的位姿与最后一轴不同
    返回
    ----
    kin : dict
        包含 'joint_type', 'H', 'P', 'RT'
    """
    alpha_vec = np.asarray(alpha_vec, dtype=float).reshape(-1)
    a_vec     = np.asarray(a_vec,     dtype=float).reshape(-1)
    d_vec     = np.asarray(d_vec,     dtype=float).reshape(-1)
    theta_vec = np.asarray(theta_vec, dtype=float).reshape(-1)

    if not (len(alpha_vec) == len(a_vec) == len(d_vec)):
        raise ValueError("alpha_vec, a_vec, d_vec 长度必须一致。")

    N = len(alpha_vec)

    kin = {}
    kin['joint_type'] = np.zeros(N-1, dtype=int)        # (N,)
    kin['H'] = np.full((3, N-1), np.nan, dtype=float)   # (3, N)
    kin['P'] = np.full((3, N), np.nan, dtype=float) # (3, N+1)

    # 初始化
    kin['P'][:, 0] = np.array([0.0, 0.0, 0.0])
    kin['H'][:, 0] = np.array([0.0, 0.0, 1.0])        # z0
    R = np.eye(3)                                     # 基坐标系旋转
    ex = np.array([[1], [0], [0]])
    ez = np.array([[0], [0], [1]])

    for i in range(N-1):

        R = R @ rot(ex, alpha_vec[i])

        kin['P'][:, i] = a_vec[i] * R[:, 0]

        R = R @ rot(ez, theta_vec[i])

        kin['P'][:, i] = kin['P'][:, i] + d_vec[i] * R[:, 2]

        # if i == N: kin.RT = rot(R(:,1), alpha_i)
        # else:      kin.H(:,i+1) = R(:,3)
        if i == N -2:
            kin['H'][:, i] = R[:, 2]
            R = R @ rot(ex, alpha_vec[i+1])
            kin['P'][:, i + 1] = a_vec[i + 1] * R[:, 0]
            R = R @ rot(ez, theta_vec[i + 1])
            kin['P'][:, i+1] = kin['P'][:, i+1] + d_vec[i+1] * R[:, 2]
            kin['T'] = kin['P'][:, i+1]
            kin['R'] = R
        else:
            kin['H'][:, i] = R[:, 2]   # z 轴

    return kin

if __name__ == "__main__":
    # KUKA LBR iiwa 14 R820 MDH parameters
    # alpha_vec = [0, -np.pi/2, np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, np.pi/2, 0]
    # a_vec     = [0, 0, 0, 0, 0, 0, 0, 0]
    # d_vec     = [0.36, 0, 0.42, 0, 0.40, 0, 0,0.126]
    # theta_vec = [0, 0, 0, 0, 0, 0, 0, 0]
    #
    # kin = mdh_to_kin(alpha_vec, a_vec, d_vec, theta_vec)
    # kin2 = kuka()
    #
    # print("Joint axes H:")
    # print(kin["H"])
    # print("\nLink vectors P:")
    # print(kin["P"])
    # print("\nEnd-effector rotation RT:")
    # print(kin["RT"])
    # print("Joint axes H:")
    # print(kin2["H"])
    # print("\nLink vectors P:")
    # print(kin2["P"])
    alpha_vec = [0, np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2, 0]
    a_vec     = [0, 0, 0, -0.077, 0, 0, 0, 0]
    d_vec     = [0.1195, 0, 0.3210, 0, 0.2760, 0, 0,0.990]
    theta_vec = [np.pi, np.pi/2, np.pi, 0, -np.pi, -np.pi, 0, 0]
    kin3 = mdh_to_kin(alpha_vec, a_vec, d_vec, theta_vec)
    print("Joint axes H:")
    print(kin3["H"])
    print("\nLink vectors P:")
    print(kin3["P"])
    print("\nEnd-effector rotation R:")
    print(kin3["R"])
    print("\nEnd-effector translation T:")
    print(kin3["T"])


