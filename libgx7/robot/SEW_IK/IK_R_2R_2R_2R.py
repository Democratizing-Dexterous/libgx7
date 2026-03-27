import numpy as np
from IK_helpers.subproblem import subproblem, rot
from IK_helpers.search_1D import search_1D  # 假设你有对应 Python 实现

def IK_R_2R_2R_2R(R_07, p_0T, SEW_class, psi, kin, show_plot=False):
    Q_all = []
    is_LS_vec = []

    # 定义匿名函数 e_fun
    def e_fun(WA):
        return q67_alignment_given_wrist_angle(WA, kin, R_07, p_0T, psi, SEW_class)

    WA_vec, soln_num_vec = search_1D(e_fun, -np.pi, 0, 200, show_plot)

    for i, WA in enumerate(WA_vec):
        _, q_solns_partial_i = q67_alignment_given_wrist_angle(WA, kin, R_07, p_0T, psi, SEW_class)
        q_partial_i = q_solns_partial_i[:, soln_num_vec[i]]
        q, is_LS = q_given_q12345(q_partial_i, kin, R_07)
        Q_all.append(q)
        is_LS_vec.append(is_LS)

    return np.array(Q_all).T, np.array(is_LS_vec)


def q67_alignment_given_wrist_angle(wrist_angle, kin, R_07, T, psi, SEW_class):
    q_solns_partial = np.full((5, 8), np.nan)
    alignment = np.full(8, np.nan)
    i_soln = 0

    p_W_EE_0 = kin["P"][:, 7].reshape(3,1)
    h_1 = kin["H"][:, 0].reshape(3,1)
    p_E2_0 = -kin["P"][:, 3].reshape(3,1)
    d_2E = np.linalg.norm(p_E2_0)
    p_WE_0 = -kin["P"][:, 5].reshape(3,1)
    d_WE = np.linalg.norm(p_WE_0)

    # Wrist and shoulder positions
    W = T.reshape(3,1) - R_07 @ p_W_EE_0.reshape(3,1)
    S = kin["P"][:, 0].reshape(3,1)
    p_17 = W - S

    # Elbow position
    w_hat = vec_normalize(W - S)
    _, n_SEW = SEW_class.inv_kin(S, W, psi)
    p_WE = rot(n_SEW, wrist_angle) @ (-w_hat) * d_WE

    # Find q1 using subproblem 3
    q_1_solns, q_1_is_LS = subproblem.sp_3(kin["P"][:, 1], p_17 + p_WE, h_1, d_2E)
    if q_1_is_LS:
        return alignment, q_solns_partial

    for q_1 in q_1_solns.flatten():
        R_10 = rot(kin["H"][:, 0], -q_1)
        q_3_solns, q_2_solns, q_23_is_LS = subproblem.sp_2(
            kin["P"][:, 3],
            R_10 @ p_17 + R_10 @ p_WE - kin["P"][:, 1],
            kin["H"][:, 2],
            -kin["H"][:, 1]
        )
        if q_23_is_LS:
            i_soln += 4
            continue

        for q_2, q_3 in zip(q_2_solns.flatten(), q_3_solns.flatten()):
            R_21 = rot(kin["H"][:, 1], -q_2)
            R_32 = rot(kin["H"][:, 2], -q_3)

            q_5_solns, q_4_solns, q_45_is_LS = subproblem.sp_2(
                kin["P"][:, 5],
                R_32 @ R_21 @ (R_10 @ p_17 - kin["P"][:, 1]) - kin["P"][:, 3],
                kin["H"][:, 4],
                -kin["H"][:, 3]
            )
            if q_45_is_LS:
                i_soln += 2
                continue

            for q_4, q_5 in zip(q_4_solns.flatten(), q_5_solns.flatten()):
                R_34 = rot(kin["H"][:, 3], q_4)
                R_45 = rot(kin["H"][:, 4], q_5)
                R_05 = R_10.T @ R_21.T @ R_32.T @ R_34 @ R_45
                e_i = kin["H"][:, 5].T @ R_05.T @ R_07 @ kin["H"][:, 6] - kin["H"][:, 5].T @ kin["H"][:, 6]
                alignment[i_soln] = e_i
                q_solns_partial[:, i_soln] = [q_1, q_2, q_3, q_4, q_5]
                i_soln += 1

    return alignment, q_solns_partial


def q_given_q12345(q12345, kin, R_07):
    R_01 = rot(kin["H"][:, 0], q12345[0])
    R_12 = rot(kin["H"][:, 1], q12345[1])
    R_23 = rot(kin["H"][:, 2], q12345[2])
    R_34 = rot(kin["H"][:, 3], q12345[3])
    R_45 = rot(kin["H"][:, 4], q12345[4])
    R_05 = R_01 @ R_12 @ R_23 @ R_34 @ R_45

    q6, q6_is_LS = subproblem.sp_1(kin["H"][:, 6], R_05.T @ R_07 @ kin["H"][:, 6], kin["H"][:, 5])
    q7, q7_is_LS = subproblem.sp_1(kin["H"][:, 5], R_07.T @ R_05 @ kin["H"][:, 5], -kin["H"][:, 6])

    q = np.array([q12345, q6, q7])
    is_LS = q6_is_LS or q7_is_LS
    return q, is_LS


def vec_normalize(vec):
    return vec / np.linalg.norm(vec)
