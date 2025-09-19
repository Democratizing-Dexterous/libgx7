import numpy as np
from IK_helpers.subproblem import subproblem, rot, wrapToPi
from IK_helpers.search_1D import search_1D  # 假设你已有对应 Python 实现


def IK_R_2R_R_3R_SJ2(R_07, p_0T, SEW_class, psi, kin, show_plot=False):
    Q_all = []
    is_LS_vec = []

    W = p_0T.reshape(3,1) - R_07 @ kin["P"][:, 7].reshape(3,1)
    p_1W = W - kin["P"][:, 0].reshape(3,1)

    def e_fun(q1):
        return wrapToPi(psi_given_q1(q1, kin, p_1W, SEW_class)[0] - psi)

    q1_vec, soln_num_vec = search_1D(e_fun, -np.pi, np.pi, 2000, show_plot)

    for i_q1, q1 in enumerate(q1_vec):
        psi_vec, partial_Q = psi_given_q1(q1, kin, p_1W, SEW_class)
        partial_q = partial_Q[:, soln_num_vec[i_q1]]
        R_01 = rot(kin["H"][:, 0], partial_q[0])
        R_12 = rot(kin["H"][:, 1], partial_q[1])
        R_23 = rot(kin["H"][:, 2], partial_q[2])
        R_34 = rot(kin["H"][:, 3], partial_q[3])
        R_04 = R_01 @ R_12 @ R_23 @ R_34

        t5, t6, t_56_is_LS = subproblem.sp_2(
            R_04.T @ R_07 @ kin["H"][:, 6],
            kin["H"][:, 6],
            -kin["H"][:, 4],
            kin["H"][:, 5]
        )

        for q5, q6 in zip(t5.flatten(), t6.flatten()):

            R_45 = rot(kin["H"][:, 4], q5)
            R_56 = rot(kin["H"][:, 5], q6)
            p = kin["H"][:, 5]  # non-collinear with h_7
            R_06 = R_04 @ R_45 @ R_56
            q7, q7_is_LS = subproblem.sp_1(p, R_06.T @ R_07 @ p, kin["H"][:, 6])

            q_i = np.array([*partial_q, q5, q6, q7])
            Q_all.append(q_i)
            is_LS_vec.append(t_56_is_LS or q7_is_LS)

    return np.array(Q_all).T, np.array(is_LS_vec)


def psi_given_q1(q1, kin, p_1W, SEW_class):
    psi_vec = np.full(4, np.nan)
    partial_Q = np.full((4, 4), np.nan)
    i_soln = 0

    p_1S = rot(kin["H"][:, 0], q1) @ kin["P"][:, 1].reshape(3,1)
    p_SW = p_1W.reshape(3,1) - p_1S.reshape(3,1)

    t4, t4_is_LS = subproblem.sp_3(kin["P"][:, 4], -kin["P"][:, 3], kin["H"][:, 3], np.linalg.norm(p_SW))
    if t4_is_LS:
        return psi_vec, partial_Q

    for q4 in t4.flatten():
        t2, t3, t23_is_LS = subproblem.sp_2(
            rot(kin["H"][:, 0], q1).T @ p_SW,
            kin["P"][:, 3] + rot(kin["H"][:, 3], q4) @ kin["P"][:, 4],
            -kin["H"][:, 1],
            kin["H"][:, 2]
        )
        if t23_is_LS:
            i_soln += 2
            continue

        for q2, q3 in zip(t2.flatten(), t3.flatten()):
            p_1E = p_1S + rot(kin["H"][:, 0], q1) @ rot(kin["H"][:, 1], q2) @ rot(kin["H"][:, 2], q3) @ kin["P"][:, 3]
            psi_i = SEW_class.fwd_kin(p_1S, p_1E, p_1W)
            psi_vec[i_soln] = psi_i
            partial_Q[:, i_soln] = [q1, q2, q3, q4]
            i_soln += 1

    return psi_vec, partial_Q