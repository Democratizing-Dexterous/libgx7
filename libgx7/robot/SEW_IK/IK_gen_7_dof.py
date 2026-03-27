import numpy as np
from IK_helpers.subproblem import subproblem, rot
from IK_helpers.search_2D import search_2D

def IK_gen_7_dof(R_07, p_0T, SEW_class, psi, kin):
    Q_all = []
    is_LS_vec = []

    # Find wrist position
    W = p_0T.reshape(3,1) - R_07 @ kin["P"][:, 7]
    S = kin["P"][:, 0]

    p_17 = W - S
    e_SW = p_17 / np.linalg.norm(p_17)
    e_CE, n_SEW = SEW_class.inv_kin(S, W, psi)

    # 2D search over q1, q2
    q1_vec, q2_vec, soln_num_vec = search_2D(
        lambda q1, q2: q4_solvability_given_q12(q1, q2, kin, n_SEW, e_CE, R_07, p_17),
        -np.pi, np.pi, -np.pi, np.pi, 5000, False
    )

    for i in range(len(q1_vec)):
        _, Q123_567 = q4_solvability_given_q12(q1_vec[i], q2_vec[i], kin, n_SEW, e_CE, R_07, p_17)
        q, is_LS = q_given_q123_567(Q123_567[:, soln_num_vec[i]], kin, R_07)
        Q_all.append(q)
        is_LS_vec.append(is_LS)

    return np.array(Q_all).T, np.array(is_LS_vec)


def q4_solvability_given_q12(q1, q2, kin, n_SEW, e_CE, R_07, p_17):
    e = np.full(8, np.nan)
    Q123_567 = np.full((6, 8), np.nan)
    i_soln = 0

    R_01 = rot(kin["H"][:, 0], q1)
    R_12 = rot(kin["H"][:, 1], q2)
    R_02 = R_01 @ R_12

    # Find q3 with Subproblem 4
    t3, t3_is_LS = subproblem.sp_4(R_02.T @ n_SEW, kin["P"][:, 3], kin["H"][:, 2],
                                    -n_SEW.T @ (R_01 @ kin["P"][:, 1] + R_02 @ kin["P"][:, 2]))
    if t3_is_LS:
        return e, Q123_567

    for idx, q3 in enumerate(t3):
        if idx == 1:
            i_soln = 4  # MATLAB indexing adjustment

        R_23 = rot(kin["H"][:, 2], q3)
        R_03 = R_02 @ R_23
        p_14 = R_01 @ kin["P"][:, 1] + R_02 @ kin["P"][:, 2] + R_03 @ kin["P"][:, 3]

        # Check correct half-plane
        if np.dot(e_CE, p_14) < 0:
            continue

        # Find (q5, q6, q7) with Subproblem 5
        t7, t6, t5 = subproblem.sp_5(
            -kin["P"][:, 6],
            R_07.T @ (p_17 - p_14),
            kin["P"][:, 5],
            kin["P"][:, 4],
            kin["H"][:, 6],
            -kin["H"][:, 5],
            -kin["H"][:, 4]
        )

        for q5, q6, q7 in zip(t5.flatten(), t6.flatten(), t7.flatten()):


            R_45 = rot(kin["H"][:, 4], q5)
            R_56 = rot(kin["H"][:, 5], q6)
            R_67 = rot(kin["H"][:, 6], q7)
            R_47 = R_45 @ R_56 @ R_67

            e[i_soln] = np.linalg.norm(R_03.T @ R_07 @ R_47.T @ kin["H"][:, 3] - kin["H"][:, 3])
            Q123_567[:, i_soln] = np.array([q1, q2, q3, q5, q6, q7])
            i_soln += 1

    return e, Q123_567


def q_given_q123_567(q123_567, kin, R_07):
    R_01 = rot(kin["H"][:, 0], q123_567[0])
    R_12 = rot(kin["H"][:, 1], q123_567[1])
    R_23 = rot(kin["H"][:, 2], q123_567[2])

    R_45 = rot(kin["H"][:, 4], q123_567[3])
    R_56 = rot(kin["H"][:, 5], q123_567[4])
    R_67 = rot(kin["H"][:, 6], q123_567[5])

    R_03 = R_01 @ R_12 @ R_23
    R_47 = R_45 @ R_56 @ R_67

    p = kin["H"][:, 2]  # Non-collinear with h4
    q4, is_LS = subproblem.sp_1(p, R_03.T @ R_07 @ R_47.T @ p, kin["H"][:, 3])
    q = np.concatenate([q123_567[:3], [q4], q123_567[3:]])
    return q, is_LS
