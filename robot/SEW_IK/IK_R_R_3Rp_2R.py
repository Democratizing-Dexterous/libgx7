import numpy as np
from IK_helpers.subproblem import subproblem, rot, wrapToPi

def IK_R_R_3Rp_2R(R_07, p_0T, SEW_class, psi, kin):
    Q_all = []
    is_LS_vec = []

    # Find wrist position
    W = p_0T.reshape(3,1) - R_07 @ kin["P"][:, 7].reshape(3,1)

    # Find shoulder position
    S = kin["P"][:, 0].reshape(3,1)

    p_17 = W - S
    e_SW = p_17 / np.linalg.norm(p_17)

    _, n_SEW = SEW_class.inv_kin(S, W, psi)

    # Find (theta_h, q2) using Subproblem 6
    H = np.column_stack([e_SW, -kin["H"][:, 2], kin["H"][:, 0], -kin["H"][:, 0]])
    K = np.column_stack([-n_SEW, -kin["H"][:, 1], n_SEW, kin["H"][:, 1]])
    P = np.column_stack([p_17, kin["P"][:, 1], e_SW, kin["H"][:, 2]])
    d1 = np.dot(kin["H"][:, 2], np.sum(kin["P"][:, 2:6], axis=1))
    t_h, t2 = subproblem.sp_6(H, K, P, d1, 0)

    for t_h in t_h.flatten():
        if t_h < 0:
            continue
        theta_h = t_h
        q2 = t2
        R_h = rot(n_SEW, theta_h)
        R_12 = rot(kin["H"][:, 1], q2)

        # Find q1 using Subproblem 1
        q1, q1_is_LS = subproblem.sp_1(R_12 @ kin["H"][:, 2], R_h @ e_SW, kin["H"][:, 0])
        R_01 = rot(kin["H"][:, 0], q1)
        R_02 = R_01 @ R_12

        # Solve (q3+q4+q5, q6) using Subproblem 2
        t6, t345, t3456_is_LS = subproblem.sp_2(
            kin["H"][:, 6],
            R_02.T @ R_07 @ kin["H"][:, 6],
            kin["H"][:, 5],
            -kin["H"][:, 2]
        )

        for q6, q345 in zip(t6.flatten(), t345.flatten()):
            R_25 = rot(kin["H"][:, 2], q345)
            R_56 = rot(kin["H"][:, 5], q6)

            # Then q7 using Subproblem 1
            p = kin["H"][:, 5].reshape(3,1)  # non-collinear with h7
            q7, q7_is_LS = subproblem.sp_1(
                p,
                R_56.T @ R_25.T @ R_02.T @ R_07 @ p,
                kin["H"][:, 6]
            )

            # Solve q3, q4, q5
            p_vec = R_02.T @ p_17 - R_12.T @ kin["P"][:, 1].reshape(3,1) - kin["P"][:, 2].reshape(3,1) - R_25 @ kin["P"][:, 5].reshape(3,1)
            t4, t4_is_LS = subproblem.sp_3(kin["P"][:, 4], -kin["P"][:, 3], kin["H"][:, 3], np.linalg.norm(p_vec))
            for q4 in t4.flatten():

                q3, q3_is_LS = subproblem.sp_1(
                    kin["P"][:, 3] + rot(kin["H"][:, 3], q4) @ kin["P"][:, 4],
                    p_vec,
                    kin["H"][:, 2]
                )

                # Find q5 with subtraction
                q5 = wrapToPi(q345 - q3 - q4)

                q_i = np.array([q1, q2, q3, q4, q5, q6, q7])
                Q_all.append(q_i)
                is_LS_vec.append(q1_is_LS or t3456_is_LS or q7_is_LS or t4_is_LS or q3_is_LS)

    return np.array(Q_all).T, np.array(is_LS_vec)