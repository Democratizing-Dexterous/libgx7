import numpy as np
from IK_helpers.subproblem import subproblem, rot

def IK_R_R_3R_2R(R_07, p_0T, SEW_class, psi, kin):
    Q_all = []
    is_LS_vec = []

    # Find wrist position
    W = p_0T.reshape(3,1) - R_07 @ kin["P"][:, 7].reshape(3,1)

    # Find shoulder position
    S = kin["P"][:, 0].reshape(3,1)

    p_17 = W - S
    e_SW = p_17 / np.linalg.norm(p_17)

    # Find (q1, q2, theta_W) with Subproblem 5
    _, n_SEW = SEW_class.inv_kin(S, W, psi)
    tW, t1, t2 = subproblem.sp_5(
        p_17,
        -e_SW * np.linalg.norm(kin["P"][:, 5]),
        kin["P"][:, 1],
        kin["P"][:, 2],
        n_SEW,
        kin["H"][:, 0],
        kin["H"][:, 1]
    )

    for qW, q1, q2 in zip(tW.flatten(), t3.flatten(), t4.flatten()):

        if qW > 0:
            continue



        # Find (q6, q7) with Subproblem 2
        t7, t6, t67_is_LS = subproblem.sp_2(
            R_07.T @ rot(n_SEW, qW) @ e_SW * np.linalg.norm(kin["P"][:, 5]),
            kin["P"][:, 5],
            kin["H"][:, 6],
            -kin["H"][:, 5]
        )

        for q6, q7 in zip(t6.flatten(), t7.flatten()):
            # Find (q3, q4) with Subproblem 2
            R_01 = rot(kin["H"][:, 0], q1)
            R_12 = rot(kin["H"][:, 1], q2)
            R_56 = rot(kin["H"][:, 5], q6)
            R_67 = rot(kin["H"][:, 6], q7)
            R_25 = (R_01 @ R_12).T @ R_07 @ (R_56 @ R_67).T

            t4, t3, t34_is_LS = subproblem.sp_2(
                kin["H"][:, 4],
                R_25 @ kin["H"][:, 4],
                kin["H"][:, 3],
                -kin["H"][:, 2]
            )

            for q3, q4 in zip(t3.flatten(), t4.flatten()):
                # Find q5 with Subproblem 1
                p = kin["H"][:, 5]  # non-collinear with h_5
                R_23 = rot(kin["H"][:, 2], q3)
                R_34 = rot(kin["H"][:, 3], q4)
                q5, q5_is_LS = subproblem.sp_1(
                    p,
                    (R_23 @ R_34).T @ R_25 @ p,
                    kin["H"][:, 4]
                )

                q_i = np.array([q1, q2, q3, q4, q5, q6, q7])
                Q_all.append(q_i)
                is_LS_vec.append(t67_is_LS or t34_is_LS or q5_is_LS)

    return np.array(Q_all).T, np.array(is_LS_vec)
