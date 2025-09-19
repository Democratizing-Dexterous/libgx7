import numpy as np
from IK_helpers.subproblem import subproblem, rot, wrapToPi



def IK_2R_3Rp_2R(R_07, p_0T, SEW_class, psi, kin):
    Q = []
    is_LS_vec = []

    # Find wrist position
    W = p_0T.reshape(3,1) - R_07 @ kin["P"][:, 7].reshape(3,1)  # MATLAB 8 → Python 7

    # Find shoulder position
    S = kin["P"][:, 0].reshape(3,1)

    p_17 = W - S
    e_SW = p_17 / np.linalg.norm(p_17)

    _, n_SEW = SEW_class.inv_kin(S, W, psi)

    # Find theta_h using Subproblem 4
    t_h, t_h_is_LS = subproblem.sp_4(
        e_SW,
        p_17,
        -n_SEW,
        np.dot(kin["H"][:, 2], np.sum(kin["P"][:, 2:6], axis=1))
    )

    for th in t_h:
        if th < 0:
            continue

        # Find (q1, q2) using Subproblem 2
        t2, t1, t12_is_LS = subproblem.sp_2(
            kin["H"][:, 2],
            rot(n_SEW, th) @ e_SW,
            kin["H"][:, 1],
            -kin["H"][:, 0]
        )

        for q1, q2 in zip(t1.flatten(), t2.flatten()):
            R_01 = rot(kin["H"][:, 0], q1)
            R_12 = rot(kin["H"][:, 1], q2)
            R_02 = R_01 @ R_12

            # Solve (q3+q4+q5, q6, q7) via spherical wrist
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
                p = kin["H"][:, 5]  # Must be non-collinear with h7
                q7, q7_is_LS = subproblem.sp_1(
                    p,
                    R_56.T @ R_25.T @ R_02.T @ R_07 @ p,
                    kin["H"][:, 6]
                )

                # Solve (q3, q4, q5)
                p_tmp = R_02.T @ p_17 - kin["P"][:, 2] - R_25 @ kin["P"][:, 5]
                t4, t4_is_LS = subproblem.sp_3(
                    kin["P"][:, 4],
                    -kin["P"][:, 3],
                    kin["H"][:, 3],
                    np.linalg.norm(p_tmp)
                )

                for q4 in t4.flatten():
                    q3, q3_is_LS = subproblem.sp_1(
                        kin["P"][:, 3] + rot(kin["H"][:, 3], q4) @ kin["P"][:, 4],
                        p_tmp,
                        kin["H"][:, 2]
                    )

                    q5 = wrapToPi(q345 - q3 - q4)

                    q_i = np.array([q1, q2, q3, q4, q5, q6, q7])
                    Q.append(q_i)

                    is_ls_flag = t_h_is_LS or t12_is_LS or t3456_is_LS or q7_is_LS or t4_is_LS or q3_is_LS
                    is_LS_vec.append(is_ls_flag)

    return np.array(Q).T, np.array(is_LS_vec)
