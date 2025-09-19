import numpy as np
from IK_helpers.subproblem import subproblem, rot, wrapToPi

def IK_3R_R_3R(R_07, p_0T, SEW_class, psi, kin):
    Q_all = []
    is_LS_vec = []

    # Wrist and shoulder positions
    W = p_0T.reshape(3,1) - R_07 @ kin["P"][:, 7].reshape(3,1)  # MATLAB 8 → Python 7
    S = kin["P"][:, 0].reshape(3,1)

    p_SW = W - S
    e_SW = p_SW / np.linalg.norm(p_SW)
    e_CE, n_SEW = SEW_class.inv_kin(S, W, psi)

    # Subproblem 3 to find q4
    t4, t4_is_LS = subproblem.sp_3(kin["P"][:, 4], -kin["P"][:, 3], kin["H"][:, 3], np.linalg.norm(p_SW))

    for q4 in t4.flatten():
        # Subproblem 2 for theta_b, theta_c
        t_b, t_c, t_bc_is_LS = subproblem.sp_2(
            p_SW,
            kin["P"][:, 3] + rot(kin["H"][:, 3], q4) @ kin["P"][:, 4],
            -n_SEW,
            e_SW
        )
        theta_b = t_b[0,0]
        theta_c = t_c[0,0]

        # Subproblem 4 for theta_a
        t_a, t_a_is_LS = subproblem.sp_4(
            n_SEW,
            rot(n_SEW, theta_b) @ rot(e_SW, theta_c) @ kin["P"][:, 3],
            e_SW,
            0
        )

        for theta_a in t_a.flatten():
            R_03 = rot(e_SW, theta_a) @ rot(n_SEW, theta_b) @ rot(e_SW, theta_c)
            if e_CE.T @ R_03 @ kin["P"][:, 3] < 0:
                continue  # Shoulder must be in correct half plane

            p_spherical_1 = kin["H"][:, 1].reshape(3,1)  # Non-collinear with h3

            # Solve q1, q2, q3 using subproblems 2 and 1
            t2, t1, t12_is_LS = subproblem.sp_2(
                kin["H"][:, 2],
                R_03 @ kin["H"][:, 2],
                kin["H"][:, 1],
                -kin["H"][:, 0]
            )

            for q1, q2 in zip(t1.flatten(), t2.flatten()):
                q3, q3_is_LS = subproblem.sp_1(
                    p_spherical_1,
                    (rot(kin["H"][:, 0], q1) @ rot(kin["H"][:, 1], q2)).T @ R_03 @ p_spherical_1,
                    kin["H"][:, 2].reshape(3,1)
                )

                # Solve q5, q6, q7 using subproblems 2 and 1
                R_01 = rot(kin["H"][:, 0], q1)
                R_12 = rot(kin["H"][:, 1], q2)
                R_23 = rot(kin["H"][:, 2], q3)
                R_34 = rot(kin["H"][:, 3], q4)
                R_47 = (R_01 @ R_12 @ R_23 @ R_34).T @ R_07

                p_spherical_2 = kin["H"][:, 5]  # Non-collinear with h7
                t6, t5, t56_is_LS = subproblem.sp_2(
                    kin["H"][:, 6],
                    R_47 @ kin["H"][:, 6],
                    kin["H"][:, 5],
                    -kin["H"][:, 4]
                )

                for q5, q6 in zip(t5.flatten(), t6.flatten()):
                    q7, q7_is_LS = subproblem.sp_1(
                        p_spherical_2,
                        (rot(kin["H"][:, 4], q5) @ rot(kin["H"][:, 5], q6)).T @ R_47 @ p_spherical_2,
                        kin["H"][:, 6]
                    )
                    q1 = wrapToPi(q1)
                    q2 = wrapToPi(q2)
                    q3 = wrapToPi(q3)
                    q4 = wrapToPi(q4)
                    q5 = wrapToPi(q5)
                    q6 = wrapToPi(q6)
                    q7 = wrapToPi(q7)

                    q_i = np.array([q1, q2, q3, q4, q5, q6, q7])
                    Q_all.append(q_i)
                    is_ls_flag = t4_is_LS or t_bc_is_LS or t_a_is_LS or t12_is_LS or q3_is_LS or t56_is_LS or q7_is_LS
                    is_LS_vec.append(is_ls_flag)

    return np.array(Q_all).T, np.array(is_LS_vec)
