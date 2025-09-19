import numpy as np
from IK_helpers.subproblem import subproblem, rot

def IK_2R_3R_2R(R_07, p_0T, SEW_class, psi, kin):
    Q = []
    is_LS_vec = []

    # Find wrist position
    W = p_0T.reshape(3,1) - R_07 @ (kin["P"][:, 7].reshape(3,1))  # MATLAB 索引 8 → Python 索引 7

    # Find shoulder position
    S = kin["P"][:, 0].reshape(3,1)

    # Use subproblem 3 to find theta_S
    p_17 = W - S
    e_17 = p_17 / np.linalg.norm(p_17)

    _, n_SEW = SEW_class.inv_kin(S, W, psi)
    theta_S, theta_S_is_LS = subproblem.sp_3(np.linalg.norm(kin["P"][:, 2]) * e_17, p_17, n_SEW, np.linalg.norm(kin["P"][:, 5]))

    # Pick theta_S > 0 so R_02 p_23 falls in the correct half plane
    q_S = np.max(theta_S)
    p_S_E = rot(n_SEW, q_S) @ (np.linalg.norm(kin["P"][:, 2]) * e_17)

    # Find q_1 and q_2 using subproblem 2
    h_1 = kin["H"][:, 0]
    h_2 = kin["H"][:, 1]
    t1, t2, t12_is_ls = subproblem.sp_2(p_S_E, kin["P"][:, 2], -h_1, h_2)

    for q1, q2 in zip(t1.flatten(), t2.flatten()):
        R_02 = rot(h_1, q1) @ rot(h_2, q2)

        # Find q_6 and q_7 using subproblem 2
        t7, t6, t67_is_ls = subproblem.sp_2(R_07.T @ (p_17 - R_02 @ kin["P"][:, 2]), kin["P"][:, 5], kin["H"][:, 6], -kin["H"][:, 5])

        for q6, q7 in zip(t6.flatten(), t7.flatten()):
            # Find q_3 and q_4 using subproblem 2
            R_57 = rot(kin["H"][:, 5], q6) @ rot(kin["H"][:, 6], q7)
            t3, t4, t34_is_ls = subproblem.sp_2(R_02.T @ R_07 @ R_57.T @ kin["H"][:, 4], kin["H"][:, 4], -kin["H"][:, 2], kin["H"][:, 3])

            for q3, q4 in zip(t3.flatten(), t4.flatten()):
                # Find q_5
                R_24 = rot(kin["H"][:, 2], q3) @ rot(kin["H"][:, 3], q4)
                p = kin["H"][:, 5]  # Can't be collinear with h_5
                q5, q5_is_ls = subproblem.sp_1(p, R_24.T @ R_02.T @ R_07 @ R_57.T @ p, kin["H"][:, 4])

                q_i = np.array([q1, q2, q3, q4, q5, q6, q7])
                Q.append(q_i)

                is_ls_flag = theta_S_is_LS or t12_is_ls or t67_is_ls or t34_is_ls or q5_is_ls
                is_LS_vec.append(is_ls_flag)

    return np.array(Q).T, np.array(is_LS_vec)
