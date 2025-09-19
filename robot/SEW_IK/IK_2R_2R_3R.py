import numpy as np
from IK_helpers.subproblem import subproblem, rot

def IK_2R_2R_3R(R_07, p_0T, SEW_class, psi, kin):
    Q = []
    is_LS_vec = []

    # Find wrist position
    W = p_0T.reshape(3,1) - R_07 @ (kin["P"][:, 7].reshape(3,1))   # MATLAB索引8 → Python索引7

    # Find shoulder position
    S = kin["P"][:, 0].reshape(3,1)

    # Use subproblem 3 to find theta_SEW
    d_S_E = np.linalg.norm(np.sum(kin["P"][:, 1:4], axis=1))
    d_E_W = np.linalg.norm(np.sum(kin["P"][:, 4:7], axis=1))
    p_17 = W - S
    e_17 = p_17 / np.linalg.norm(p_17)

    _, n_SEW = SEW_class.inv_kin(S, W, psi)
    theta_SEW, theta_SEW_is_LS = subproblem.sp_3(d_S_E * e_17, p_17, n_SEW, d_E_W)

    # Pick theta_SEW > 0 so R_02 p_23 falls in the correct half plane
    q_SEW = np.max(theta_SEW)
    p_S_E = rot(n_SEW, q_SEW) @ (d_S_E * e_17)
    E = p_S_E + S

    # Find q_1 and q_2 using subproblem 2
    h_1 = kin["H"][:, 0]
    h_2 = kin["H"][:, 1]
    p_S_E_0 = np.sum(kin["P"][:, 1:4], axis=1)

    t1, t2, t12_is_ls = subproblem.sp_2(p_S_E, p_S_E_0, -h_1, h_2)

    for q1, q2 in zip(t1.flatten(), t2.flatten()):
        # Find q_3 and q_4 using subproblem 2
        h_3 = kin["H"][:, 2]
        h_4 = kin["H"][:, 3]
        p_E_W_0 = (np.sum(kin["P"][:, 4:7], axis=1)).reshape(3,1)

        p_E_W = W - E

        R_2 = rot(h_1, q1) @ rot(h_2, q2)
        t3, t4, t34_is_ls = subproblem.sp_2(R_2.T @ p_E_W, p_E_W_0, -h_3, h_4)

        for q3, q4 in zip(t3.flatten(), t4.flatten()):
            # Find q_5 and q_6 using subproblem 2
            h_5 = kin["H"][:, 4]
            h_6 = kin["H"][:, 5]

            R_4 = R_2 @ rot(h_3, q3) @ rot(h_4, q4)
            t5, t6, t56_is_ls = subproblem.sp_2(R_4.T @ R_07 @ kin["H"][:, 6], kin["H"][:, 6], -h_5, h_6)

            for q5, q6 in zip(t5.flatten(), t6.flatten()):
                # Find q_7
                h_7 = kin["H"][:, 6]
                R_6 = R_4 @ rot(h_5, q5) @ rot(h_6, q6)
                q7, q7_is_ls = subproblem.sp_1(h_6, R_6.T @ R_07 @ h_6, h_7)

                q_i = np.array([q1, q2, q3, q4, q5, q6, q7])
                Q.append(q_i)

                is_ls_flag = theta_SEW_is_LS or t12_is_ls or t34_is_ls or t56_is_ls or q7_is_ls
                is_LS_vec.append(is_ls_flag)

    return np.array(Q).T, np.array(is_LS_vec)