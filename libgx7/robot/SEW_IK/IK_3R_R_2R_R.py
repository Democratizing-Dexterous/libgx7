import numpy as np
from IK_helpers.subproblem import subproblem, rot

def IK_3R_R_2R_R(R_07, p_0T, SEW_class, psi, kin, search_1D):
    """
    IK for 3R-R-2R-R manipulator.
    search_1D: callable similar to MATLAB search_1D(func, start, end, num, verbose)
    """
    Q_all = []
    is_LS_vec = []

    # Wrist and shoulder positions
    W = p_0T - R_07 @ kin["P"][:, 7].reshape(3,1)  # MATLAB 8 → Python 7
    S = kin["P"][:, 0].reshape(3,1)

    p_17 = W - S
    e_SW = p_17 / np.linalg.norm(p_17)
    _, n_SEW = SEW_class.inv_kin(S, W, psi)

    # Inner function: check q4 solvability given TS
    def q4_solvability_given_TS(theta_S):
        q_solns_partial = np.full((3, 4), np.nan)
        e = np.full(4, np.nan)
        i_soln = 0

        p_14 = rot(n_SEW, theta_S) @ e_SW * np.linalg.norm(kin["P"][:, 3])

        # Subproblem 3 for q7
        t7, t7_is_LS = subproblem.sp_3(kin["P"][:, 6], R_07.T @ (p_17 - p_14), -kin["H"][:, 6], np.linalg.norm(kin["P"][:, 4]))
        if t7_is_LS:
            return e, q_solns_partial

        for q7 in t7.flatten():
            R_67 = rot(kin["H"][:, 6], q7)
            t5, t6, t56_is_LS = subproblem.sp_2(
                kin["P"][:, 4],
                R_67 @ R_07.T @ (p_17 - p_14) - kin["P"][:, 6],
                -kin["H"][:, 4],
                kin["H"][:, 5]
            )
            if t56_is_LS:
                i_soln += 2
                continue

            for q5, q6 in zip(t5.flatten(), t6.flatten()):
                R_45 = rot(kin["H"][:, 4], q5)
                R_56 = rot(kin["H"][:, 5], q6)
                R_47 = R_45 @ R_56 @ R_67

                e[i_soln] = kin["H"][:, 3].T @ (kin["P"][:, 3] - R_47 @ R_07.T @ p_14)
                q_solns_partial[:, i_soln] = [q5, q6, q7]
                i_soln += 1

        return e, q_solns_partial

    # Inner function: complete solution given q5, q6, q7 and TS
    def q_given_q567(q567, theta_S):
        Q_partial = []
        is_LS_partial = []

        q5, q6, q7 = q567
        p_14 = rot(n_SEW, theta_S) @ e_SW * np.linalg.norm(kin["P"][:, 3])
        R_45 = rot(kin["H"][:, 4], q5)
        R_56 = rot(kin["H"][:, 5], q6)
        R_67 = rot(kin["H"][:, 6], q7)
        R_47 = R_45 @ R_56 @ R_67

        # Find q4 using sp_1
        q4, q4_is_LS = subproblem.sp_1(kin["P"][:, 3], R_47 @ R_07.T @ p_14, -kin["H"][:, 3])
        R_34 = rot(kin["H"][:, 3], q4)
        R_03 = R_07 @ R_47.T @ R_34.T

        # Solve spherical shoulder: q1, q2, q3
        t2, t1, t12_is_LS = subproblem.sp_2(kin["H"][:, 2], R_03 @ kin["H"][:, 2], kin["H"][:, 1], -kin["H"][:, 0])

        for q1, q2 in zip(t1.flatten(), t2.flatten()):
            R_01 = rot(kin["H"][:, 0], q1)
            R_12 = rot(kin["H"][:, 1], q2)
            p = kin["H"][:, 1]
            q3, q3_is_LS = subproblem.sp_1(p, R_12.T @ R_01.T @ R_03 @ p, kin["H"][:, 2])

            q_i = np.array([q1, q2, q3, q4, q5, q6, q7])
            Q_partial.append(q_i)
            is_LS_partial.append(q4_is_LS or t12_is_LS or q3_is_LS)

        return Q_partial, is_LS_partial

    # 1D search over TS
    TS_vec, soln_num_vec = search_1D(q4_solvability_given_TS, 0, np.pi, 200, False)

    for i, TS in enumerate(TS_vec):
        _, q_solns_partial_i = q4_solvability_given_TS(TS)
        q_partial_i = q_solns_partial_i[:, soln_num_vec[i]]
        Q_part, is_LS_part = q_given_q567(q_partial_i, TS)
        Q_all.extend(Q_part)
        is_LS_vec.extend(is_LS_part)

    return np.array(Q_all).T, np.array(is_LS_vec)
