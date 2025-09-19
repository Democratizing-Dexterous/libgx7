import numpy as np
from IK_helpers.subproblem import rot  # 你需要确保有 rot 函数


def fwdkin_inter(kin, theta, inter):
    """
    Forward kinematics with intermediate positions.

    Parameters
    ----------
    kin : robot kinematics object
        Must have kin.P (3xN), kin.H (3xN), kin.joint_type (N,)
    theta : array_like
        Joint values (Nx1 or 1xN)
    inter : list
        List of joint indices (1-based in MATLAB, convert to 0-based in Python)

    Returns
    -------
    R : 3x3 array
        End-effector rotation matrix
    p : 3x1 array
        End-effector position
    p_inter : 3xlen(inter) array
        Intermediate positions at specified joints
    """
    theta = np.ravel(theta)
    p = kin['P'][:, 0].reshape(3, 1)
    R = np.eye(3)
    p_inter = np.full((3, len(inter)), np.nan)
    i_inter = 0

    for i in range(len(kin['joint_type'])):
        if i in inter:
            p_inter[:, i_inter] = p[:, 0]
            i_inter += 1

        if kin['joint_type'][i] in [0, 2]:  # rotational
            R = R @ rot(kin['H'][:, i].reshape(3, 1), theta[i])
        elif kin['joint_type'][i] in [1, 3]:  # prismatic
            p = p + R @ kin['H'][:, i].reshape(3, 1) * theta[i]

        p = p + R @ (kin['P'][:, i+1].reshape(3, 1))

    return R, p, p_inter
