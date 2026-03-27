import numpy as np

def wrapToPi(angle):
    """Wrap angle to [-pi, pi]"""

    return (angle + np.pi) % (2 * np.pi) - np.pi

def vec_normalize(vec):
    """Normalize a vector."""
    return vec / np.linalg.norm(vec)

def hat(v):
    """
    Skew-symmetric matrix (hat operator)
    v: 3-vector
    """
    v = np.asarray(v).reshape(3)
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

def rot(k, theta):
    """
    Rodrigues 旋转公式
    输入: k (3,), theta (float)
    输出: R (3,3)
    """
    k = np.asarray(k).reshape(3)
    k = k / np.linalg.norm(k)  # 保证是单位向量

    K = np.array([
        [0, -k[2], k[1]],
        [k[2], 0, -k[0]],
        [-k[1], k[0], 0]
    ])
    R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)
    return R

def fwdkin(kin, theta):
    """
    General forward kinematics for serial chain robot (POE version).

    Parameters
    ----------
    kin : dict
        Robot kinematics with keys:
            - "H": (3, n) array, joint axes
            - "P": (3, n+1) array, displacement vectors
            - "joint_type": (n,) list/array of joint types
                0: rotational
                1: prismatic
                2: mobile orientation
                3: mobile translation
    theta : array-like, shape (n,)
        Joint variables (radians for rotational joints)

    Returns
    -------
    R : ndarray, shape (3,3)
        End-effector orientation
    p : ndarray, shape (3,)
        End-effector position
    """
    H = np.asarray(kin["H"], dtype=float)
    P = np.asarray(kin["P"], dtype=float)
    joint_type = np.asarray(kin["joint_type"], dtype=int)
    theta = np.asarray(theta, dtype=float).reshape(-1)

    n = len(joint_type)
    p = P[:, 0].copy()
    R = np.eye(3)

    for i in range(n):
        if joint_type[i] in [0, 2]:  # rotational
            R = R @ rot(H[:, i], theta[i])
        elif joint_type[i] in [1, 3]:  # prismatic
            p = p + R @ (H[:, i] * theta[i])

        p = p + R @ P[:, i+1]
    p = p.reshape(3,1)
    return R, p


def robotjacobian(kin, theta):
    """
    Calculate Jacobian for serial chain robot.

    Parameters
    ----------
    kin : dict
        Robot kinematics with keys:
            - "H": (3, n) array, joint axes
            - "P": (3, n+1) array, displacement vectors
            - "joint_type": list/array of n joint types
                0: rotational
                1: prismatic
                2: mobile orientation
                3: mobile translation
    theta : array-like, shape (n,)
        Joint states (radians for rotational)

    Returns
    -------
    J : ndarray, shape (6, n)
        Jacobian matrix
    """
    H = np.asarray(kin["H"], dtype=float)
    P = np.asarray(kin["P"], dtype=float)
    joint_type = np.asarray(kin["joint_type"], dtype=int)
    theta = np.asarray(theta, dtype=float).reshape(-1)

    n = len(joint_type)
    p = P[:, 0].copy()
    R = np.eye(3)

    J = np.zeros((6, n))
    hi = np.zeros((3, n))
    pOi = np.zeros((3, n + 1))
    pOi[:, 0] = p

    # Forward kinematics step-by-step
    for i in range(n):
        if joint_type[i] in [0, 2]:  # rotational
            R = R @ rot(H[:, i], theta[i])
        elif joint_type[i] in [1, 3]:  # prismatic
            p = p + R @ (H[:, i] * theta[i])
        p = p + R @ P[:, i + 1]
        pOi[:, i + 1] = p
        hi[:, i] = R @ H[:, i]

    pOT = pOi[:, -1]

    # Compute Jacobian
    i = 0
    j = 0
    while i < n:
        if joint_type[i] == 0:  # revolute
            J[:, j] = np.concatenate((hi[:, i], hat(hi[:, i]) @ (pOT - pOi[:, i])))
        elif joint_type[i] == 1:  # prismatic
            J[:, j] = np.concatenate((np.zeros(3), hi[:, i]))
        elif joint_type[i] == 3:  # nonholonomic mobile
            # Special case: depends on i+2 (unicycle model)
            J[:, j] = np.concatenate((np.zeros(3), rot(hi[:, i + 2], theta[i + 2]) @ hi[:, i]))
            J[:, j + 1] = np.concatenate((hi[:, i + 2], hat(hi[:, i + 2]) @ (pOT - pOi[:, i + 2])))
            J = J[:, :-1]  # trim last column (same as MATLAB `J = J(:,1:end-1)`)
            i += 2
            j += 1
        i += 1
        j += 1

    return J

def circle_polynomials(p0_i, k_i, p_i, p_i_s, k2):
    """
    Represent polynomials P_i, R_i for Subproblem 5
    """
    kiXk2 = np.cross(k_i, k2)
    kiXkiXk2 = np.cross(k_i, kiXk2)
    norm_kiXk2_sq = np.dot(kiXk2, kiXk2)

    kiXpi = np.cross(k_i, p_i)
    norm_kiXpi_sq = np.dot(kiXpi, kiXpi)

    delta = np.dot(k2, p_i_s)
    alpha = np.dot(p0_i, kiXkiXk2) / norm_kiXk2_sq
    beta = np.dot(p0_i, kiXk2) / norm_kiXk2_sq

    P_const = norm_kiXpi_sq + np.dot(p_i_s, p_i_s) + 2 * alpha * delta
    P = np.array([-2 * alpha, P_const])

    R = np.array([-1, 2*delta, -delta**2])
    R[-1] += norm_kiXpi_sq * norm_kiXk2_sq
    R = (2*beta)**2 * R

    return P, R

# Placeholder implementations for sub-functions
def quartic_roots(coeffs):
    """Solve polynomial roots with highest power first (like MATLAB)."""
    # In numpy.poly1d, coeffs[0] is constant term, so reverse for compatibility
    coeffs = coeffs[::-1]
    return np.roots(coeffs)

def qr_LS_and_null(A, b):
    """
    QR decomposition to find minimum norm solution x and null space n
    """
    Q, R = np.linalg.qr(A.T)
    n = Q[:,2:4]  # Null space (last 2 columns)

    # Solve for x using first 2 cols of Q and 2x2 R submatrix
    R2 = R[0:2, 0:2].T
    x = Q[:,0:2] @ np.linalg.solve(R2, b)
    return x, n


# Placeholder function for ellipse intersection
def solve_2_ellipse_numeric(x_min1, x_null1, x_min2, x_null2):
    """
    Solve intersection of two ellipses numerically.
    This is a placeholder: implement the numeric intersection algorithm.
    """
    # For demonstration, return one solution
    xi_1 = np.array([0.0])
    xi_2 = np.array([0.0])
    return xi_1, xi_2

class subproblem:
    @staticmethod
    def sp_1(p1, p2, k):
        """
        Subproblem 1: Circle and point

        theta = sp_1(p1, p2, k) finds theta such that
                rot(k, theta) @ p1 = p2

        If there's no solution, theta minimizes the least-squares residual
                || rot(k, theta) @ p1 - p2 ||

        Parameters
        ----------
        p1 : array_like, shape (3,)
            First vector
        p2 : array_like, shape (3,)
            Target vector
        k : array_like, shape (3,)
            Unit vector (axis of rotation)

        Returns
        -------
        theta : float
            Rotation angle in radians
        is_LS : bool
            True if the solution is least-squares
        """
        p1 = np.asarray(p1).reshape(3)
        p2 = np.asarray(p2).reshape(3)*10
        k = np.asarray(k).reshape(3)

        # Compute K × p1
        KxP = np.cross(k, p1)

        # Matrix A = [KxP, -cross(k, KxP)]
        A = np.column_stack((KxP, -np.cross(k, KxP)))*10

        # x = A' * p2
        x = A.T @ p2

        theta = np.arctan2(x[0], x[1])

        # Least-squares flag
        is_LS = (abs(np.linalg.norm(p1) - np.linalg.norm(p2)) > 1e-8 or
                 abs(np.dot(k, p1) - np.dot(k, p2)) > 1e-8)

        return theta, is_LS

    @staticmethod
    def sp_2(p1, p2, k1, k2):
        """
        Subproblem 2: Two circles
        Solve theta1, theta2 such that rot(k1, theta1) @ p1 = rot(k2, theta2) @ p2
        If no exact solution, minimize || rot(k1, theta1)*p1 - rot(k2, theta2)*p2 ||

        Inputs:
            p1: 3x1 column vector
            p2: 3x1 column vector
            k1: 3x1 column vector, norm(k1)=1
            k2: 3x1 column vector, norm(k2)=1
        Outputs:
            theta1: 1xN row vector (radians)
            theta2: 1xN row vector (radians)
            is_LS: boolean, True if least-squares solution
        """
        # Normalize for least-squares case
        p1_nrm = p1 / np.linalg.norm(p1)
        p2_nrm = p2 / np.linalg.norm(p2)

        theta1, t1_is_LS = subproblem.sp_4(k2, p1_nrm, k1, float(k2.T @ p2_nrm))
        theta2, t2_is_LS = subproblem.sp_4(k1, p2_nrm, k2, float(k1.T @ p1_nrm))

        # Ensure solutions correspond by flipping theta2
        if theta1.size > 1 or theta2.size > 1:
            theta1 = np.array([theta1[0, 0], theta1[0, -1]]).reshape(1, 2)
            theta2 = np.array([theta2[0, -1], theta2[0, 0]]).reshape(1, 2)

        # Determine least-squares flag
        is_LS = (abs(np.linalg.norm(p1) - np.linalg.norm(p2)) > 1e-8) or t1_is_LS or t2_is_LS

        return theta1, theta2, is_LS

    @staticmethod
    def sp_3(p1, p2, k, d):
        """
        解方程: || p - rot(k,θ) q || = d
        返回: theta[], is_ls
        """
        c = 0.5 * (np.dot(p1.T, p1) + np.dot(p2.T, p2) - d**2)
        theta, is_LS = subproblem.sp_4(p2, p1, k, c)
        theta = np.atleast_1d(theta).reshape(1, -1)

        return theta, is_LS

    @staticmethod
    def sp_4(h, p, k, d):
        """
        Subproblem 4: Circle and Plane
        Solve theta such that h.T @ rot(k, theta) @ p = d
        If no exact solution, minimize |h.T @ rot(k,theta) @ p - d|

        Inputs:
            h: 3x1 column vector, norm(h)=1
            p: 3x1 column vector
            k: 3x1 column vector, norm(k)=1
            d: scalar
        Outputs:
            theta: 1xN row vector (radians)
            is_LS: boolean, True if least-squares solution
        """
        # Cross product and matrix A1
        A11 = np.cross(k.flatten(), p.flatten()).reshape(3, 1)  # 保持列向量
        A1 = np.hstack([A11, -np.cross(k.flatten(), A11.flatten()).reshape(3, 1)])  # 3x2

        A = h.T @ A1  # 1x2 row vector
        A = A.reshape(1, 2)

        b = d - (h.T @ k) * (k.T @ p)  # scalar

        norm_A2 = np.dot(A.flatten(), A.flatten())  # scalar

        x_ls_tilde = (A1.T @ (h * b)).reshape(2, 1)  # 2x1 column vector

        if norm_A2 > b ** 2:
            xi = np.sqrt(norm_A2 - b ** 2)
            x_N_prime_tilde = np.array([A[0, 1], -A[0, 0]]).reshape(2, 1)  # 2x1

            sc1 = x_ls_tilde + xi * x_N_prime_tilde
            sc2 = x_ls_tilde - xi * x_N_prime_tilde

            theta = np.array([np.arctan2(sc1[0, 0], sc1[1, 0]),
                              np.arctan2(sc2[0, 0], sc2[1, 0])]).reshape(1, 2)
            is_LS = False
        else:
            theta = np.array([np.arctan2(x_ls_tilde[0, 0], x_ls_tilde[1, 0])]).reshape(1, 1)
            is_LS = True

        return theta, is_LS

    @staticmethod
    def sp_5(p0, p1, p2, p3, k1, k2, k3):
        """
        Subproblem 5: Three circles
        Solve theta1, theta2, theta3 such that
            p0 + rot(k1,theta1)*p1 = rot(k2,theta2)*(p2 + rot(k3,theta3)*p3)
        """
        theta1 = np.full(4, np.nan)
        theta2 = np.full(4, np.nan)
        theta3 = np.full(4, np.nan)
        i_soln = 0

        # Projection along axes
        p1_s = p0 + k1 * np.dot(k1, p1)
        p3_s = p2 + k3 * np.dot(k3, p3)

        delta1 = np.dot(k2, p1_s)
        delta3 = np.dot(k2, p3_s)

        P_1, R_1 = circle_polynomials(p0, k1, p1, p1_s, k2)
        P_3, R_3 = circle_polynomials(p2, k3, p3, p3_s, k2)

        # Solve quartic
        P_13 = np.polynomial.polynomial.polysub(P_1, P_3)
        P_13_sq = np.polynomial.polynomial.polymul(P_13, P_13)
        RHS = np.polynomial.polynomial.polysub(R_3, R_1) - P_13_sq
        EQN = np.polynomial.polynomial.polymul(RHS, RHS) - 4 * np.polynomial.polynomial.polymul(P_13_sq, R_1)

        all_roots = quartic_roots(EQN)
        z_vec = all_roots[np.abs(np.imag(all_roots)) < 1e-6].real
        z_vec = np.unique(np.round(z_vec, 12))  # uniquetol approximation

        # Precompute matrices
        KxP1 = np.cross(k1, p1)
        KxP3 = np.cross(k3, p3)
        A_1 = np.column_stack([KxP1, -np.cross(k1, KxP1)])
        A_3 = np.column_stack([KxP3, -np.cross(k3, KxP3)])
        signs = np.array([[1, 1, -1, -1],
                          [1, -1, 1, -1]])
        J = np.array([[0, 1], [-1, 0]])

        for z in z_vec:
            if i_soln == 4:
                break

            const_1 = A_1.T @ k2 * (z - delta1)
            const_3 = A_3.T @ k2 * (z - delta3)

            norm_A1k2_sq = np.linalg.norm(A_1.T @ k2) ** 2
            norm_A3k2_sq = np.linalg.norm(A_3.T @ k2) ** 2

            if norm_A1k2_sq - (z - delta1) ** 2 < 0:
                continue
            if norm_A3k2_sq - (z - delta3) ** 2 < 0:
                continue

            pm_1 = J @ A_1.T @ k2 * np.sqrt(norm_A1k2_sq - (z - delta1) ** 2)
            pm_3 = J @ A_3.T @ k2 * np.sqrt(norm_A3k2_sq - (z - delta3) ** 2)

            for i_sign in range(4):
                if i_soln == 4:
                    break
                sign_1 = signs[0, i_sign]
                sign_3 = signs[1, i_sign]

                sc1 = (const_1 + sign_1 * pm_1) / norm_A1k2_sq
                sc3 = (const_3 + sign_3 * pm_3) / norm_A3k2_sq

                Rp_1 = A_1 @ sc1 + p1_s
                Rp_3 = A_3 @ sc3 + p3_s

                if np.abs(np.linalg.norm(Rp_1) - np.linalg.norm(Rp_3)) < 1e-6:
                    theta1[i_soln] = np.arctan2(sc1[0], sc1[1])
                    theta2[i_soln],_ = subproblem.sp_1(Rp_3, Rp_1, k2)
                    theta3[i_soln] = np.arctan2(sc3[0], sc3[1])
                    i_soln += 1

        theta1 = theta1[:i_soln]
        theta2 = theta2[:i_soln]
        theta3 = theta3[:i_soln]

        return theta1, theta2, theta3

    @staticmethod
    def sp_6(H, K, P, d1, d2):
        """
        Subproblem 6: Four circles
        Solve theta1, theta2 such that:
            h1'*rot(k1,theta1)*p1 + h2'*rot(k2,theta2)*p2 = d1
            h3'*rot(k3,theta1)*p3 + h4'*rot(k4,theta2)*p4 = d2
        H = [h1 h2 h3 h4], K = [k1 k2 k3 k4], P = [p1 p2 p3 p4]
        """
        # Build A matrices
        k1Xp1 = np.cross(K[:, 0], P[:, 0])
        k2Xp2 = np.cross(K[:, 1], P[:, 1])
        k3Xp3 = np.cross(K[:, 2], P[:, 2])
        k4Xp4 = np.cross(K[:, 3], P[:, 3])

        A_1 = np.column_stack([k1Xp1, -np.cross(K[:, 0], k1Xp1)])
        A_2 = np.column_stack([k2Xp2, -np.cross(K[:, 1], k2Xp2)])
        A_3 = np.column_stack([k3Xp3, -np.cross(K[:, 2], k3Xp3)])
        A_4 = np.column_stack([k4Xp4, -np.cross(K[:, 3], k4Xp4)])

        # Build system matrix
        A = np.block([[H[:, 0].T @ A_1, H[:, 1].T @ A_2],
                      [H[:, 2].T @ A_3, H[:, 3].T @ A_4]])

        # Right-hand side vector
        b = np.array([
            d1 - H[:, 0].T @ (K[:, 0] * (K[:, 0].T @ P[:, 0])) - H[:, 1].T @ (K[:, 1] * (K[:, 1].T @ P[:, 1])),
            d2 - H[:, 2].T @ (K[:, 2] * (K[:, 2].T @ P[:, 2])) - H[:, 3].T @ (K[:, 3] * (K[:, 3].T @ P[:, 3]))
        ])

        # Solve for minimum norm solution and null space
        x_min, x_null = qr_LS_and_null(A, b)
        x_null_1 = x_null[:, 0]
        x_null_2 = x_null[:, 1]

        # Solve intersection of two ellipses (numeric)
        xi_1, xi_2 = solve_2_ellipse_numeric(x_min[0:2], x_null[0:2, :],
                                             x_min[2:4], x_null[2:4, :])

        # Convert to theta1, theta2
        theta1 = np.full_like(xi_1, np.nan)
        theta2 = np.full_like(xi_2, np.nan)

        for i in range(len(xi_1)):
            x = x_min + x_null_1 * xi_1[i] + x_null_2 * xi_2[i]
            theta1[i] = np.arctan2(x[0], x[1])
            theta2[i] = np.arctan2(x[2], x[3])

        return theta1, theta2
