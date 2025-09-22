import numpy as np
from .subproblem import rot, vec_normalize, subproblem


class sew_conv:
    def __init__(self, e_r):
        self.e_r = e_r.reshape(3, 1)  # 确保为列向量

    def fwd_kin(self, S, E, W):
        """给定肩、肘、手腕位置，计算 SEW 角 psi"""
        S = S.reshape(3, 1)
        E = E.reshape(3, 1)
        W = W.reshape(3, 1)

        p_SE = E - S
        e_SW = vec_normalize(W - S)

        # 这里假设 subproblem.sp_1 返回标量 psi
        # 如果你已有 sp_1 函数，可以直接替换下面这一行
        psi, _ = subproblem.sp_1(self.e_r, p_SE, e_SW)
        return psi

    def inv_kin(self, S, W, psi):
        """给定肩、手腕位置和 SEW 角，求 e_CE 和 n_SEW"""
        S = S.reshape(3, 1)
        W = W.reshape(3, 1)
        e_SW = vec_normalize(W - S)
        e_y = vec_normalize(np.cross(e_SW.flatten(), self.e_r.flatten()).reshape(3, 1))

        n_SEW = rot(e_SW, psi) @ e_y
        e_CE = np.cross(n_SEW.flatten(), e_SW.flatten()).reshape(3, 1)
        return e_CE, n_SEW

    def jacobian(self, S, E, W):
        """求 SEW 角关于肘和手腕速度的雅可比"""
        S = S.reshape(3, 1)
        E = E.reshape(3, 1)
        W = W.reshape(3, 1)

        p_SE = E - S
        p_SW = W - S
        e_SW = vec_normalize(p_SW)
        k_y = np.cross(p_SW.flatten(), self.e_r.flatten()).reshape(3, 1)
        e_y = vec_normalize(k_y)

        p_CE = -np.cross(
            e_SW.flatten(), np.cross(e_SW.flatten(), p_SE.flatten())
        ).reshape(3, 1)
        e_CE = vec_normalize(p_CE)

        J_psi_E = (
            np.cross(e_SW.flatten(), e_CE.flatten()).reshape(1, 3)
            / np.linalg.norm(p_CE)
        ).T

        J_w_1 = (np.dot(e_SW.flatten(), self.e_r.flatten()) / np.linalg.norm(k_y)) * e_y
        J_w_2 = (
            np.dot(e_SW.flatten(), p_SE.flatten())
            / (np.linalg.norm(p_SW) * np.linalg.norm(p_CE))
        ) * np.cross(e_SW.flatten(), e_CE.flatten()).reshape(3, 1)

        J_psi_W = J_w_1 - J_w_2
        return J_psi_E, J_psi_W
