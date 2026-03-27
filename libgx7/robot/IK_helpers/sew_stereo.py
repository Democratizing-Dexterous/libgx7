import numpy as np
from .subproblem import rot, vec_normalize


class sew_stereo:
    def __init__(self, R, V):
        """
        R: reference vector (3,)
        V: reference vector (3,)
        """
        self.R = R.reshape(3, 1)
        self.V = V.reshape(3, 1)

    def fwd_kin(self, S, E, W):
        S = S.reshape(3, 1)
        E = E.reshape(3, 1) - S
        W = W.reshape(3, 1) - S

        w_hat = vec_normalize(W)
        n_hat_sew = vec_normalize(np.cross(W.flatten(), E.flatten()).reshape(3, 1))
        n_hat_ref = vec_normalize(
            np.cross(w_hat.flatten() - self.R.flatten(), self.V.flatten()).reshape(3, 1)
        )

        psi = np.arctan2(
            n_hat_sew.T @ np.cross(w_hat.flatten(), n_hat_ref.flatten()).reshape(3, 1),
            n_hat_sew.T @ n_hat_ref,
        ).item()

        return psi

    def alt_fwd_kin(self, S, E, W):
        S = S.reshape(3, 1)
        E = E.reshape(3, 1) - S
        W = W.reshape(3, 1) - S

        w_hat = vec_normalize(W)
        n_sew = np.cross(W.flatten(), E.flatten()).reshape(3, 1)
        n_phi = -np.cross((w_hat - self.R).flatten(), n_sew.flatten()).reshape(3, 1)

        psi = np.arctan2(
            n_phi.T @ np.cross((-self.R).flatten(), self.V.flatten()).reshape(3, 1),
            n_phi.T @ self.V,
        ).item()
        return psi

    def inv_kin(self, S, W, psi):
        S = S.reshape(3, 1)
        W = W.reshape(3, 1)
        p_SW = W - S
        e_SW = vec_normalize(p_SW)
        k_r = np.cross((e_SW - self.R).flatten(), self.V.flatten()).reshape(3, 1)
        k_x = np.cross(k_r.flatten(), p_SW.flatten()).reshape(3, 1)
        e_x = vec_normalize(k_x)

        e_CE = rot(e_SW, psi) @ e_x
        n_SEW = np.cross(e_SW.flatten(), e_CE.flatten()).reshape(3, 1)
        return e_CE, n_SEW

    def jacobian(self, S, E, W):
        S = S.reshape(3, 1)
        E = E.reshape(3, 1) - S
        W = W.reshape(3, 1) - S
        w_hat = vec_normalize(W)

        n_ref = np.cross((w_hat - self.R).flatten(), self.V.flatten()).reshape(3, 1)
        x_c = np.cross(n_ref.flatten(), W.flatten()).reshape(3, 1)
        x_hat_c = vec_normalize(x_c)
        y_hat_c = np.cross(w_hat.flatten(), x_hat_c.flatten()).reshape(3, 1)

        p = (np.eye(3) - w_hat @ w_hat.T) @ E
        p_hat = vec_normalize(p)

        J_e = (
            np.cross(w_hat.flatten(), p_hat.flatten()).reshape(1, 3) / np.linalg.norm(p)
        ).T

        J_w_1 = ((w_hat.T @ self.V).item() / np.linalg.norm(x_c)) * y_hat_c
        J_w_2 = (
            (
                w_hat.T @ np.cross(self.R.flatten(), self.V.flatten()).reshape(3, 1)
            ).item()
            / np.linalg.norm(x_c)
        ) * x_hat_c
        J_w_3 = (
            (w_hat.T @ E).item() / (np.linalg.norm(W) * np.linalg.norm(p))
        ) * np.cross(w_hat.flatten(), p_hat.flatten()).reshape(3, 1)

        J_w = J_w_1 + J_w_2 - J_w_3
        return J_e, J_w
