import numpy as np


class Kinematics:
    def __init__(self, urdf=None) -> None:
        self.urdf = urdf

    def fk(self, joint_positions):
        return np.array([0.1, 0.2, 0.3]), np.array([0, 0, 0])

    def ik(self, position, orientation, num_iters=100, threshold=1e-3):
        return np.array([0.1]*7)

    def jac(self, joint_positions):
        return np.zeros((6, 7))
