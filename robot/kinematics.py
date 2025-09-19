import numpy as np

from ik_solve import gx7_ik_solve, gx7_fk


class Kinematics:
    def __init__(self, limits):
        self.limits = limits

    def ik(self, pos, ori, psi=np.pi / 6):
        ik_output = gx7_ik_solve(pos, ori, psi=psi)
        lower_limits = [limit[0] * np.pi / 180 for limit in self.limits]
        upper_limits = [limit[1] * np.pi / 180 for limit in self.limits]
        ik_output_limit = []
        for i, qs in enumerate(ik_output):

            qs_normal = []
            for q, l, u in zip(qs, lower_limits, upper_limits):
                if q > np.pi:
                    q = q - 2 * np.pi
                elif q < -np.pi:
                    q = q + 2 * np.pi
                qs_normal.append(q)

                if q < l or q > u:
                    break
            if len(qs_normal) == len(lower_limits):
                # If all joint angles are within limits, add to the output list
                ik_output_limit.append(np.array(qs_normal).ravel())

        return ik_output_limit

    def fk(self, joint_angles):
        pos, ori = gx7_fk(joint_angles)
        return pos, ori
