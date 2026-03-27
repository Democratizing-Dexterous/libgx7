import numpy as np
from .SEW_IK.IK_3R_R_3R import IK_3R_R_3R
from .IK_helpers.subproblem import rot, wrapToPi
from .IK_helpers.sew_stereo import sew_stereo
from .IK_helpers.fwdkin_inter import fwdkin_inter
import numpy as np

# hard fix
ori_align = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])

ee_align = np.eye(4)
ee_align[2, -1] = 0.099


def gx7_ik_solve(pos, ori, psi=np.pi / 6):

    ee_pose = np.eye(4)
    ee_pose[:3, :3] = ori
    ee_pose[:3, 3] = pos.ravel()

    ee_pose = ee_pose @ np.linalg.inv(ee_align)

    pos = ee_pose[:3, 3]
    ori = ee_pose[:3, :3]

    ori = ori @ ori_align.T

    kin = {}
    kin["joint_type"] = [0, 0, 0, 0, 0, 0, 0]
    kin["H"] = np.array(
        [[0, 0, -1, 0, 1, 0, 1], [0, 1, 0, -1, 0, -1, 0], [1, 0, 0, 0, 0, 0, 0]]
    )
    kin["P"] = np.array(
        [
            [0, 0, 0, -0.32099, 0.2670, 0, 0, 0],
            [0, 0, 0, -0.00000001, 0, 0, 0, 0],
            [0.1195, 0, 0, 0.077, 0, 0, 0, 0],
        ]
    )

    pos = pos.reshape(3, 1)

    SEW = sew_stereo(np.array([[0], [0], [-1]]), np.array([[0], [1], [0]]))
    Q, is_LS_vec = IK_3R_R_3R(ori, pos, SEW, psi, kin)
    return Q.T


def gx7_fk(joint_angles):
    kin = {}
    kin["joint_type"] = [0, 0, 0, 0, 0, 0, 0]

    kin["H"] = np.array(
        [[0, 0, -1, 0, 1, 0, 1], [0, 1, 0, -1, 0, -1, 0], [1, 0, 0, 0, 0, 0, 0]]
    )
    kin["P"] = np.array(
        [
            [0, 0, 0, -0.32099, 0.2670, 0, 0, 0],
            [0, 0, 0, -0.00000001, 0, 0, 0, 0],
            [0.1195, 0, 0, 0.077, 0, 0, 0, 0],
        ]
    )

    joint_angles = joint_angles.reshape(-1)
    ori, pos, sew = fwdkin_inter(kin, joint_angles, inter=[1, 3, 5])
    ori = ori @ ori_align

    ee_pose = np.eye(4)
    ee_pose[:3, :3] = ori
    ee_pose[:3, 3] = pos.ravel()

    ee_pose = ee_pose @ ee_align

    pos = ee_pose[:3, 3]
    ori = ee_pose[:3, :3]

    return pos, ori


if __name__ == "__main__":
    # Test the IK solver with a sample position and orientation
    target_position = np.array([0.4, 0, 0.2])
    target_orientation = np.array(
        [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]
    )  # Identity matrix for no rotation

    ik_solutions = gx7_ik_solve(target_position, target_orientation, psi=0)

    lower_limits = [-120, 0, -120, 0, -120, -91, -180]
    upper_limits = [120, 180, 120, 120, 120, 91, 180]

    lower_limits = np.array(lower_limits) * np.pi / 180
    upper_limits = np.array(upper_limits) * np.pi / 180

    ik_solutions_valid = []
    for i, qs in enumerate(ik_solutions):

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
            ik_solutions_valid.append(np.array(qs_normal).ravel())

    print("IK Solutions:")
    for sol in ik_solutions_valid:
        # sol = 0*sol - 0.1
        print(sol)
        pos, ori = gx7_fk(sol)
        print(f"FK Position:\n {pos.ravel()}\n")
        print(f"FK Orientation:\n {ori}\n")

    import roboticstoolbox as rtb

    mdh_parameters = [
        [np.pi, 0.1195, 0, 0],
        [np.pi / 2, 0, 0, np.pi / 2],
        [0, 0.32099, 0, np.pi / 2],
        [0, 0, 0.077, np.pi / 2],
        [-np.pi, 0.267, 0, np.pi / 2],
        [np.pi, 0, 0, np.pi / 2],
        [np.pi, 0.099, 0, np.pi / 2],
    ]

    mdh_config = []
    for i, params in enumerate(mdh_parameters):
        theta, d, a, alpha = params

        mdh_config.append(rtb.RevoluteMDH(d=d, a=a, alpha=alpha, offset=theta))

    robot = rtb.DHRobot(mdh_config, name="gx7")

    T = robot.fkine(sol).data[0]

    pos = T[:3, 3]

    ori = T[:3, :3]

    print("MDH RTB")
    print("pos=\n", pos)

    print("ori=\n", ori)
