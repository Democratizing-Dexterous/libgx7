import numpy as np
import os
import pybullet as p
import math
import sys

abs_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(abs_path)

os.chdir(abs_path)

class Kinematics:
    def __init__(self, urdf='../descriptions/urdf/gx7.urdf') -> None:
        self.urdf = urdf
        p.connect(p.DIRECT)
        self.robotId = p.loadURDF(urdf, [0, 0, 0], useFixedBase=True)
        
        self.urdf_sign = np.array([-1, 1, 1, 1, 1, 1])
        
        num_joints = p.getNumJoints(self.robotId)
        
        self.valid_joints = [i for i in range(num_joints) if p.getJointInfo(self.robotId, i)[2] == p.JOINT_REVOLUTE]
        
        self.ee_link_index = 7
        
    def fk(self, qs):
        qs = np.array(qs) * self.urdf_sign
        for i, q in zip(self.valid_joints, qs):
            p.setJointMotorControl2(
                    self.robotId, 
                    i, 
                    p.POSITION_CONTROL, 
                    targetPosition=q
                )
        for i in range(200):
            p.stepSimulation()
        
        link_state = p.getLinkState(self.robotId, self.ee_link_index)
        position = link_state[4]  # Position of the link
        orientation = link_state[5]  # Orientation of the link (quaternion)
        
        rpy = p.getEulerFromQuaternion(orientation)
        
        return position, rpy
        
    def ik(self, xyz, rpy=[0, 0, 0], num_iters=300, threshold=1e-6):
        ori_q = p.getQuaternionFromEuler(rpy)
        qs = p.calculateInverseKinematics(self.robotId, self.ee_link_index, xyz, ori_q, maxNumIterations=num_iters, residualThreshold=threshold)        
        return np.array(qs)*self.urdf_sign
        

    def jac(self, qs):
        qs = np.array(qs) * self.urdf_sign
        result = p.getLinkState(self.robotId,
                            self.ee_link_index,
                            computeLinkVelocity=1,
                            computeForwardKinematics=1)
        link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot, link_vt, link_vr = result

        zero_vec = [0.0] * len(qs)
        jac_t, jac_r = p.calculateJacobian(self.robotId, self.ee_link_index, com_trn, qs.tolist(), zero_vec, zero_vec)

        jac = np.concatenate([np.array(jac_t), np.array(jac_r)], axis=0)

        return jac

if __name__ == "__main__":
    kin = Kinematics()
    
    
