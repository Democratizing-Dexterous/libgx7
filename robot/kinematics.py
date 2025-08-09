import numpy as np
import os
import pybullet as p

import sys

abs_path = os.path.dirname(os.path.abspath(__file__))


sys.path.append(abs_path)

class Kinematics:
    def __init__(self, urdf='../descriptions/urdf/gx7.urdf') -> None:
        self.urdf = urdf
        p.connect(p.DIRECT)
        self.robotId = p.loadURDF(urdf, [0, 0, 0.1], useFixedBase=True)
        
        num_joints = p.getNumJoints(self.robotId)
        
        self.valid_joints = [i for i in range(num_joints) if p.getJointInfo(self.robotId, i)[2] == p.JOINT_REVOLUTE]
        
        self.ee_link_index = 7
        
    def fk(self, qs):
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
        
    def ik(self, xyz, rpy=[0, 0, 0], num_iters=100, threshold=1e-5):
        ori_q = p.getQuaternionFromEuler(rpy)
        return p.calculateInverseKinematics(self.robotId, self.ee_link_index, xyz, ori_q, maxNumIterations=num_iters, residualThreshold=threshold)

        

    def jac(self, qs):
        pass


if __name__ == "__main__":
    kin = Kinematics()
    
    
