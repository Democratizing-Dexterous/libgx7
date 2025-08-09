import numpy as np
import pinocchio as pin
import os

abs_path = os.path.dirname(os.path.abspath(__file__))
urdf_path = os.path.join(abs_path, "../descriptions/urdf/gx7.urdf")

class Kinematics:
    def __init__(self, urdf=urdf_path) -> None:
        self.urdf = urdf
        self.model = pin.buildModelFromUrdf(urdf)
        self.data = self.model.createData()
        self.ee_id = self.model.getFrameId("Link7")
        

    def fk(self, qs):
        pin.forwardKinematics(self.model, self.data, np.array(qs))
        pin.updateFramePlacements(self.model, self.data)
        ee_pos = self.data.oMf[self.ee_id].translation
        ee_rot = self.data.oMf[self.ee_id].rotation
        return ee_pos, ee_rot
        

    def ik(self, position, orientation, num_iters=100, threshold=1e-3):
        

    def jac(self, qs):
        return pin.computeJointJacobians(self.model, self.data, np.array(qs))


if __name__ == "__main__":
    kin = Kinematics()
    
    
