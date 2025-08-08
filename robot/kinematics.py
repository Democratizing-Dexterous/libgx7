import numpy as np
import pinocchio as pin


class Kinematics:
    def __init__(self, urdf="../descriptions/urdf/gx7.urdf") -> None:
        self.urdf = urdf
        self.model = pin.buildModelFromUrdf(urdf)
        self.data = self.model.createData()
        self.ee_id = self.model.getFrameId("Link7")
        

    def fk(self, qs):
        pin.forwardKinematics(self.model, self.data, qs)
        pin.updateFramePlacements(self.model, self.data)
        ee_pos = self.data.oMf[self.ee_id].translation
        ee_rot = self.data.oMf[self.ee_id].rotation
        return ee_pos, ee_rot
        

    def ik(self, position, orientation, num_iters=100, threshold=1e-3):
        return np.array([0.1]*7)

    def jac(self, joint_positions):
        return np.zeros((6, 7))


if __name__ == "__main__":
    import os
    import pybullet as p
    import time
    
    # Initialize Pinocchio FK
    kinematics = Kinematics(urdf="../descriptions/urdf/gx7.urdf")
    
    # Initialize PyBullet
    physicsClient = p.connect(p.DIRECT)  # Use DIRECT mode for computation without visualization
    
    # Get the current directory and load the robot URDF
    current_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.dirname(current_dir)
    urdf_path = os.path.join(parent_dir, "descriptions/urdf/gx7.urdf")
    robotId = p.loadURDF(urdf_path, useFixedBase=True)
    
    # Get number of joints
    num_joints = p.getNumJoints(robotId)
    print(f"Number of joints in PyBullet: {num_joints}")
    
    # Print joint information
    joint_indices = []
    for i in range(num_joints):
        joint_info = p.getJointInfo(robotId, i)
        print(f"Joint {i}: {joint_info[1].decode('utf-8')}, Type: {joint_info[2]}")
        if joint_info[2] == p.JOINT_REVOLUTE:
            joint_indices.append(i)
    
    # Test with different joint configurations
    test_configs = [
        np.array([0.0] * 7),  # Zero configuration
        np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]),  # Some arbitrary configuration
        np.array([1.0, -0.5, 0.7, 0.2, -0.3, 0.8, 0.1]),  # Another configuration
    ]
    
    for config_idx, qs in enumerate(test_configs):
        print(f"\nTesting configuration {config_idx+1}: {qs}")
        
        # Calculate FK using Pinocchio
        pin_ee_pos, pin_ee_rot = kinematics.fk(qs)
        
        # Set joint positions in PyBullet
        for i, joint_idx in enumerate(joint_indices):
            if i < len(qs):
                p.setJointMotorControl2(robotId, joint_idx, p.POSITION_CONTROL, targetPosition=qs[i])
        
        for i in range(200):
            p.stepSimulation()
        
        # Get link state for the end effector (Link7)
        # Find the Link7 index
        link7_idx = None
        for i in range(num_joints):
            joint_info = p.getJointInfo(robotId, i)
            if joint_info[1].decode('utf-8') == "joint7":
                link7_idx = joint_info[0]
                break
        
        if link7_idx is not None:
            # Get the link state
            link_state = p.getLinkState(robotId, link7_idx)
            pb_ee_pos = link_state[4]  # Position
            pb_ee_orn = link_state[5]  # Orientation (quaternion)
            
            # Convert quaternion to rotation matrix for comparison
            pb_ee_rot = np.array(p.getMatrixFromQuaternion(pb_ee_orn)).reshape(3, 3)
            
            # Print and compare results
            print("Pinocchio FK Position:", pin_ee_pos)
            print("PyBullet FK Position:", pb_ee_pos)
            pos_diff = np.linalg.norm(pin_ee_pos - np.array(pb_ee_pos))
            print(f"Position difference: {pos_diff}")
            
            print("\nPinocchio FK Rotation:")
            print(pin_ee_rot)
            print("PyBullet FK Rotation:")
            print(pb_ee_rot)
            rot_diff = np.linalg.norm(pin_ee_rot - pb_ee_rot)
            print(f"Rotation difference: {rot_diff}")
            
            # Check if the results are close
            if pos_diff < 1e-3 and rot_diff < 1e-3:
                print("✓ FK calculations are consistent!")
            else:
                print("✗ FK calculations differ!")
                # Analyze the differences
                if pos_diff >= 1e-3:
                    print(f"  Position difference: {pos_diff}")
                    print(f"  Component differences: {pin_ee_pos - np.array(pb_ee_pos)}")
                if rot_diff >= 1e-3:
                    print(f"  Rotation difference: {rot_diff}")
        else:
            print("Could not find Link7 in PyBullet model")
    
    # Disconnect from PyBullet
    p.disconnect()

    # 计算Pin FK时间
    start_time = time.time()
    for i in range(1000):
        pin_ee_pos, pin_ee_rot = kinematics.fk(qs)
    end_time = time.time()
    print(f"Pinocchio FK 1K time: {end_time - start_time}")