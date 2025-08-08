import pybullet as p
import pybullet_data
import time
import os
import math
from robot import GX7

directions_urdf = [-1, 1, 1, 1, -1, -1]

def main():
    # Initialize PyBullet in GUI mode
    physicsClient = p.connect(p.GUI)
    FREQ = 100  # Hz
    robot = GX7(FREQ, 'win', 'pv')
    
    
    # Set up the simulation environment
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)  # Set gravity
    
    # Load a ground plane
    planeId = p.loadURDF("plane.urdf")
    
    # Get the current directory
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Load the GX7 robot URDF with -90 degree rotation around y-axis
    urdf_path = "urdf/GX7_250721.urdf"
    # Convert -90 degrees to quaternion (rotation around y-axis)
    orientation = p.getQuaternionFromEuler([0, -math.pi/2, 0])
    robotId = p.loadURDF(urdf_path, [0, 0, 0], orientation, useFixedBase=True)
    
    # Get number of joints
    num_joints = p.getNumJoints(robotId)
    print(f"Number of joints: {num_joints}")
    
    # Print joint information
    for i in range(num_joints):
        joint_info = p.getJointInfo(robotId, i)
        print(f"Joint {i}: {joint_info[1].decode('utf-8')}, Type: {joint_info[2]}")
    
    # Enable joint control for dragging
    for i in range(num_joints):
        # Check if the joint is a revolute joint (type 0)
        joint_info = p.getJointInfo(robotId, i)
        if joint_info[2] == p.JOINT_REVOLUTE:
            # Enable joint motor control with zero target velocity and force
            p.setJointMotorControl2(robotId, i, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
    
    # Create sliders for joint control
    joint_sliders = []
    for i in range(num_joints):
        joint_info = p.getJointInfo(robotId, i)
        if joint_info[2] == p.JOINT_REVOLUTE:
            joint_name = joint_info[1].decode('utf-8')
            lower_limit = joint_info[8]
            upper_limit = joint_info[9]
            
            # Create a slider for this joint
            slider_id = p.addUserDebugParameter(
                joint_name, 
                lower_limit, 
                upper_limit, 
                0  # initial value
            )
            joint_sliders.append((i, slider_id))
    
    # Enable real-time simulation
    p.setRealTimeSimulation(1)
    
    # Enable mouse picking for dragging
    print("You can now drag the robot joints with your mouse or use the sliders.")
    print("Press Ctrl+C to exit.")
    
    robot.setup()

    robot.run()
    
    try:
        while True:
            i = 1
            # Update joint positions based on sliders
            for joint_idx, slider_id in joint_sliders:
                target_position = p.readUserDebugParameter(slider_id)
                p.setJointMotorControl2(
                    robotId, 
                    joint_idx, 
                    p.POSITION_CONTROL, 
                    targetPosition=target_position
                )
                
                
                
                robot.setJPV(i, target_position*directions_urdf[i-1], 1)
                
                i += 1
                
                if i > 6:
                    break
            
            # Get and print current joint positions
            joint_states = []
            for i in range(num_joints):
                joint_info = p.getJointInfo(robotId, i)
                if joint_info[2] == p.JOINT_REVOLUTE:
                    joint_state = p.getJointState(robotId, i)
                    joint_states.append((joint_info[1].decode('utf-8'), joint_state[0]))
            
            # Uncomment to print joint positions continuously
            # print("Current joint positions:", joint_states)
            
            time.sleep(0.01)  # Faster update rate for smoother control
    except KeyboardInterrupt:
        print("Exiting...")
    
    # Disconnect from the physics server
    p.disconnect()

if __name__ == "__main__":
    main()
