import pybullet as p
import pybullet_data
import time
import os
import math
import numpy as np

directions_urdf = [-1, 1, 1, 1, -1, -1]

def main():
    # Initialize PyBullet in GUI mode
    physicsClient = p.connect(p.GUI)
    
    
    # Set up the simulation environment
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)  # Set gravity
    
    # Load a ground plane
    planeId = p.loadURDF("plane.urdf")
    
    # Get the current directory
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Load the GX7 robot URDF with -90 degree rotation around y-axis
    urdf_path = "../descriptions/urdf/gx7.urdf"
    # Convert -90 degrees to quaternion (rotation around y-axis)
    orientation = p.getQuaternionFromEuler([0, 0, 0])
    robotId = p.loadURDF(urdf_path, [0, 0, 0.1], orientation, useFixedBase=True)
    
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
    
    # Create debug line IDs for the last link frame visualization
    line_ids = [-1, -1, -1]  # IDs for X, Y, Z axes lines
    
    try:
        while True:

            # Update joint positions based on sliders
            for joint_idx, slider_id in joint_sliders:
                target_position = p.readUserDebugParameter(slider_id)
                p.setJointMotorControl2(
                    robotId, 
                    joint_idx, 
                    p.POSITION_CONTROL, 
                    targetPosition=target_position
                )

                
            
            # Get and print current joint positions
            joint_states = []
            for i in range(num_joints):
                joint_info = p.getJointInfo(robotId, i)
                if joint_info[2] == p.JOINT_REVOLUTE:
                    joint_state = p.getJointState(robotId, i)
                    joint_states.append((joint_info[1].decode('utf-8'), joint_state[0]))
            
            # Uncomment to print joint positions continuously
            # print("Current joint positions:", joint_states)
            
            # Get the state of the last link (Link7)
            # Find the link index for Link7
            link_name = "Link7"
            link_index = -1
            for i in range(num_joints):
                joint_info = p.getJointInfo(robotId, i)
                if joint_info[12].decode('utf-8') == link_name:
                    link_index = i
                    break
            
            if link_index != -1:
                
                # Get the position and orientation of the last link
                link_state = p.getLinkState(robotId, link_index)
                position = link_state[4]  # Position of the link
                orientation = link_state[5]  # Orientation of the link (quaternion)
                
                print(position, p.getEulerFromQuaternion(orientation))
                
                # Convert quaternion to rotation matrix
                rot_matrix = p.getMatrixFromQuaternion(orientation)
                rot_matrix = np.array(rot_matrix).reshape(3, 3).T
                
                # Define the length of the coordinate axes
                axis_length = 0.05
                
                # Calculate the endpoints for the three axes using rot_matrix
                x_end = [position[0] + axis_length * rot_matrix[0][0],
                         position[1] + axis_length * rot_matrix[0][1],
                         position[2] + axis_length * rot_matrix[0][2]]
                
                y_end = [position[0] + axis_length * rot_matrix[1][0],
                         position[1] + axis_length * rot_matrix[1][1],
                         position[2] + axis_length * rot_matrix[1][2]]
                
                z_end = [position[0] + axis_length * rot_matrix[2][0],
                         position[1] + axis_length * rot_matrix[2][1],
                         position[2] + axis_length * rot_matrix[2][2]]
                
                # Draw the coordinate frame axes (replace existing lines if they exist)
                # X-axis (red)
                if line_ids[0] != -1:
                    p.removeUserDebugItem(line_ids[0])
                line_ids[0] = p.addUserDebugLine(position, x_end, [1, 0, 0], 3)
                
                # Y-axis (green)
                if line_ids[1] != -1:
                    p.removeUserDebugItem(line_ids[1])
                line_ids[1] = p.addUserDebugLine(position, y_end, [0, 1, 0], 3)
                
                # Z-axis (blue)
                if line_ids[2] != -1:
                    p.removeUserDebugItem(line_ids[2])
                line_ids[2] = p.addUserDebugLine(position, z_end, [0, 0, 1], 3)
            
            time.sleep(0.01)  # Faster update rate for smoother control
    except KeyboardInterrupt:
        print("Exiting...")
    
    # Disconnect from the physics server
    p.disconnect()

if __name__ == "__main__":
    main()
