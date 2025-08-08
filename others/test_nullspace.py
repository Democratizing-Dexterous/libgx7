import pybullet as p
import pybullet_data
import numpy as np
import time
import math

from robot import GX7

directions_urdf = [-1, 1, 1, 1, -1, -1]

def getJointStates(robot):
  joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
  joint_positions = [state[0] for state in joint_states]
  joint_velocities = [state[1] for state in joint_states]
  joint_torques = [state[3] for state in joint_states]
  return joint_positions, joint_velocities, joint_torques


def getMotorJointStates(robot):
  joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
  joint_infos = [p.getJointInfo(robot, i) for i in range(p.getNumJoints(robot))]
  joint_states = [j for j, i in zip(joint_states, joint_infos) if i[3] > -1]
  joint_positions = [state[0] for state in joint_states]
  joint_velocities = [state[1] for state in joint_states]
  joint_torques = [state[3] for state in joint_states]
  return joint_positions, joint_velocities, joint_torques


def setJointPosition(robot, position, kp=1.0, kv=0.3):
  num_joints = p.getNumJoints(robot)
  zero_vec = [0.0] * num_joints
  if len(position) == num_joints:
    p.setJointMotorControlArray(robot,
                                range(num_joints),
                                p.POSITION_CONTROL,
                                targetPositions=position,
                                targetVelocities=zero_vec,
                                positionGains=[kp] * num_joints,
                                velocityGains=[kv] * num_joints)




def multiplyJacobian(robot, jacobian, vector):
  result = [0.0, 0.0, 0.0]
  i = 0
  for c in range(len(vector)):
    if p.getJointInfo(robot, c)[3] > -1:
      for r in range(3):
        result[r] += jacobian[r][i] * vector[c]
      i += 1
  return result


p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

time_step = 0.001
gravity_constant = -9.81
p.resetSimulation()
p.setTimeStep(time_step)
p.setGravity(0.0, 0.0, gravity_constant)

p.loadURDF("plane.urdf", [0, 0, -0.3])

kukaId = p.loadURDF("./urdf/GX7_250721.urdf", useFixedBase=True)

FREQ = 100  # Hz
robot = GX7(FREQ, 'pv')

valid_joints = []
for i in range(p.getNumJoints(kukaId)):
    info = p.getJointInfo(kukaId, i)
    print(info[1])
    if info[2] == p.JOINT_REVOLUTE:
        
        valid_joints.append(i)
#kukaId = p.loadURDF("TwoJointRobot_w_fixedJoints.urdf",[0,0,0])
#kukaId = p.loadURDF("kuka_iiwa/model.urdf",[0,0,0])
#kukaId = p.loadURDF("kuka_lwr/kuka.urdf",[0,0,0])
#kukaId = p.loadURDF("humanoid/nao.urdf",[0,0,0])
# p.resetBasePositionAndOrientation(kukaId, [0, 0, 0])
numJoints = p.getNumJoints(kukaId)
kukaEndEffectorIndex = numJoints - 1

xyz = [0, 0, 0.6]
# Set a joint target for the position control and step the sim.
joint_positions = p.calculateInverseKinematics(kukaId, 6, xyz, maxNumIterations=300, residualThreshold=1e-4)


robot.setup()
robot.run()

for j in range(10):
    # 只控制有效的关节
    idx = 0
    for i, joint_position in zip(valid_joints, joint_positions):
        p.setJointMotorControl2(kukaId, i, p.POSITION_CONTROL, joint_position)
        idx += 1
        
        if idx <=6:
            robot.setJPV(idx, joint_position*directions_urdf[idx-1], 1)
        
    
    p.stepSimulation()
    
direction = 1
j = 0
duration = 1200
step_size = 0.03

while True:
    # Get the joint and link state directly from Bullet.
    pos, vel, torq = getJointStates(kukaId)
    mpos, mvel, mtorq = getMotorJointStates(kukaId)

    result = p.getLinkState(kukaId,
                            kukaEndEffectorIndex,
                            computeLinkVelocity=1,
                            computeForwardKinematics=1)
    link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot, link_vt, link_vr = result
    # Get the Jacobians for the CoM of the end-effector link.
    # Note that in this example com_rot = identity, and we would need to use com_rot.T * com_trn.
    # The localPosition is always defined in terms of the link frame coordinates.

    zero_vec = [0.0] * len(mpos)
    jac_t, jac_r = p.calculateJacobian(kukaId, kukaEndEffectorIndex, com_trn, mpos, zero_vec, zero_vec)

    jac = np.concatenate([np.array(jac_t), np.array(jac_r)], axis=0)

    # 获取雅可比矩阵的实际维度
    jac_rows, jac_cols = jac.shape

    # 使用SVD分解
    u, s, vh = np.linalg.svd(jac, full_matrices=True)

    
    null_space = vh[-1].T  # 取V^H矩阵的最后一行的转置
    
    # if np.linalg.norm(null_space) < 1e-5:
    #     direction *= -1
    
    
    error = np.linalg.norm(jac@null_space)
    
   
    
    # 设置步长，控制移动速度
    
    m_pos_new = np.array(mpos) + null_space * step_size * direction
    
    j += 1    
    
    if j % duration == 0:
        j = 0
        direction *= -1
        
    idx = 0
    # 只控制有效的关节
    for i, joint_position in zip(valid_joints, m_pos_new):
        p.setJointMotorControl2(kukaId, i, p.POSITION_CONTROL, joint_position)
        
        idx += 1
        
        if idx <=6:
            robot.setJPV(idx, joint_position*directions_urdf[idx-1], 1)
    
    # print(m_pos_new)
    
    p.stepSimulation()
    time.sleep(1/240.)
