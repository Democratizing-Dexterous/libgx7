import numpy as np
from robot import GX7

FREQ = 100  # Hz
robot = GX7(FREQ, 'win')
robot.run()  # Start the robot thread

dt = 1 / FREQ

# Print some instructions for interactive mode
print("\nRobot is running in the background.")
print("You can access robot data using the following commands:")
print("  robot.getJP()             - Get joint positions")
print("  robot.getJV()             - Get joint velocities")
print("  robot.getJT()             - Get joint torques")
print("  robot.getRotorTemp()      - Get rotor temperatures")
print("  robot.getMossTemp()       - Get MOSFET temperatures")
print("  robot.fk(robot.getJP())   - Get end effector position")
print("  robot.get_status_summary() - Get a complete status summary")
print("\nYou can also control the robot with:")
print("  robot.setJPVT(positions, velocities, torques) - Set position, velocity, and torque in PVT mode")
print("  robot.setJPV(positions, velocities)          - Set position and velocity in PV mode")
print("  robot.setJP(positions)                       - Set position in MIT mode")
print("  robot.switch_pvt()                           - Switch to PVT mode")
print("  robot.switch_pv()                            - Switch to PV mode")
print("  robot.switch_mit()                           - Switch to MIT mode")


# # time.sleep(1)

# # # robot.setJPVT([0.3, 1.2, 0.6], [0.5]*3, [0.1]*3)
# # # start_t = time.time()

# # # robot.switch_pv()

# # # while True:
# # #     robot.setPV([-0.3, 0.2, -0.6], [0.5]*3)
# # #     time.sleep(3)
# # #     robot.setPV([0.3, 1.2, 0.6], [0.5]*3)

# # #     time.sleep(3)

# j_ps1 = [0.014686808575570254, 0.7303349355306317, 0.14248111696040233, 0.9458686198214696, -0.04482337682154558, 0.478942549782559]

# j_ps = j_ps1
# robot.setJPVT(j_ps, [0.3]*len(j_ps), [0.35]*len(j_ps))
# # # robot.setJPVT([0], [0.5], [0.1])
# time.sleep(5)

# while True:
#     j_ps2 = [0.3587777523460751, 1.1137178606851297, -0.06008239871824195, 0.6528953994048976, 1.6088731212329286, 1.177424277103837]
#     j_ps = j_ps2
#     robot.setJPVT(j_ps, [0.3]*len(j_ps), [0.35]*len(j_ps))
#     time.sleep(5)

#     j_ps3 = [0.31834134431982974, 1.8560692759594115, -0.5296787975890744, 1.059929808499275, -0.7291905088883794, 1.1297398336766609]
#     j_ps = j_ps3
#     robot.setJPVT(j_ps, [0.3]*len(j_ps), [0.35]*len(j_ps))
#     time.sleep(5)


# j_ps = [
#     0.014686808575570254,
#     0.7303349355306317,
#     0.14248111696040233,
#     0.9458686198214696,
#     -0.04482337682154558,
#     0.478942549782559,
# ]
# robot.setJPVT(j_ps, [0.3] * len(j_ps), [0.35] * len(j_ps))
# # # robot.setJPVT([0], [0.5], [0.1])
# time.sleep(5)

# start_t = time.time()
# while True:
#     t = time.time() - start_t
#     j = [
#         1.2*np.sin(0.6 * t),
#         np.pi / 2 + 1.2*np.sin(0.6 * t - np.pi),
#         1.2*np.sin(0.6 * t + np.pi),
#         j_ps[3] + 0.3*np.sin(0.6 * t - np.pi),
#         j_ps[4] + 0.5*np.sin(1.6 * t - np.pi),
#         j_ps[5] + 0.2*np.sin(1.6 * t - np.pi),
#     ]
#     robot.setJPVT(j, [4]*len(j), [0.7]*len(j))
#     # robot.setJPVT([np.pi/2*np.sin(1*t)], [1], [0.1])
#     time.sleep(dt)

# robot.switch_mit()
# print(robot.getP())


# time.sleep(3)
# print(robot.getP())
