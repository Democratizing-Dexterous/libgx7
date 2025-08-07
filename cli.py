import numpy as np
from robot import GX7


FREQ = 100  # Hz
robot = GX7(FREQ, 'win', 'pvt')
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


