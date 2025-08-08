import numpy as np
from robot import GX7
import time
from robot.dynamics import Dynamics


FREQ = 100  # Hz
robot = GX7(FREQ, 'pvt', soft_limit=False)

dt = 1/FREQ
robot.setup()
robot.run()  


dynamics = Dynamics()


urdf_sign = np.array([-1, 1, 1, 1, -1, -1])


robot.switch_mit()

time.sleep(0.5)

qd_zero = np.zeros((1, 6))
qdd_zero = np.zeros((1, 6))

while True:
    q = robot.getJP()
    
    tau = dynamics.calc(np.array(q), qd_zero, qdd_zero)
    tau = tau*urdf_sign[:, None]
    
    robot.setJTs(tau.ravel().tolist())
    
    time.sleep(dt)
    
