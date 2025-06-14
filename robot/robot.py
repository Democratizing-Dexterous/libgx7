import time
import threading
import platform
import os
from dataclasses import dataclass

from motors import CAN, RobotMotors
from utils.utils import set_high_priority, precise_sleep
from utils.utils import MotorMITCmd, MotorPVTCmd

MODE_MIT = 1
MODE_PVT = 4

@dataclass
class ControlState:
    current_control_state: int = MODE_PVT
    prev_control_state: int = MODE_PVT
    
class Robot:
    def __init__(self, freq=300):
        self.control_state = ControlState()
        can = CAN()
        self.freq = freq
        self.robot_motors = RobotMotors(can)
        
        self.motors_mit_cmd = [MotorMITCmd()]*self.robot_motors.num_motors
        self.motors_pvt_cmd = [MotorPVTCmd()]*self.robot_motors.num_motors

        # update id
        for i in range(self.robot_motors.num_motors):
            self.motors_mit_cmd[i].id = i + 1 # id start from 1

        for i in range(self.robot_motors.num_motors):
            self.motors_pvt_cmd[i].id = i + 1 # id start from 1

        # 设置PVT模式
        self.robot_motors.write_control_mode_all(MODE_PVT) 
        time.sleep(1)

    def get_mode(self):
        return self.control_state.current_control_state

    def switch_mit(self):
        if self.control_state.current_control_state == MODE_MIT:
            return
        else:
            # 从pvt切换到mit，先获取切换前位置，然后设置MIT控制位保持当前位置
            positions = self.getP()
            self.setP(positions)

            # 先复制prev control state
            self.control_state.prev_control_state = self.control_state.current_control_state
            # 然后设置current为MIT
            self.control_state.current_control_state = MODE_MIT

    def switch_pvt(self):
        if self.control_state.current_control_state == MODE_PVT:
            return 
        else:
            # 从mit切换到pvt，先获取切换前位置，然后设置PVT控制位保持当前位置
            positions = self.getP()
            self.setPVT(positions, [0]*len(positions), [0.6]*len(positions))

            # 先复制prev control state
            self.control_state.prev_control_state = self.control_state.current_control_state
            # 然后设置current为MIT
            self.control_state.current_control_state = MODE_PVT

    def load_global_motors_status(self, global_motors_status):
        self.global_motors_status = global_motors_status

    def update_status(self, feedbacks_all):
        # global_motors_status is global and need to be modified
        for f, m in zip(feedbacks_all, self.global_motors_status):
            m.id = f['id']
            m.state = f['state']
            m.position = f['position']
            m.velocity = f['velocity']
            m.torque = f['torque']
            m.temp_mos = f['temp_mos']
            m.temp_rotor = f['temp_rotor']
            m.timestamp = f['timestamp']

    def get_delay_ms(self):
        return [(time.perf_counter() - m.timestamp)*1000 for m in self.global_motors_status]

    def getP(self):
        return [m.position for m in self.global_motors_status]
    
    def getV(self):
        return [m.velocity for m in self.global_motors_status]
    
    def getT(self):
        return [m.torque for m in self.global_motors_status]
    

    ###############################
    ### MIT Mode ##################
    ###############################

    def setP(self, positions):
        for i, p in enumerate(positions):
            self.motors_mit_cmd[i].position = p
            self.motors_mit_cmd[i].velocity = 0
            self.motors_mit_cmd[i].torque = 0
            self.motors_mit_cmd[i].kp = 1
            self.motors_mit_cmd[i].kd = 1
            self.motors_mit_cmd[i].id = i + 1 # id start form 1

    def setT(self, torques):
        for i, t in enumerate(torques):
            self.motors_mit_cmd[i].torque = t
            self.motors_mit_cmd[i].kp = 0
            self.motors_mit_cmd[i].kd = 0
            self.motors_mit_cmd[i].id = i + 1 # id start form 1
        
    def mit_cmd(self):
        ids = [m.id for m in self.motors_mit_cmd]
        poss = [m.position for m in self.motors_mit_cmd]
        vels = [m.velocity for m in self.motors_mit_cmd]
        kps = [m.kp for m in self.motors_mit_cmd]
        kds = [m.kd for m in self.motors_mit_cmd]
        torques = [m.torque for m in self.motors_mit_cmd]

        return self.robot_motors.set_motor_mit_all(ids, poss, vels, kps, kds, torques) 

    ###############################
    ### PVT Mode ##################
    ###############################

    def setPVT(self, positions, velocities, torques):
        """
        set position, velocity, torque (%) for all motors
        """
        for i, p, v, t in zip(range(len(positions)), positions, velocities, torques):
            self.motors_pvt_cmd[i].position = p
            self.motors_pvt_cmd[i].velocity = v
            if t > 1:
                t = 1
            self.motors_pvt_cmd[i].torque = t
            self.motors_pvt_cmd[i].id = i + 1 # id start form 1
        
        
   
    def pvt_cmd(self):
        ids = [m.id for m in self.motors_pvt_cmd]
        poss = [m.position for m in self.motors_pvt_cmd]
        vels = [m.velocity for m in self.motors_pvt_cmd]
        torques = [m.torque for m in self.motors_pvt_cmd]

        return self.robot_motors.set_motor_pvt_all(ids, poss, vels, torques)


    def run(self):
        robot_motors = self.robot_motors

        # 设置高优先级
        set_high_priority()
        
        period = 1.0 / self.freq
        
        # 记录开始时间
        start_time = time.perf_counter()
        iterations = 0
        
        # 失能所有电机
        feedbacks_all = robot_motors.disable_all()
        self.update_status(feedbacks_all)
        time.sleep(1)

        # 使能所有电机
        feedbacks_all = robot_motors.enable_all()
        self.update_status(feedbacks_all)
        
        while True:
            # 记录循环开始时间
            loop_start = time.perf_counter()

            # 判断是否切换模式，如果切换，则control_state内有差异
            if self.control_state.current_control_state != self.control_state.prev_control_state:
                self.robot_motors.write_control_mode_all(self.control_state.current_control_state) # 切换模式
                self.control_state.prev_control_state = self.control_state.current_control_state # 切换之后更新，control_state内无差异
            
            # 执行任务
            if self.control_state.current_control_state == MODE_MIT:
                # print('mit control')
                feedbacks_all = self.mit_cmd()
            elif self.control_state.current_control_state == MODE_PVT:
                # print('pvt control')
                feedbacks_all = self.pvt_cmd()

            self.update_status(feedbacks_all)
            # print('updated..')
            
            # 计算已经使用的时间
            elapsed = time.perf_counter() - loop_start
            
            # 计算需要等待的时间以保持频率
            sleep_time = period - elapsed
            
            # 如果还有时间，则进行精确睡眠
            if sleep_time > 0:
                precise_sleep(sleep_time)
            
            iterations += 1
            
            # 每1000次循环输出实际频率
            if iterations % 1000 == 0:
                actual_freq = iterations / (time.perf_counter() - start_time)
                # print(f"实际频率: {actual_freq:.2f} Hz")
                iterations = 0
                start_time = time.perf_counter()



    