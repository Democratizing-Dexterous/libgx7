import time
import threading
import platform
import os
from dataclasses import dataclass
import numpy as np

from motors import CAN, RobotMotors
from utils.utils import set_high_priority, precise_sleep
from utils.utils import MotorMITCmd, MotorPVTCmd, MotorPVCmd

from .kinematics import Kinematics

MODE_MIT = 1 # MIT模式
MODE_PV = 2 # 位置限速模式
MODE_PVT = 4 # 力位混合模式

DEFAULT_MODE = MODE_PVT # 初始默认模式为PVT

@dataclass
class ControlState:
    current_control_state: int = DEFAULT_MODE
    prev_control_state: int = DEFAULT_MODE


class Robot:
    def __init__(self, freq=300):
        self.kin = Kinematics()
        self.control_state = ControlState()
        can = CAN()
        self.freq = freq
        self.robot_motors = RobotMotors(can)
        self.num_dof = self.robot_motors.num_motors

        init_mit_cmd_params = [[0] * self.num_dof]*5
        self.motors_mit_cmd = MotorMITCmd(*init_mit_cmd_params)

        init_pvt_cmd_params = [[0] * self.num_dof]*3
        self.motors_pvt_cmd = MotorPVTCmd(*init_pvt_cmd_params)

        init_pv_cmd_params = [[0] * self.num_dof]*2
        self.motors_pv_cmd = MotorPVCmd(*init_pv_cmd_params)

        # 设置默认模式
        self.robot_motors.write_control_mode_all(DEFAULT_MODE)
        time.sleep(1)

    def get_mode(self):
        return self.control_state.current_control_state

    def switch_mit(self):
        if self.control_state.current_control_state == MODE_MIT:
            return
        else:
            # 从pvt切换到mit，先获取切换前位置，然后设置MIT控制位保持当前位置
            positions = self.getJP()
            self.setJP(positions)

            # 先复制prev control state
            self.control_state.prev_control_state = (
                self.control_state.current_control_state
            )
            # 然后设置current为MIT
            self.control_state.current_control_state = MODE_MIT

    def switch_pvt(self):
        if self.control_state.current_control_state == MODE_PVT:
            return
        else:
            # 从mit切换到pvt，先获取切换前位置，然后设置PVT控制位保持当前位置
            positions = self.getJP()
            self.setJPVT(positions, [0] * len(positions), [0.6] * len(positions))

            # 先复制prev control state
            self.control_state.prev_control_state = (
                self.control_state.current_control_state
            )
            # 然后设置current为MIT
            self.control_state.current_control_state = MODE_PVT

    def switch_pv(self):
        if self.control_state.current_control_state == MODE_PV:
            return
        else:
            # 从mit切换到pvt，先获取切换前位置，然后设置PVT控制位保持当前位置
            positions = self.getJP()
            self.setJPV(positions, [0.5] * len(positions))

            # 先复制prev control state
            self.control_state.prev_control_state = (
                self.control_state.current_control_state
            )
            # 然后设置current为MIT
            self.control_state.current_control_state = MODE_PV

    def load_global_motors_status(self, global_motors_status):
        self.global_motors_status = global_motors_status

    def update_status(self, feedbacks_all):
        # global_motors_status is global and need to be modified
        self.global_motors_status.ids = [f["id"] for f in feedbacks_all]
        self.global_motors_status.states = [f["state"] for f in feedbacks_all]
        self.global_motors_status.positions = [f["position"] for f in feedbacks_all]
        self.global_motors_status.velocities = [f["velocity"] for f in feedbacks_all]
        self.global_motors_status.torques = [f["torque"] for f in feedbacks_all]
        self.global_motors_status.temp_moss = [f["temp_mos"] for f in feedbacks_all]
        self.global_motors_status.temp_rotors = [f["temp_rotor"] for f in feedbacks_all]
        self.global_motors_status.timestamps = [f["timestamp"] for f in feedbacks_all]

    def get_delay_ms(self):
        return [
            (time.perf_counter() - m) * 1000
            for m in self.global_motors_status.timestamps
        ]


    def fk(self, joint_positions):
        return self.kin.fk(joint_positions)
    
    def ik(self, position, orientation, num_iters=100, threshold=1e-3):
        return self.kin.ik(position, orientation, num_iters=100, threshold=1e-3)

    def jac(self, joint_positions):
        return self.kin.jac(joint_positions)

    def getJP(self):
        return self.global_motors_status.positions

    def getJV(self):
        return self.global_motors_status.velocities

    def getJT(self):
        return self.global_motors_status.torques

    ###############################
    ### MIT Mode ##################
    ###############################

    def setJP(self, positions):
        self.motors_mit_cmd.positions = positions
        self.motors_mit_cmd.velocities = [0] * len(positions)
        self.motors_mit_cmd.torques = [0] * len(positions)
        self.motors_mit_cmd.kps = [10] * len(positions)
        self.motors_mit_cmd.kds = [2] * len(positions)

    def setJT(self, torques):
        self.motors_mit_cmd.torques = torques
        self.motors_mit_cmd.velocities = [0] * len(positions)
        self.motors_mit_cmd.torques = [0] * len(positions)
        self.motors_mit_cmd.kps = [0] * len(positions)
        self.motors_mit_cmd.kds = [0] * len(positions)

    def mit_cmd(self):
        ids = list(np.arange(1, self.num_dof + 1))
        poss = self.motors_mit_cmd.positions
        vels = self.motors_mit_cmd.velocities
        torques = self.motors_mit_cmd.torques
        kps = self.motors_mit_cmd.kps
        kds = self.motors_mit_cmd.kds

        return self.robot_motors.set_motor_mit_all(ids, poss, vels, kps, kds, torques)

    ###############################
    ### PVT Mode ##################
    ###############################

    def setJPVT(self, positions, velocities, torques):
        """
        set position, velocity, torque (%, percentage) for all motors
        """
        self.motors_pvt_cmd.positions = positions
        self.motors_pvt_cmd.velocities = velocities
        self.motors_pvt_cmd.torques = torques


    def pvt_cmd(self):
        ids = list(np.arange(1, self.num_dof + 1))
        poss = self.motors_pvt_cmd.positions
        vels = self.motors_pvt_cmd.velocities
        torques = self.motors_pvt_cmd.torques

        return self.robot_motors.set_motor_pvt_all(ids, poss, vels, torques)

    ###############################
    ### PV Mode ##################
    ###############################

    def setJPV(self, positions, velocities):
        """
        set position, velocity (limit) for all motors
        """

        self.motors_pv_cmd.positions = positions
        self.motors_pv_cmd.velocities = velocities

    def pv_cmd(self):
        ids = list(np.arange(1, self.num_dof + 1))
        poss = self.motors_pv_cmd.positions
        vels = self.motors_pv_cmd.velocities

        return self.robot_motors.set_motor_pv_all(ids, poss, vels)
    

    def enable(self):
        return self.robot_motors.enable_all()
    
    def disable(self):
        return self.robot_motors.disable_all()

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

        while True: # 注意所有can的使用都必须在同一个线程里面
            # 记录循环开始时间
            loop_start = time.perf_counter()

            # 判断是否切换模式，如果切换，则control_state内有差异
            if (
                self.control_state.current_control_state
                != self.control_state.prev_control_state
            ):
                self.robot_motors.write_control_mode_all(
                    self.control_state.current_control_state
                )  # 切换模式
                self.control_state.prev_control_state = (
                    self.control_state.current_control_state
                )  # 切换之后更新，control_state内无差异

            # 执行任务，通过判断模式，发送不同的can指令
            if self.control_state.current_control_state == MODE_MIT:
                # print('mit control')
                feedbacks_all = self.mit_cmd()
            elif self.control_state.current_control_state == MODE_PVT:
                # print('pvt control')
                feedbacks_all = self.pvt_cmd()
            elif self.control_state.current_control_state == MODE_PV:
                feedbacks_all = self.pv_cmd()

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
                print(f"实际频率: {actual_freq:.2f} Hz")
                iterations = 0
                start_time = time.perf_counter()
