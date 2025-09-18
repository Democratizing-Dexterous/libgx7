import enum
import time
import threading
import platform
import os
from dataclasses import dataclass
import numpy as np
import threading

from hardware import VCICAN, RobotMotors
from utils.utils import precise_sleep
from utils.utils import MotorStatus

from .kinematics import Kinematics

MODE_MIT = 1 # MIT模式
MODE_PV = 2 # 位置限速模式
MODE_PVT = 4 # 力位混合模式

@dataclass
class ControlState:
    current_control_state: int
    prev_control_state: int

class Robot:
    def __init__(self, can: VCICAN, can_channel=0, freq=100, control_mode='pvt', soft_limit=True):
        self.kin = Kinematics()
        self.freq = freq
        
        self.can = can
        self.can_channel = can_channel
        
        self.robot_motors = RobotMotors(self.can, self.can_channel)
        self.num_dof = self.robot_motors.num_motors
        self.motor_limits = self.robot_motors.motor_limits
        
        self.init_motors_status_params = [[0] * self.num_dof] * 8
        self.global_motors_status = MotorStatus(*self.init_motors_status_params)
      
        ##############
        # PVT control
        ##############
        self.pvt_control_positions = [0] * self.num_dof
        self.pvt_control_velocities = [0] * self.num_dof
        self.pvt_control_torques = [0] * self.num_dof
        
        ##############
        # PV control
        ##############
        self.pv_control_positions = [0] * self.num_dof
        self.pv_control_velocities = [0] * self.num_dof
        
        
        ##############
        # MIT control
        ##############
        self.mit_control_positions = [0] * self.num_dof
        self.mit_control_velocities = [0] * self.num_dof
        self.mit_control_torques = [0] * self.num_dof
        self.mit_control_kps = [0] * self.num_dof
        self.mit_control_kds = [0] * self.num_dof
        
        
        # Flag to control the thread loop
        self.running = False
        self.threading_robot_run = None
        
        self.setup_done = False

        mode = 0
        if control_mode == 'pvt':
            mode = MODE_PVT
        if control_mode == 'pv':
            mode = MODE_PV
        if control_mode == 'mit':
            mode = MODE_MIT
        
        self.soft_limit = soft_limit
        
        # 设置模式
        self.robot_motors.write_control_mode_all(mode)
        self.control_state = ControlState(mode, mode)
        print(f'Init Mode: {control_mode}')
        
    def run(self):
        """
        Start the robot control thread if not already running.
        """
        if self.threading_robot_run is not None and self.threading_robot_run.is_alive():
            print("Robot thread is already running.")
            return
            
        self.running = True
        self.threading_robot_run = threading.Thread(target=self.loop)
        # Make thread daemon so it doesn't block program exit
        self.threading_robot_run.daemon = True
        self.threading_robot_run.start()
        print("Robot thread started.")
        time.sleep(1)
        
    def stop(self):
        """
        Stop the robot control thread.
        """
        self.running = False
        if self.threading_robot_run and self.threading_robot_run.is_alive():
            print("Stopping robot thread...")
            # Wait for the thread to finish
            self.threading_robot_run.join(timeout=2.0)
            print("Robot thread stopped.")
    
    def get_mode(self):
        return self.control_state.current_control_state

    def switch_mit(self):
        if self.control_state.current_control_state == MODE_MIT:
            return
        else:
            # 从pvt切换到mit，先获取切换前位置，然后设置MIT控制位保持当前位置
            positions = self.getJP()
            self.setJPs(positions)

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
            self.setJPVTs(positions, [0] * len(positions), [0.6] * len(positions))

            # 先复制prev control state
            self.control_state.prev_control_state = (
                self.control_state.current_control_state
            )
            # 然后设置current为PVT
            self.control_state.current_control_state = MODE_PVT

    def switch_pv(self):
        if self.control_state.current_control_state == MODE_PV:
            return
        else:
            # 从mit切换到pv，先获取切换前位置，然后设置PV控制位保持当前位置
            positions = self.getJP()
            self.setJPVs(positions, [0.5] * len(positions))

            # 先复制prev control state
            self.control_state.prev_control_state = (
                self.control_state.current_control_state
            )
            # 然后设置current为PV
            self.control_state.current_control_state = MODE_PV
        
    def get_status(self):
        return self.global_motors_status

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

    def get_delay(self):
        """unit: ms"""
        return [
            (time.perf_counter() - m) * 1000
            for m in self.global_motors_status.timestamps
        ]


    def fk(self, joint_positions):
        return self.kin.fk(joint_positions)
    
    def ik(self, position, orientation=[0, 0, 0], num_iters=200, threshold=1e-6):
        return self.kin.ik(position, orientation, num_iters=num_iters, threshold=threshold)

    def jac(self, joint_positions):
        return self.kin.jac(joint_positions)


    def check_joint_limits(self):
        """
        检查关节位置和速度是否超出限制
        """
        jps = self.global_motors_status.positions
        for i, jp in enumerate(jps):
            if jp < self.motor_limits[i][0]*np.pi/180 or jp > self.motor_limits[i][1]*np.pi/180:
                info = f'第{i+1}关节超出位置限制！请拖动到合理范围然后重新启动程序'
                print(i+1, jp, self.motor_limits[i][0]*np.pi/180, self.motor_limits[i][1]*np.pi/180)
                return False, info
        return True, ''

    def check_error(self):
        # 8——超压； 9——欠压；A——过电流； B——MOS 过温；C——电机线圈过温； D——通讯丢失；E——过载；
        for i, jp in enumerate(self.global_motors_status.states):
            if jp == 0x8:
                info = f'第{i+1}关节超压！'
                return False, info
            elif jp == 0x9:
                info = f'第{i+1}关节欠压！'
                return False, info
            elif jp == 0xA:
                info = f'第{i+1}关节过电流！'
                return False, info
            elif jp == 0xB:
                info = f'第{i+1}关节MOS过温！'
                return False, info
            elif jp == 0xC:
                info = f'第{i+1}关节电机线圈过温！'
                return False, info
            elif jp == 0xD:
                info = f'第{i+1}关节通信丢失！'
                return False, info
            elif jp == 0xE:
                info = f'第{i+1}关节过载！'
                return False, info

        return True, ''

    ###############################
    ### Joint Info ################
    ###############################

    def getJP(self):
        return self.global_motors_status.positions

    def getJV(self):
        return self.global_motors_status.velocities

    def getJT(self):
        return self.global_motors_status.torques

    def getRotorTemp(self):
        return self.global_motors_status.temp_rotors

    def getMossTemp(self):
        return self.global_motors_status.temp_moss
        
    def get_status_summary(self):
        """
        Returns a summary of the robot's current status in a readable format.
        Useful for interactive mode.
        """
        positions = self.getJP()
        velocities = self.getJV()
        torques = self.getJT()
        rotor_temps = self.getRotorTemp()
        moss_temps = self.getMossTemp()
        
        summary = "Robot Status Summary:\n"
        summary += "--------------------\n"
        summary += f"Control Mode: {self.get_mode()}\n"
        summary += "Joint Positions (rad): " + str(positions) + "\n"
        summary += "Joint Velocities (rad/s): " + str(velocities) + "\n"
        summary += "Joint Torques (Nm): " + str(torques) + "\n"
        summary += "Rotor Temperatures: " + str(rotor_temps) + "\n"
        summary += "MOSFET Temperatures: " + str(moss_temps) + "\n"
        
        # Add end effector position if joint positions are available
        if positions and len(positions) >= 6:
            try:
                ee_pos = self.fk(positions)
                summary += "End Effector Position: " + str(ee_pos) + "\n"
            except:
                pass
                
        return summary


    ###############################
    ### MIT Mode ##################
    ###############################

    def setJP(self, id, position):
        self.mit_control_positions[id-1] = position
        self.mit_control_velocities[id-1] = 0
        self.mit_control_kps[id-1] = 10
        self.mit_control_kds[id-1] = 2

    def setJPs(self, positions):
        """
        Set position for all motors in MIT mode
        """
        self.mit_control_positions = positions
        self.mit_control_velocities = [0] * len(positions)
        self.mit_control_torques = [0] * len(positions)
        self.mit_control_kps = [30] * len(positions)
        self.mit_control_kds = [2] * len(positions)

    def setJTs(self, torques):
        """
        Set torque for all motors in MIT mode
        """
        self.mit_control_torques = torques
        self.mit_control_velocities = [0] * len(torques)
        self.mit_control_kps = [0] * len(torques)
        self.mit_control_kds = [0] * len(torques)

    def mit_cmd(self):
        ids = list(np.arange(1, self.num_dof + 1))
        poss = self.mit_control_positions
        vels = self.mit_control_velocities
        torques = self.mit_control_torques
        kps = self.mit_control_kps
        kds = self.mit_control_kds

        return self.robot_motors.set_motor_mit_all(ids, poss, vels, kps, kds, torques)

    ###############################
    ### PVT Mode ##################
    ###############################

    def setJPVTs(self, positions, velocities, torques):
        """
        set position (control goal), velocity (limit), torque (%, percentage, limit) for all motors
        """
        self.pvt_control_positions = positions
        self.pvt_control_velocities = velocities
        self.pvt_control_torques = torques
        
    def setJPVT(self, id, position, velocity, torque):
        self.pvt_control_positions[id-1] = position
        self.pvt_control_velocities[id-1] = velocity
        self.pvt_control_torques[id-1] = torque

    def pvt_cmd(self):
        ids = list(np.arange(1, self.num_dof + 1))
        poss = self.pvt_control_positions
        vels = self.pvt_control_velocities
        torques = self.pvt_control_torques

        return self.robot_motors.set_motor_pvt_all(ids, poss, vels, torques)

    ###############################
    ### PV Mode ###################
    ###############################
    
    def setJPVs(self, positions, velocities):
        """
        set position (control goal), velocity (limit) for all motors
        """
        self.pv_control_positions = positions
        self.pv_control_velocities = velocities
        
    def setJPV(self, id, position, velocity):
        """
        set position (control goal), velocity (limit) for all motors
        """
        self.pv_control_positions[id-1] = position
        self.pv_control_velocities[id-1] = velocity
        
    def pv_cmd(self):
        ids = list(np.arange(1, self.num_dof + 1))
        poss = self.pv_control_positions
        vels = self.pv_control_velocities

        return self.robot_motors.set_motor_pv_all(ids, poss, vels)
    

    def enable(self):
        return self.robot_motors.enable_all()
    
    def disable(self):
        return self.robot_motors.disable_all()
    
    def setup(self):
        if not self.setup_done:
            robot_motors = self.robot_motors
            # 失能所有电机
            feedbacks_all = robot_motors.disable_all()
            self.update_status(feedbacks_all)
            # 清除错误
            robot_motors.clear_error_all()
            time.sleep(1)

            # 使能所有电机
            feedbacks_all = robot_motors.enable_all()
            self.update_status(feedbacks_all)
            self.setup_done = True


    def loop(self):
        """
        The main thread function that runs in the background.
        This function contains the main control loop for the robot.
        """
        robot_motors = self.robot_motors
        period = 1.0 / self.freq

        # 记录开始时间
        start_time = time.perf_counter()
        iterations = 0

        print('Starting robot thread...')
        
        if not self.setup_done:
            print('Please setup first!')
            return 
        try:
            # Main control loop
            while self.running:  # Use flag to control the loop
                # 记录循环开始时间
                loop_start = time.perf_counter()
                # print(loop_start)

                # if self.soft_limit:
                #     # 判断关节限位
                #     joints_valid, info = self.check_joint_limits()
                #     if not joints_valid:
                #         print(info)
                #         robot_motors.disable_all()
                #         self.running = False
                #         break

                # 检查错误
                error_free, info = self.check_error()
                if not error_free:
                    print(info)
                    robot_motors.disable_all()
                    self.running = False
                    break

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
                try:
                    if self.control_state.current_control_state == MODE_MIT:
                        feedbacks_all = self.mit_cmd()
                    elif self.control_state.current_control_state == MODE_PVT:
                        feedbacks_all = self.pvt_cmd()
                    elif self.control_state.current_control_state == MODE_PV:
                        feedbacks_all = self.pv_cmd()

                    self.update_status(feedbacks_all)
                except Exception as e:
                    print(f"Error in control loop: {e}")
                    continue  # Continue the loop even if there's an error

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
        
        except Exception as e:
            print(f"Error in robot thread: {e}")
        finally:
            # Make sure to disable motors when the thread exits
            try:
                robot_motors.disable_all()
                print("Robot motors disabled.")
            except:
                print("Failed to disable robot motors.")
            
            self.running = False
            print("Robot thread stopped.")
