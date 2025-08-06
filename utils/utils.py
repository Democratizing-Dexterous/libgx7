import time
import platform
import os
from dataclasses import dataclass
from ctypes import c_double as double



def precise_sleep(duration):
    """高精度睡眠函数"""
    start = time.perf_counter()
    while time.perf_counter() - start < duration:
        pass  # 忙等待以获得更高精度


@dataclass
class MotorStatus:
    """电机状态数据类"""

    ids: list
    states: list
    positions: list
    velocities: list
    torques: list
    temp_moss: list
    temp_rotors: list
    # double
    timestamps: list


@dataclass
class MotorMITCmd:
    """
    MIT模式下的电机指令数据类
    """

    positions: list
    velocities: list
    torques: list
    kps: list
    kds: list


@dataclass
class MotorPVTCmd:
    """
    PVT模式下的电机指令数据类
    """

    positions: list
    velocities: list
    torques: list

@dataclass
class MotorPVCmd:
    """
    PV模式下的电机指令数据类
    """

    positions: list
    velocities: list
