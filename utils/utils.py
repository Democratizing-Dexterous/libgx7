import time
import platform
import os
from dataclasses import dataclass
from ctypes import c_double as double



def set_high_priority():
    """设置进程高优先级"""
    system = platform.system()
    try:
        if system == "Windows":
            import psutil
            p = psutil.Process(os.getpid())
            p.nice(psutil.HIGH_PRIORITY_CLASS)
        elif system == "Linux" or system == "Darwin":  # Linux或macOS
            os.nice(-20)  # 最高优先级，需要root权限
    except Exception as e:
        print(f"设置高优先级失败: {e}")

def precise_sleep(duration):
    """高精度睡眠函数"""
    start = time.perf_counter()
    while time.perf_counter() - start < duration:
        pass  # 忙等待以获得更高精度


@dataclass
class MotorStatus:
    id: int = 0
    state: int = 0
    position: float = 0 
    velocity: float = 0
    torque: float = 0
    temp_mos: float = 0
    temp_rotor: float = 0
    # double
    timestamp: double = 0

@dataclass
class MotorMITCmd:
    id: int = 0
    position: float = 0
    velocity: float = 0
    torque: float = 0
    kp: float = 0
    kd: float = 0

@dataclass
class MotorPVTCmd:
    id: int = 0
    position: float = 0
    velocity: float = 0
    torque: float = 0