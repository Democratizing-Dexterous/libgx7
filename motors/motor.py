from ctypes import *
import os
import time
import numpy as np
import struct


abs_path = os.path.abspath(__file__)


def uint_to_float(x_int, x_min, x_max, bits):
    span = x_max - x_min
    offset = x_min
    return (x_int * span / ((1 << bits) - 1)) + offset


def float_to_uint(value, min_val, max_val, bits):
    span = max_val - min_val
    offset = min_val
    return int(((value - offset) * ((1 << bits) - 1)) / span)


VCI_USBCAN2 = 4
STATUS_OK = 1


class VCI_INIT_CONFIG(Structure):
    _fields_ = [
        ("AccCode", c_uint),
        ("AccMask", c_uint),
        ("Reserved", c_uint),
        ("Filter", c_ubyte),
        ("Timing0", c_ubyte),
        ("Timing1", c_ubyte),
        ("Mode", c_ubyte),
    ]


class VCI_CAN_OBJ(Structure):
    _fields_ = [
        ("ID", c_uint),
        ("TimeStamp", c_uint),
        ("TimeFlag", c_ubyte),
        ("SendType", c_ubyte),
        ("RemoteFlag", c_ubyte),
        ("ExternFlag", c_ubyte),
        ("DataLen", c_ubyte),
        ("Data", c_ubyte * 8),
        ("Reserved", c_ubyte * 3),
    ]


class VCI_CAN_OBJ_ARRAY(Structure):
    _fields_ = [("SIZE", c_uint16), ("STRUCT_ARRAY", POINTER(VCI_CAN_OBJ))]

    def __init__(self, num_of_structs):
        self.STRUCT_ARRAY = cast((VCI_CAN_OBJ * num_of_structs)(), POINTER(VCI_CAN_OBJ))
        self.SIZE = num_of_structs
        self.ADDR = self.STRUCT_ARRAY[0]


class CAN:
    def __init__(self):
        self.lib_can_path = os.path.join(
            os.path.dirname(abs_path), "libs/ControlCAN.dll"
        )
        self.canDLL = windll.LoadLibrary(self.lib_can_path)

    def initCan1(self):
        vci_initconfig = VCI_INIT_CONFIG(
            0x80000008, 0xFFFFFFFF, 0, 0, 0x00, 0x14, 0
        )  # 1M bps

        ret = self.canDLL.VCI_OpenDevice(VCI_USBCAN2, 0, 0)
        if ret == STATUS_OK:
            print("VCI_OpenDevice successed!")
        else:
            print("VCI_OpenDevice error!")
            os._exit(0)

        ret = self.canDLL.VCI_InitCAN(VCI_USBCAN2, 0, 0, byref(vci_initconfig))
        if ret == STATUS_OK:
            print("VCI_InitCAN1 successed!")
        else:
            print("VCI_InitCAN1 error!")
            os._exit(0)

        self.canDLL.VCI_ClearBuffer(VCI_USBCAN2, 0, 0)
        ret = self.canDLL.VCI_StartCAN(VCI_USBCAN2, 0, 0)

        if ret == STATUS_OK:
            print("Start OK!")
            time.sleep(0.5)
        else:
            print("Start Failed!")
            os._exit(0)

    def send_frame(self, id, hex_list, timeout=0):
        ubyte_array = c_ubyte * 8
        a = ubyte_array(*hex_list)
        ubyte_3array = c_ubyte * 3
        b = ubyte_3array(0, 0, 0)
        vci_can_obj = VCI_CAN_OBJ(id, 0, 0, 1, 0, 0, 8, a, b)
        ret = self.canDLL.VCI_Transmit(VCI_USBCAN2, 0, 0, byref(vci_can_obj), 1)
        rx_vci_can_obj = VCI_CAN_OBJ_ARRAY(2500)
        if ret:
            while True:
                ret = self.canDLL.VCI_Receive(
                    VCI_USBCAN2, 0, 0, byref(rx_vci_can_obj.ADDR), 2500, 0
                )
                if ret:
                    feedback_frame = rx_vci_can_obj.STRUCT_ARRAY[0].Data[:8]
                    return feedback_frame

    def read_control_mode(self, id, max_retry=20):
        control_mode = 0
        for i in range(max_retry):
            feedback_frame = self.send_frame(0x7FF, [id, 0x00, 0x33, 0x0A] + [0x00] * 4)
            if feedback_frame[0] == id:  # match
                control_mode = int.from_bytes(feedback_frame[4:8], byteorder="little")
                break
            time.sleep(0.1)
        return control_mode

    def write_control_mode(self, id, mode, max_retry=20):
        control_mode = 0
        mode = int.to_bytes(mode, 4, byteorder="little")
        for i in range(max_retry):
            feedback_frame = self.send_frame(0x7FF, [id, 0x00, 0x55, 0x0A] + list(mode))
            if feedback_frame[0] == id:  # match
                control_mode = int.from_bytes(feedback_frame[4:8], byteorder="little")
                break
            time.sleep(0.1)
        return control_mode


def extract_feedback_frame(motor_types, motor_name, rx_data):
    # https://gl1po2nscb.feishu.cn/wiki/VYrlwHI7liHzXIkx0s0cUOVdnzb
    motor_id = (rx_data[0]) & 0x0F
    motor_state = (rx_data[0]) >> 4

    motor_p = (rx_data[1] << 8) | rx_data[2]
    motor_v = (rx_data[3] << 4) | (rx_data[4] >> 4)
    motor_t = ((rx_data[4] & 0xF) << 8) | rx_data[5]

    # read from motor_types
    PMAX = motor_types[motor_name]["pmax"]
    VMAX = motor_types[motor_name]["vmax"]
    TMAX = motor_types[motor_name]["tmax"]

    motor_pos = uint_to_float(motor_p, -PMAX, PMAX, 16)
    motor_vel = uint_to_float(motor_v, -VMAX, VMAX, 12)
    motor_t = uint_to_float(motor_t, -TMAX, TMAX, 12)

    motor_Tmos = float(rx_data[6])
    motor_Trotor = float(rx_data[7])

    timestamp = time.perf_counter()

    return {
        "id": motor_id,
        "state": motor_state,
        "position": motor_pos,
        "velocity": motor_vel,
        "torque": motor_t,
        "temp_mos": motor_Tmos,
        "temp_rotor": motor_Trotor,
        "timestamp": timestamp,
    }


def make_mit_frame(motor_types, motor_name, pos, vel, kp, kd, tor):
    # https://gl1po2nscb.feishu.cn/wiki/VYrlwHI7liHzXIkx0s0cUOVdnzb
    data = [0] * 8
    # read from motor_types
    PMAX = motor_types[motor_name]["pmax"]
    VMAX = motor_types[motor_name]["vmax"]
    TMAX = motor_types[motor_name]["tmax"]

    # clip in range
    pos = np.clip(pos, -PMAX, PMAX)
    vel = np.clip(vel, -VMAX, VMAX)
    tor = np.clip(tor, -TMAX, TMAX)

    pos_tmp = float_to_uint(pos, -PMAX, PMAX, 16)
    vel_tmp = float_to_uint(vel, -VMAX, VMAX, 12)
    tor_tmp = float_to_uint(tor, -TMAX, TMAX, 12)

    # for example
    KP_MIN = 0
    KP_MAX = 500
    KD_MIN = 0
    KD_MAX = 5

    # clip in range
    kp = np.clip(kp, KP_MIN, KP_MAX)
    kd = np.clip(kd, KD_MIN, KD_MAX)

    kp_tmp = float_to_uint(
        kp, KP_MIN, KP_MAX, 12
    )  # 假设 KP_MIN 和 KP_MAX 是预定义的常量
    kd_tmp = float_to_uint(
        kd, KD_MIN, KD_MAX, 12
    )  # 假设 KD_MIN 和 KD_MAX 是预定义的常量

    data[0] = (pos_tmp >> 8) & 0xFF
    data[1] = pos_tmp & 0xFF
    data[2] = (vel_tmp >> 4) & 0xFF
    data[3] = ((vel_tmp & 0xF) << 4) | ((kp_tmp >> 8) & 0xF)
    data[4] = kp_tmp & 0xFF
    data[5] = (kd_tmp >> 4) & 0xFF
    data[6] = ((kd_tmp & 0xF) << 4) | ((tor_tmp >> 8) & 0xF)
    data[7] = tor_tmp & 0xFF

    return data


def make_pvt_frame(pos, vel, torque):

    vel = int(vel * 100)

    torque = int(torque * 10000)

    vel_bytes = list(int.to_bytes(vel, 2, byteorder="little"))
    torque_bytes = list(int.to_bytes(torque, 2, byteorder="little"))

    # pos is float
    pos_bytes = list(struct.pack("<f", pos))

    return pos_bytes + vel_bytes + torque_bytes
