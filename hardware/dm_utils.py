import struct
import numpy as np
import time

def uint_to_float(x_int, x_min, x_max, bits):
    span = x_max - x_min
    offset = x_min
    return (x_int * span / ((1 << bits) - 1)) + offset

def float_to_uint(value, min_val, max_val, bits):
    span = max_val - min_val
    offset = min_val
    return int(((value - offset) * ((1 << bits) - 1)) / span)


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


def make_pv_frame(pos, vel):
    pos_bytes = list(struct.pack("<f", pos))
    vel_bytes = list(struct.pack("<f", vel))

    return pos_bytes + vel_bytes
