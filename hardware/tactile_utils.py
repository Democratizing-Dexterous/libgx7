import serial
import time
from typing import List, Optional, Dict
import numpy as np


# -------------------------- 设备协议配置（与GEN3传感器匹配）--------------------------
AUTO_PUSH_REG = 0x0017  # 自动回传控制寄存器（1=开启，0=关闭）
AUTO_PUSH_FRAME_HEAD = b"\xaa\x56"  # 自动回传数据帧头
VERSION_REG = 0x0000  # 版本号寄存器地址
VERSION_DATA_LEN = 0x000F  # 版本号数据长度（15字节）
DATA_TYPE_REG = 0x0016  # 数据类型组合寄存器
BAUDRATE = 921600  # 高速通信波特率
TIMEOUT_CMD = 1.0  # 指令响应超时（秒）
TIMEOUT_AUTO_PUSH = 0.05  # 自动回传监听超时（秒）

# 帧结构常量
REQ_HEAD = b"\x55\xaa"  # 请求帧头（主机→传感器）
RESP_HEAD_GENERAL = b"\xaa\x55"  # 普通响应帧头（传感器→主机）
RESP_HEAD_AUTO_PUSH = b"\xaa\x56"  # 自动回传相关响应帧头
RESERVED = b"\x00"  # 预留字段
FUNC_READ = 0x03  # 读寄存器功能码
FUNC_WRITE = 0x10  # 写寄存器功能码

# 指令示例（用于校验）
CMD_EXAMPLE_AUTO_PUSH = "55AA00101700010001D8"  # 开启自动推送示例指令
CMD_EXAMPLE_VERSION = "55AA000300000F00EF"  # 读版本号示例指令


def send_hex_cmd(ser: serial.Serial, hex_cmd: str) -> bool:
    """发送16进制指令"""
    try:
        cmd_bytes = bytes.fromhex(hex_cmd)
        ser.flushOutput()
        sent_len = ser.write(cmd_bytes)
        if sent_len != len(cmd_bytes):
            return False
        return True
    except (serial.SerialException, ValueError, Exception) as e:
        return False


def read_serial_data(
    ser: serial.Serial,
    timeout: float = TIMEOUT_CMD,
    expected_head: Optional[bytes] = None,
) -> Optional[bytes]:
    """读取串口数据（支持指定预期帧头）"""
    try:
        start_time = time.time()
        recv_data = b""
        while time.time() - start_time < timeout:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                recv_data += chunk

                # 如果指定了预期帧头，检查是否已收到
                if expected_head and expected_head in recv_data:
                    # 从帧头开始截取数据
                    head_pos = recv_data.find(expected_head)
                    recv_data = recv_data[head_pos:]
                    break  # 找到预期帧头后退出等待

                start_time = time.time()  # 重置超时计时器
                time.sleep(0.005)  # 等待可能的后续数据
            time.sleep(0.001)

        if recv_data:
            return recv_data
        return None
    except Exception as e:
        return None


def calc_lrc(data: bytes) -> int:
    """计算LRC校验（累加→取反→加1→低8位）"""
    try:
        lrc_sum = 0
        for byte in data:
            lrc_sum = (lrc_sum + byte) & 0xFF  # 8位累加防溢出
        lrc = ((~lrc_sum) + 1) & 0xFF  # 补码计算
        return lrc
    except Exception as e:
        return 0


def build_request_frame(
    func_code: int, reg_addr: int, data_len: int, write_data: bytes = b""
) -> Optional[str]:
    """构建请求帧（完全匹配协议：Head+预留+功能码+寄存器地址+数据长度+数据+LRC）"""
    try:
        # 寄存器地址和数据长度均采用小端模式（匹配协议要求）
        reg_addr_bytes = reg_addr.to_bytes(2, byteorder="little")
        data_len_bytes = data_len.to_bytes(2, byteorder="little")

        # 组装帧主体（不含LRC）
        frame_without_lrc = (
            REQ_HEAD
            + RESERVED
            + func_code.to_bytes(1, "big")
            + reg_addr_bytes
            + data_len_bytes
            + write_data
        )

        # 计算LRC并补全帧
        lrc = calc_lrc(frame_without_lrc).to_bytes(1, "big")
        full_frame = frame_without_lrc + lrc
        frame_hex = full_frame.hex().upper()

        return frame_hex
    except Exception as e:
        return None


def parse_auto_response(response: bytes) -> Optional[Dict]:
    """解析自动回传相关响应（包括关闭指令响应），帧头为AA 56"""
    try:
        # 基础校验：最小帧长度
        if len(response) < 7:
            return None

        # 帧头校验（AA 56）
        if response[:2] != RESP_HEAD_AUTO_PUSH:
            return None

        # 提取基础字段
        parsed = {
            "head": response[:2].hex(),
            "reserved": response[2],
            "valid_frame_len": int.from_bytes(
                response[3:5], "little"
            ),  # 有效帧长度：小端
            "error_code": response[5],
            "valid_data": b"",
            "valid_data_len": 0,
            "lrc_valid": False,
            "lrc_calc": 0,
            "lrc_recv": response[-1] if len(response) >= 7 else 0,
        }

        # 计算有效数据长度
        parsed["valid_data_len"] = parsed["valid_frame_len"] - 1

        # 提取有效数据
        if parsed["valid_data_len"] > 0:
            data_end_pos = 6 + parsed["valid_data_len"]
            if data_end_pos <= len(response) - 1:  # 预留LRC位置
                parsed["valid_data"] = response[6:data_end_pos]
            else:
                parsed["valid_data"] = response[6:-1]  # 截取到LRC前

        return parsed
    except Exception as e:
        return None


def parse_response(response: bytes) -> Optional[Dict]:
    """解析普通设备响应帧（Head+预留+功能码+地址+数据长度+数据+LRC）"""
    try:
        # 基础校验：最小帧长度
        if len(response) < 8:  # 无数据时最小8字节
            return None

        # 帧头校验（普通响应为AA 55）
        if response[:2] != RESP_HEAD_GENERAL:
            return None

        # 提取字段（所有多字节字段采用小端解析）
        parsed = {
            "is_error": False,
            "reserved": response[2],
            "func_code": response[3],
            "reg_addr": int.from_bytes(response[4:6], "little"),
            "data_len": int.from_bytes(response[6:8], "little"),
            "actual_data_len": len(response) - 9 if len(response) > 8 else 0,
            "data": b"",
            "lrc_valid": False,
            "lrc_calc": 0,
            "lrc_recv": response[-1] if len(response) >= 9 else 0,
        }

        # 错误响应处理（功能码最高位为1表示错误）
        if (parsed["func_code"] & 0x80) != 0:
            parsed["is_error"] = True
            parsed["error_code"] = parsed["func_code"] & 0x7F
            return parsed

        # 功能码校验
        if parsed["func_code"] not in [FUNC_READ, FUNC_WRITE]:
            return None

        # 提取有效数据
        if parsed["data_len"] > 0 and len(response) >= 8 + parsed["data_len"] + 1:
            parsed["data"] = response[8 : 8 + parsed["data_len"]]
        elif parsed["data_len"] > 0:
            parsed["data"] = response[8:-1] if len(response) > 8 else b""

        # LRC校验
        if len(response) >= 9:
            parsed["lrc_calc"] = calc_lrc(response[:-1])
            parsed["lrc_valid"] = parsed["lrc_calc"] == parsed["lrc_recv"]

        return parsed
    except Exception as e:
        return None


def read_register(ser: serial.Serial, reg_addr: int, read_len: int) -> Optional[bytes]:
    """读取寄存器数据（功能码0x03）"""
    if not (1 <= read_len <= 512):
        return None

    read_frame = build_request_frame(FUNC_READ, reg_addr, read_len)
    if not read_frame:
        return None

    if not send_hex_cmd(ser, read_frame):
        return None

    time.sleep(0.2)  # 预留响应时间
    response = read_serial_data(ser, TIMEOUT_CMD, RESP_HEAD_GENERAL)
    if not response:
        return None

    parsed = parse_response(response)
    if not parsed or parsed["is_error"] or parsed["func_code"] != FUNC_READ:
        return None

    return parsed["data"]


def write_register(
    ser: serial.Serial, reg_addr: int, write_data: bytes, is_auto_push: bool = False
) -> bool:
    """写入寄存器数据（功能码0x10），is_auto_push标识是否为自动回传相关操作"""
    write_len = len(write_data)
    if not (1 <= write_len <= 10):
        return False

    write_frame = build_request_frame(FUNC_WRITE, reg_addr, write_len, write_data)
    if not write_frame:
        return False

    if not send_hex_cmd(ser, write_frame):
        return False

    time.sleep(0.2)  # 预留响应时间

    # 根据是否为自动回传相关操作选择不同的帧头
    expected_head = RESP_HEAD_AUTO_PUSH if is_auto_push else RESP_HEAD_GENERAL
    response = read_serial_data(ser, TIMEOUT_CMD, expected_head)
    if not response:
        return False

    # 解析响应
    if is_auto_push:
        parsed = parse_auto_response(response)
        # 自动回传响应通过error_code判断是否成功（0表示成功）
        if not parsed or parsed["error_code"] != 0:
            return False
    else:
        parsed = parse_response(response)
        if not parsed or parsed["is_error"] or parsed["func_code"] != FUNC_WRITE:
            return False

        # 验证写入状态（返回数据为0表示成功）
        if len(parsed["data"]) > 0:
            write_status = int.from_bytes(parsed["data"], "little")
            if write_status != 0:
                return False

    return True


def disable_auto_push(ser: serial.Serial) -> bool:
    """关闭自动回传功能（写入0x0017寄存器为0x00）"""
    try:
        # 直接构建并发送关闭指令，不等待响应
        disable_cmd = build_request_frame(FUNC_WRITE, AUTO_PUSH_REG, 1, b"\x00")
        if not disable_cmd:
            return False

        # 发送指令但不验证响应
        if send_hex_cmd(ser, disable_cmd):
            # 短暂延迟确保指令被接收
            time.sleep(0.1)
            return True
        return False
    except Exception as e:
        return False


def enable_auto_push(ser: serial.Serial) -> bool:
    """开启自动回传功能（写入0x0017寄存器为0x01）"""
    return write_register(ser, AUTO_PUSH_REG, b"\x01", is_auto_push=True)


def parse_auto_push_data(data: bytes, expected_length: int = 0) -> Optional[Dict]:
    """解析自动回传数据帧（AA56头+预留+有效帧长度+总错误码+有效数据+LRC）"""
    try:
        # 基础校验：最小帧长度（Head2+预留1+有效帧长度2+总错误码1+LRC1=7字节）
        if len(data) < 7:
            return None

        # 帧头校验
        if data[:2] != AUTO_PUSH_FRAME_HEAD:
            return None

        # 提取基础字段
        parsed = {
            "head": data[:2].hex(),
            "reserved": data[2],
            "valid_frame_len": int.from_bytes(data[3:5], "little"),  # 有效帧长度：小端
            "error_code": data[5],
            "valid_data": b"",
            "valid_data_len": 0,
            "expected_data_len": expected_length,
            "length_match": False,
            "lrc_valid": False,
            "lrc_calc": 0,
            "lrc_recv": data[-1] if len(data) >= 7 else 0,
        }

        # 计算有效数据长度（有效帧长度 = 有效数据长度 + 1）
        parsed["valid_data_len"] = parsed["valid_frame_len"] - 1

        # 提取有效数据
        if parsed["valid_data_len"] > 0:
            data_end_pos = 6 + parsed["valid_data_len"]
            if data_end_pos <= len(data) - 1:  # 预留LRC位置
                parsed["valid_data"] = data[6:data_end_pos]
            else:
                parsed["valid_data"] = data[6:-1]  # 截取到LRC前

        # 校验数据长度是否符合预期配置
        if expected_length > 0:
            parsed["length_match"] = parsed["valid_data_len"] == expected_length

        # LRC校验
        if len(data) >= 6 + parsed["valid_data_len"] + 1:
            parsed["lrc_calc"] = calc_lrc(data[:-1])
            parsed["lrc_valid"] = parsed["lrc_calc"] == parsed["lrc_recv"]

        return parsed
    except Exception as e:
        return None


def get_device_version(ser: serial.Serial) -> Optional[str]:
    """获取设备版本号"""
    version_data = read_register(ser, VERSION_REG, VERSION_DATA_LEN)
    if version_data:
        try:
            ascii_version = version_data.decode("ascii", errors="ignore").strip()
            return f"ASCII: {ascii_version} | 16进制: {version_data.hex().upper()}"
        except:
            return f"16进制: {version_data.hex().upper()}"
    return None


num_points = 31
force_locations = np.array(
    [
        [-5.36825237, -0.81971299, 1.93772424],
        [-5.55172331, 2.69670671, 1.93782972],
        [-5.66768606, 6.21590495, 1.93776648],
        [-5.6040785, 9.73615552, 1.93783705],
        [-5.13956525, 13.22048109, 1.93779489],
        [-3.71510007, 2.04097458, 5.18509353],
        [-4.00750987, 5.51729569, 5.48620161],
        [-4.07078449, 9.01418632, 5.60938089],
        [-3.41310733, -1.42401701, 4.7780852],
        [-3.60030056, 12.46300302, 5.35300798],
        [-3.30080129, 16.14599966, 1.93782074],
        [-2.73057326, 14.79946941, 4.52881831],
        [0.00002345, -1.55880623, 5.4119337],
        [-0.02029023, 1.86416301, 6.09365659],
        [-0.00029363, 5.2935317, 6.74328381],
        [-0.00004412, 8.76418618, 7.07467407],
        [-0.00000298, 12.19682661, 6.5354801],
        [-0.00000003, 15.19909143, 4.80192037],
        [-0.00000148, 17.14451483, 1.93778105],
        [2.73057326, 14.79946941, 4.52881831],
        [3.41310733, -1.42401701, 4.7780852],
        [3.71510007, 2.04097458, 5.18509353],
        [3.60030056, 12.46300302, 5.35300798],
        [3.30080129, 16.14599966, 1.93782074],
        [4.00750987, 5.51729569, 5.48620161],
        [4.07078449, 9.01418632, 5.60938089],
        [5.36825237, -0.81971299, 1.93772424],
        [5.55172331, 2.69670671, 1.93782972],
        [5.6040785, 9.73615552, 1.93783705],
        [5.13956525, 13.22048109, 1.93779489],
        [5.66768606, 6.21590495, 1.93776648],
    ]
)
