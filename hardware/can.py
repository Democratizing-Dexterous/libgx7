from ctypes import *
import os
import time
import numpy as np
import serial
import struct

abs_path = os.path.abspath(__file__)


# for VCI usb2can 
VCI_USBCAN2 = 4
STATUS_OK = 1

class VCI_BOARD_INFO(Structure):
    _fields_ = [
        ("hw_Version", c_uint16),
        ("fw_Version", c_uint16),
        ("dr_Version", c_uint16),
        ("in_Version", c_uint16),
        ("irq_Num", c_uint16),
        ("can_Num", c_uint8),
        ("str_Serial_Num", c_char * 20),
        ("str_hw_Type", c_char * 40),
        ("Reserved", c_uint16 * 4),
    ]

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
        
class VCI_BOARD_INFO_ARRAY(Structure):
    _fields_ = [("SIZE", c_uint16), ("STRUCT_ARRAY", POINTER(VCI_BOARD_INFO))]

    def __init__(self, num_of_structs):
        self.STRUCT_ARRAY = cast((VCI_BOARD_INFO * num_of_structs)(), POINTER(VCI_BOARD_INFO))
        self.SIZE = num_of_structs
        self.ADDR = self.STRUCT_ARRAY[0]


class VCICAN:
    def __init__(self, device_id=0, device_serial_number=None):
        # linux so
        self.lib_can_path = os.path.join(
            os.path.dirname(abs_path), "libs/libcontrolcan.so"
        )
        self.usbcan = cdll.LoadLibrary(self.lib_can_path)
         
        self.device_id = device_id
        
        if device_serial_number is not None:
            devices = self.find_devices()
            if device_serial_number in devices:
                self.device_id = devices.index(device_serial_number)
            else:
                raise ValueError(f"Device {device_serial_number} not found")
        
         
    def find_devices(self):
        pInfo = VCI_BOARD_INFO_ARRAY(50) # 最大支持50个设备
        dev_num = self.usbcan.VCI_FindUsbDevice2(byref(pInfo.ADDR))
        
        devices = []
        
        if dev_num > 0:
            print(f"find {dev_num} devices")
            for i in range(dev_num):
                serial_number = pInfo.STRUCT_ARRAY[i].str_Serial_Num.decode('utf-8')
                devices.append(serial_number)
                print(f"Device {i}: {serial_number}")
        else:
            print("no device found")
        
        return devices

    def init_can(self): 
        """初始化双路CAN，1M bps"""
        vci_initconfig = VCI_INIT_CONFIG(
            0x80000008, 0xFFFFFFFF, 0, 0, 0x00, 0x14, 0
        )  # 1M bps

        # DWORD __stdcall VCI_OpenDevice(DWORD DevType,DWORD DevIndex,DWORD Reserved);
        # DevIndex 设备索引，比如当只有一个USB-CAN适配器时，索引号为0，这时再插入一个USB-CAN适配器那么后面插入的这个设备索引号就是1，以此类推。
        ret = self.usbcan.VCI_OpenDevice(VCI_USBCAN2, self.device_id, 0) 
        if ret == STATUS_OK:
            print("VCI_OpenDevice successed!")
        else:
            print("VCI_OpenDevice error!")
            os._exit(0)

        for can_channel in [0, 1]: # 双路CAN
            # DWORD __stdcall VCI_InitCAN(DWORD DevType, DWORD DevIndex, DWORD CANIndex, PVCI_INIT_CONFIG pInitConfig);
            ret = self.usbcan.VCI_InitCAN(VCI_USBCAN2, self.device_id, can_channel, byref(vci_initconfig))
            if ret == STATUS_OK:
                print(f"VCI_InitCAN successed! CAN ID: {can_channel}")
            else:
                print(f"VCI_InitCAN error! CAN ID: {can_channel}")
                os._exit(0)

            # DWORD __stdcall VCI_ClearBuffer(DWORD DevType,DWORD DevIndex,DWORD CANIndex);
            self.usbcan.VCI_ClearBuffer(VCI_USBCAN2, self.device_id, can_channel)
            
            # DWORD __stdcall VCI_StartCAN(DWORD DevType,DWORD DevIndex,DWORD CANIndex);
            ret = self.usbcan.VCI_StartCAN(VCI_USBCAN2, self.device_id, can_channel)

            if ret == STATUS_OK:
                print(f"Start OK! CAN ID: {can_channel}")
                time.sleep(0.5)
            else:
                print(f"Start Failed! CAN ID: {can_channel}")
                os._exit(0)

    def send_frame(self, can_channel, id, hex_list):
        """
        发送CAN帧
        :param can_channel: CAN通道，0或1
        :param id: CAN ID
        :param hex_list: 数据列表，长度为8
        :param timeout: 超时时间，单位为ms，默认0表示阻塞等待
        """
        ubyte_array = c_ubyte * 8
        a = ubyte_array(*hex_list)
        ubyte_3array = c_ubyte * 3
        b = ubyte_3array(0, 0, 0)
        vci_can_obj = VCI_CAN_OBJ(id, 0, 0, 1, 0, 0, 8, a, b)
        
        # DWORD __stdcall VCI_Transmit(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,PVCI_CAN_OBJ pSend,DWORD Length);
        ret = self.usbcan.VCI_Transmit(VCI_USBCAN2, self.device_id, can_channel, byref(vci_can_obj), 1)
        rx_vci_can_obj = VCI_CAN_OBJ_ARRAY(2500)
        if ret:
            while True:
                # DWORD __stdcall VCI_Receive(DWORD DevType, DWORD DevIndex, DWORD CANIndex, PVCI_CAN_OBJ pReceive, ULONG Len, INT WaitTime);
                ret = self.usbcan.VCI_Receive(
                    VCI_USBCAN2, self.device_id, can_channel, byref(rx_vci_can_obj.ADDR), 2500, 0
                )
                if ret:
                    feedback_frame = rx_vci_can_obj.STRUCT_ARRAY[0].Data[:8]
                    return feedback_frame
    
    def read_control_mode(self, can_channel, id, max_retry=20):
        control_mode = 0
        for i in range(max_retry):
            feedback_frame = self.send_frame(can_channel, 0x7FF, [id, 0x00, 0x33, 0x0A] + [0x00] * 4)
            if feedback_frame[0] == id:  # match
                control_mode = int.from_bytes(feedback_frame[4:8], byteorder="little")
                break
            time.sleep(0.1)
        return control_mode

    def write_control_mode(self, can_channel, id, mode, max_retry=20):
        control_mode = 0
        mode = int.to_bytes(mode, 4, byteorder="little")
        for i in range(max_retry):
            feedback_frame = self.send_frame(can_channel, 0x7FF, [id, 0x00, 0x55, 0x0A] + list(mode))
            if feedback_frame[0] == id:  # match
                control_mode = int.from_bytes(feedback_frame[4:8], byteorder="little")
                break
            time.sleep(0.1)
        return control_mode
    
    def read_vel_kp(self, can_channel, id, max_retry=20):
        data = 0
        for i in range(max_retry):
            feedback_frame = self.send_frame(can_channel, 0x7FF, [id, 0x00, 0x33, 0x19] + [0x00] * 4)
            if feedback_frame[0] == id:  # match
                byte_data = bytes(feedback_frame[4:8])
                data = struct.unpack("<f", byte_data)[0]
                break
            time.sleep(0.1)
        return data
    
    def write_vel_kp(self, can_channel, id, kp, max_retry=20):
        data = 0
        if kp < 0:
            print('kp must be positive!')
            return 0
            
        for i in range(max_retry):
            hex = struct.pack("<f", kp)
            feedback_frame = self.send_frame(can_channel, 0x7FF, [id, 0x00, 0x55, 0x19] + list(hex))
            if feedback_frame[0] == id:  # match
                byte_data = bytes(feedback_frame[4:8])
                data = struct.unpack("<f", byte_data)[0]
                break
            time.sleep(0.1)
        return data
    
    
    def read_vel_ki(self, can_channel, id, max_retry=20):
        data = 0
        for i in range(max_retry):
            feedback_frame = self.send_frame(can_channel, 0x7FF, [id, 0x00, 0x33, 0x1A] + [0x00] * 4)
            if feedback_frame[0] == id:  # match
                byte_data = bytes(feedback_frame[4:8])
                data = struct.unpack("<f", byte_data)[0]
                break
            time.sleep(0.1)
        return data
    
    def read_pos_kp(self, can_channel, id, max_retry=20):
        data = 0
        for i in range(max_retry):
            feedback_frame = self.send_frame(can_channel, 0x7FF, [id, 0x00, 0x33, 0x1B] + [0x00] * 4)
            if feedback_frame[0] == id:  # match
                byte_data = bytes(feedback_frame[4:8])
                data = struct.unpack("<f", byte_data)[0]
                break
            time.sleep(0.1)
        return data
    
    def write_pos_kp(self, can_channel, id, kp, max_retry=20):
        data = 0
        if kp < 0:
            print('kp must be positive!')
            return 0
            
        for i in range(max_retry):
            hex = struct.pack("<f", kp)
            feedback_frame = self.send_frame(can_channel, 0x7FF, [id, 0x00, 0x55, 0x1B] + list(hex))
            if feedback_frame[0] == id:  # match
                byte_data = bytes(feedback_frame[4:8])
                data = struct.unpack("<f", byte_data)[0]
                break
            time.sleep(0.1)
        return data
    
    def read_pos_ki(self, can_channel, id, max_retry=20):
        data = 0
        for i in range(max_retry):
            feedback_frame = self.send_frame(can_channel, 0x7FF, [id, 0x00, 0x33, 0x1C] + [0x00] * 4)
            if feedback_frame[0] == id:  # match
                byte_data = bytes(feedback_frame[4:8])
                data = struct.unpack("<f", byte_data)[0]
                break
            time.sleep(0.1)
        return data
    
    def read_acc(self, can_channel, id, max_retry=20):
        data = 0
        for i in range(max_retry):
            feedback_frame = self.send_frame(can_channel, 0x7FF, [id, 0x00, 0x33, 0x04] + [0x00] * 4)
            if feedback_frame[0] == id:  # match
                byte_data = bytes(feedback_frame[4:8])
                data = struct.unpack("<f", byte_data)[0]
                break
            time.sleep(0.1)
        return data
    
    
    def write_acc(self, can_channel, id, acc, max_retry=20):
        data = 0
        if acc < 0:
            print('acc must be positive!')
            return 0
        
        for i in range(max_retry):
            hex = struct.pack("<f", acc)
            feedback_frame = self.send_frame(can_channel, 0x7FF, [id, 0x00, 0x55, 0x04] + list(hex))
            if feedback_frame[0] == id:  # match
                byte_data = bytes(feedback_frame[4:8])
                data = struct.unpack("<f", byte_data)[0]
                break
            time.sleep(0.1)
        return data
    
    
    def read_dec(self, can_channel, id, max_retry=20):
        data = 0
        for i in range(max_retry):
            feedback_frame = self.send_frame(can_channel, 0x7FF, [id, 0x00, 0x33, 0x05] + [0x00] * 4)
            if feedback_frame[0] == id:  # match
                byte_data = bytes(feedback_frame[4:8])
                data = struct.unpack("<f", byte_data)[0]
                break
            time.sleep(0.1)
        return data

    def write_dec(self, can_channel, id, dec, max_retry=20):
        data = 0
        if dec > 0:
            print('dec must be negative!')
            return 0
        
        for i in range(max_retry):
            hex = struct.pack("<f", dec)
            feedback_frame = self.send_frame(can_channel, 0x7FF, [id, 0x00, 0x55, 0x05] + list(hex))
            if feedback_frame[0] == id:  # match
                byte_data = bytes(feedback_frame[4:8])
                data = struct.unpack("<f", byte_data)[0]
                break
            time.sleep(0.1)
        return data


if __name__ == "__main__":
    can = VCICAN(device_serial_number='31F10002CCE')
    can.find_devices()
    
    
    