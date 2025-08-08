from ctypes import *
import os
import time
import numpy as np
import serial
import struct

abs_path = os.path.abspath(__file__)


#### for VCI usb2can, windows 
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


class VCICAN:
    def __init__(self, platform='win'):
         
        if platform == 'win':
            # win x64 dll
            self.lib_can_path = os.path.join(
                os.path.dirname(abs_path), "libs/ControlCAN.dll"
            )
            self.usbcan = windll.LoadLibrary(self.lib_can_path)
        elif platform == 'linux':
            # linux so
            self.lib_can_path = os.path.join(
                os.path.dirname(abs_path), "libs/libcontrolcan.so"
            )
            self.usbcan = cdll.LoadLibrary(self.lib_can_path)
         
        else:
            print('Not supported OS!!!')
            os._exit(0)

    def init_can(self):
        vci_initconfig = VCI_INIT_CONFIG(
            0x80000008, 0xFFFFFFFF, 0, 0, 0x00, 0x14, 0
        )  # 1M bps

        ret = self.usbcan.VCI_OpenDevice(VCI_USBCAN2, 0, 0)
        if ret == STATUS_OK:
            print("VCI_OpenDevice successed!")
        else:
            print("VCI_OpenDevice error!")
            os._exit(0)

        ret = self.usbcan.VCI_InitCAN(VCI_USBCAN2, 0, 0, byref(vci_initconfig))
        if ret == STATUS_OK:
            print("VCI_InitCAN successed!")
        else:
            print("VCI_InitCAN error!")
            os._exit(0)

        self.usbcan.VCI_ClearBuffer(VCI_USBCAN2, 0, 0)
        ret = self.usbcan.VCI_StartCAN(VCI_USBCAN2, 0, 0)

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
        ret = self.usbcan.VCI_Transmit(VCI_USBCAN2, 0, 0, byref(vci_can_obj), 1)
        rx_vci_can_obj = VCI_CAN_OBJ_ARRAY(2500)
        if ret:
            while True:
                ret = self.usbcan.VCI_Receive(
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
    
    def read_vel_kp(self, id, max_retry=20):
        data = 0
        for i in range(max_retry):
            feedback_frame = self.send_frame(0x7FF, [id, 0x00, 0x33, 0x19] + [0x00] * 4)
            if feedback_frame[0] == id:  # match
                byte_data = bytes(feedback_frame[4:8])
                data = struct.unpack("<f", byte_data)[0]
                break
            time.sleep(0.1)
        return data
    
    def write_vel_kp(self, id, kp, max_retry=20):
        data = 0
        if kp < 0:
            print('kp must be positive!')
            return 0
            
        for i in range(max_retry):
            hex = struct.pack("<f", kp)
            feedback_frame = self.send_frame(0x7FF, [id, 0x00, 0x55, 0x19] + list(hex))
            if feedback_frame[0] == id:  # match
                byte_data = bytes(feedback_frame[4:8])
                data = struct.unpack("<f", byte_data)[0]
                break
            time.sleep(0.1)
        return data
    
    
    def read_vel_ki(self, id, max_retry=20):
        data = 0
        for i in range(max_retry):
            feedback_frame = self.send_frame(0x7FF, [id, 0x00, 0x33, 0x1A] + [0x00] * 4)
            if feedback_frame[0] == id:  # match
                byte_data = bytes(feedback_frame[4:8])
                data = struct.unpack("<f", byte_data)[0]
                break
            time.sleep(0.1)
        return data
    
    def read_pos_kp(self, id, max_retry=20):
        data = 0
        for i in range(max_retry):
            feedback_frame = self.send_frame(0x7FF, [id, 0x00, 0x33, 0x1B] + [0x00] * 4)
            if feedback_frame[0] == id:  # match
                byte_data = bytes(feedback_frame[4:8])
                data = struct.unpack("<f", byte_data)[0]
                break
            time.sleep(0.1)
        return data
    
    def write_pos_kp(self, id, kp, max_retry=20):
        data = 0
        if kp < 0:
            print('kp must be positive!')
            return 0
            
        for i in range(max_retry):
            hex = struct.pack("<f", kp)
            feedback_frame = self.send_frame(0x7FF, [id, 0x00, 0x55, 0x1B] + list(hex))
            if feedback_frame[0] == id:  # match
                byte_data = bytes(feedback_frame[4:8])
                data = struct.unpack("<f", byte_data)[0]
                break
            time.sleep(0.1)
        return data
    
    def read_pos_ki(self, id, max_retry=20):
        data = 0
        for i in range(max_retry):
            feedback_frame = self.send_frame(0x7FF, [id, 0x00, 0x33, 0x1C] + [0x00] * 4)
            if feedback_frame[0] == id:  # match
                byte_data = bytes(feedback_frame[4:8])
                data = struct.unpack("<f", byte_data)[0]
                break
            time.sleep(0.1)
        return data
    
    def read_acc(self, id, max_retry=20):
        data = 0
        for i in range(max_retry):
            feedback_frame = self.send_frame(0x7FF, [id, 0x00, 0x33, 0x04] + [0x00] * 4)
            if feedback_frame[0] == id:  # match
                byte_data = bytes(feedback_frame[4:8])
                data = struct.unpack("<f", byte_data)[0]
                break
            time.sleep(0.1)
        return data
    
    
    def write_acc(self, id, acc, max_retry=20):
        data = 0
        if acc < 0:
            print('acc must be positive!')
            return 0
        
        for i in range(max_retry):
            hex = struct.pack("<f", acc)
            feedback_frame = self.send_frame(0x7FF, [id, 0x00, 0x55, 0x04] + list(hex))
            if feedback_frame[0] == id:  # match
                byte_data = bytes(feedback_frame[4:8])
                data = struct.unpack("<f", byte_data)[0]
                break
            time.sleep(0.1)
        return data
    
    
    def read_dec(self, id, max_retry=20):
        data = 0
        for i in range(max_retry):
            feedback_frame = self.send_frame(0x7FF, [id, 0x00, 0x33, 0x05] + [0x00] * 4)
            if feedback_frame[0] == id:  # match
                byte_data = bytes(feedback_frame[4:8])
                data = struct.unpack("<f", byte_data)[0]
                break
            time.sleep(0.1)
        return data

    def write_dec(self, id, dec, max_retry=20):
        data = 0
        if dec > 0:
            print('dec must be negative!')
            return 0
        
        for i in range(max_retry):
            hex = struct.pack("<f", dec)
            feedback_frame = self.send_frame(0x7FF, [id, 0x00, 0x55, 0x05] + list(hex))
            if feedback_frame[0] == id:  # match
                byte_data = bytes(feedback_frame[4:8])
                data = struct.unpack("<f", byte_data)[0]
                break
            time.sleep(0.1)
        return data


if __name__ == "__main__":
    can = VCICAN()
    can.init_can()
    
    