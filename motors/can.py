from ctypes import *
import os
import time
import numpy as np

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
    def __init__(self):
        # win x64 dll
        self.lib_can_path = os.path.join(
            os.path.dirname(abs_path), "libs/ControlCAN.dll"
        )
        self.usbcan = windll.LoadLibrary(self.lib_can_path)

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


#### for SOULDE usb2can, for linux

FRAME_lENGTH = 8
# 定义结构体，与C中的FrameInfo对应
class FrameInfo(Structure):
    _fields_ = [
        ("canID", c_uint32),
        ("frameType", c_uint8),
        ("dataLength", c_uint8)
    ]
    _pack_ = 1  # 保证与C结构体相同的内存对齐方式

class SOLCAN:
    def __init__(self, dev_name='/dev/USB2CAN0'):
        # linux so
        self.lib_can_path = os.path.join(
            os.path.dirname(abs_path), "libs/libusb_can.so"
        )
        self.usbcan = cdll.LoadLibrary(self.lib_can_path)


        self.usbcan.openUSBCAN.argtypes = [c_char_p]
        self.usbcan.openUSBCAN.restype = c_int32

        self.usbcan.closeUSBCAN.argtypes = [c_int32]
        self.usbcan.closeUSBCAN.restype = c_int32

        self.usbcan.sendUSBCAN.argtypes = [c_int32, c_uint8, POINTER(FrameInfo), POINTER(c_uint8)]
        self.usbcan.sendUSBCAN.restype = c_int32

        self.usbcan.readUSBCAN.argtypes = [c_int32, POINTER(c_uint8), POINTER(FrameInfo), POINTER(c_uint8), c_int32]
        self.usbcan.readUSBCAN.restype = c_int32

        self.dev_name = dev_name
    
    def init_can(self):
        self.dev_handle = self.usbcan.openUSBCAN(self.dev_name.encode('utf-8'))
        if self.dev_handle > 0:
            print("SOL_OpenDevice successed!")
        else:
            print("SOL_OpenDevice error!")
            os._exit(0)

    def send_frame(self, id, hex_list, timeout=0):
        if self.dev_handle < 0:
            raise RuntimeError("Device not opened")
        
        frame_info = FrameInfo()
        frame_info.canID = id
        frame_info.frameType = 0
        frame_info.dataLength = FRAME_lENGTH
        
        # data转换成uinit8_t*
        data_array = (c_uint8 * len(hex_list))(*hex_list)
    
        send_ret = self.usbcan.sendUSBCAN(self.dev_handle, 1, byref(frame_info), data_array) # 使用通道1发送

        data_buffer = (c_uint8 * FRAME_lENGTH)()
        channel = c_uint8(0)

        if send_ret == 17:
            while True:
                read_ret = self.usbcan.readUSBCAN(self.dev_handle, byref(channel), byref(frame_info), data_buffer, 1000)
                if read_ret == 0:
                    return list(data_buffer)

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

if __name__ == "__main__":
    can = SOLCAN()
    can.init_can()