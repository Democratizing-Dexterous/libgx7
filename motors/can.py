from ctypes import *
import os
import time
import numpy as np
import serial

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
                os.path.dirname(abs_path), "libs/libusb_can.so"
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



#### for serial2can
class SerialCAN:

    send_data_frame = np.array(
        [0x55, 0xAA, 0x1e, 0x03, 0x01, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0, 0x00, 0x08, 0x00,
         0x00, 0, 0, 0, 0, 0, 0, 0, 0, 0x00], np.uint8)

    def __init__(self, dev_name='/dev/ttyUSB0'):
        self.dev_name = dev_name
        self.serial = serial.Serial(self.dev_name, 921600, timeout=0.5)

        self.data_save = bytes()  # save data

    def send_frame(self, id, hex_list, timeout=0):
        self.send_data_frame[13] = id & 0xff
        self.send_data_frame[14] = (id >> 8)& 0xff  #id high 8 bits

        data = np.array(hex_list, np.uint8)

        self.send_data_frame[21:29] = data
        self.serial.write(bytes(self.send_data_frame.T))

    def recv(self):
        # 把上次没有解析完的剩下的也放进来
        data_recv = b''.join([self.data_save, self.serial_.read_all()])
        packets = self.__extract_packets(data_recv)
        for packet in packets:
            data = packet[7:15]
            CANID = (packet[6] << 24) | (packet[5] << 16) | (packet[4] << 8) | packet[3]
            CMD = packet[1]
            self.__process_packet(data, CANID, CMD)
    
    def __extract_packets(self, data):
        frames = []
        header = 0xAA
        tail = 0x55
        frame_length = 16
        i = 0
        remainder_pos = 0

        while i <= len(data) - frame_length:
            if data[i] == header and data[i + frame_length - 1] == tail:
                frame = data[i:i + frame_length]
                frames.append(frame)
                i += frame_length
                remainder_pos = i
            else:
                i += 1
        self.data_save = data[remainder_pos:]
        return frames
 
