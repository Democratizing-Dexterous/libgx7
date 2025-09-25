import serial
import time
import serial.tools.list_ports
from scipy.interpolate import griddata
import cv2
from .tactile_utils import *


class TactileSerial:
    def __init__(self, port, baudrate=BAUDRATE):
        self.port = port
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1,
            write_timeout=0.5,
            inter_byte_timeout=0.001,  # 高速通信防字节丢失
        )

    def init(self):
        if not self.ser.is_open:
            self.ser.open()
        print(f"\nConnecting Tactile Sensors on {self.port}...")
        write_register(self.ser, 0x0022, [0x01])  # 重启设备
        time.sleep(3)  # 等待上电稳定

        auto_push_cmd = "55AA00101700010001D8"
        if send_hex_cmd(self.ser, auto_push_cmd):
            print("Tactile Auto Push Mode Enabled")

        width, height = 13, 18
        # 1. 取出 XY 作为图像坐标（Z暂不映射坐标）
        locs_xy = force_locations[:, :2]

        # 归一化到图像尺寸范围
        x_min, x_max = locs_xy[:, 0].min(), locs_xy[:, 0].max()
        y_min, y_max = locs_xy[:, 1].min(), locs_xy[:, 1].max()

        norm_x = (locs_xy[:, 0] - x_min) / (x_max - x_min) * (width - 1)
        norm_y = (locs_xy[:, 1] - y_min) / (y_max - y_min) * (height - 1)

        points_2d = np.stack([norm_x, norm_y], axis=1)

        # 3. 建立图像网格
        grid_x, grid_y = np.meshgrid(np.arange(width), np.arange(height))

        self.width = width
        self.height = height
        self.points_2d = points_2d
        self.grid_x = grid_x
        self.grid_y = grid_y

    def read_forces(self):

        valid = False
        forces1 = np.zeros((num_points, 3))
        forces2 = np.zeros((num_points, 3))

        image1 = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        image2 = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        tactile_image = np.zeros((self.height * 10, self.width * 20, 3), dtype=np.uint8)

        all_forces1 = np.zeros(3)
        all_forces2 = np.zeros(3)

        push_data = read_serial_data(self.ser, TIMEOUT_AUTO_PUSH, AUTO_PUSH_FRAME_HEAD)
        if push_data:
            parsed = parse_auto_push_data(push_data)
            if parsed is not None and parsed["lrc_valid"]:

                all_forces1[0] = int.from_bytes(
                    parsed["valid_data"][0:2], byteorder="little", signed=True
                )
                all_forces1[1] = int.from_bytes(
                    parsed["valid_data"][2:4], byteorder="little", signed=True
                )
                all_forces1[2] = int.from_bytes(
                    parsed["valid_data"][4:6], byteorder="little", signed=False
                )

                all_forces2[0] = int.from_bytes(
                    parsed["valid_data"][99:101], byteorder="little", signed=True
                )
                all_forces2[1] = int.from_bytes(
                    parsed["valid_data"][101:103], byteorder="little", signed=True
                )
                all_forces2[2] = int.from_bytes(
                    parsed["valid_data"][103:105], byteorder="little", signed=False
                )

                # 更新 forces
                for i in range(num_points):
                    forces1[i, 0] = int.from_bytes(
                        parsed["valid_data"][6 + i * 3 : 6 + i * 3 + 1],
                        byteorder="little",
                        signed=True,
                    )
                    forces1[i, 1] = int.from_bytes(
                        parsed["valid_data"][6 + i * 3 + 1 : 6 + i * 3 + 2],
                        byteorder="little",
                        signed=True,
                    )
                    forces1[i, 2] = int.from_bytes(
                        parsed["valid_data"][6 + i * 3 + 2 : 6 + i * 3 + 3],
                        byteorder="little",
                        signed=False,
                    )
                    forces2[i, 0] = int.from_bytes(
                        parsed["valid_data"][105 + i * 3 : 105 + i * 3 + 1],
                        byteorder="little",
                        signed=True,
                    )
                    forces2[i, 1] = int.from_bytes(
                        parsed["valid_data"][105 + i * 3 + 1 : 105 + i * 3 + 2],
                        byteorder="little",
                        signed=True,
                    )
                    forces2[i, 2] = int.from_bytes(
                        parsed["valid_data"][105 + i * 3 + 2 : 105 + i * 3 + 3],
                        byteorder="little",
                        signed=False,
                    )

                for c in range(3):
                    grid_z = griddata(
                        self.points_2d,
                        forces1[:, c],
                        (self.grid_x, self.grid_y),
                        method="cubic",
                        fill_value=0,
                    )

                    if c < 2:
                        grid_z = grid_z + 128  # 偏移使得负值可视化
                    image1[..., c] = np.clip(grid_z, 0, 255)
                    image1 = image1[::-1, :, :]  # 上下翻转以匹配坐标系

                for c in range(3):
                    grid_z = griddata(
                        self.points_2d,
                        forces2[:, c],
                        (self.grid_x, self.grid_y),
                        method="cubic",
                        fill_value=0,
                    )

                    if c < 2:
                        grid_z = grid_z + 128  # 偏移使得负值可视化
                    image2[..., c] = np.clip(grid_z, 0, 255)
                    image2 = image2[::-1, :, :]  # 上下翻转以匹配坐标系

                image = np.concatenate([image1, image2], axis=1)

                tactile_image = cv2.resize(image, (self.width * 20, self.height * 10))
                valid = True

        return {
            "valid": valid,
            "tac_allforces1": all_forces1,  # 合力
            "tac_allforces2": all_forces2,
            "tac1": forces1,  # 分布力
            "tac2": forces2,
            "tac_img": tactile_image,  # 组合的触觉图像
        }


if __name__ == "__main__":

    tactile = TactileSerial(port="/dev/ttyACM0")

    tactile.init()

    while True:
        data = tactile.read_forces()
        if data["valid"]:
            print("Tactile 1 All Forces:", data["tac_allforces1"])
            print("Tactile 2 All Forces:", data["tac_allforces2"])
            cv2.imshow("Tactile Image", data["tac_img"])
            cv2.waitKey(1)
        time.sleep(0.01)
