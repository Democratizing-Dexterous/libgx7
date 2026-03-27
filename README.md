# libgx7

> GX7 机械臂 Python 控制库，支持 USB2CAN 通信、双通道连接、7 轴逆运动学与带夹爪版本配置。

## ✨ 特性

- 支持 **Ubuntu 20.04 / 22.04**
- 支持 **Python 3.8+**
- 支持 **USB2CAN 双 CAN 通道**
- 支持 **GX7 机械臂** 与 **带夹爪版本**
- 提供 **7 轴逆运动学示例**
- 提供 **API 文档** 与 **动力学辨识工具**

---

## 🧰 运行环境

- Ubuntu 20.04 / 22.04
- Python 3.8+

---

## 📦 安装方式

在项目根目录执行：

```bash
pip install -e .
```

---

## 🔐 USB2CAN 权限设置

首次使用 USB2CAN 前，请执行以下命令配置设备权限。执行完成后，**重新拔插 USB 设备**，或**重启系统**使配置生效。

```bash
sudo cp libgx7/hardware/99-myusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

---

## 🔌 USB2CAN 连接说明

USB2CAN 设备包含两个 CAN 通道，外壳上标记为：

- **CAN1** → 对应代码中的 `can_channel=0`
- **CAN2** → 对应代码中的 `can_channel=1`

<div align="center">
  <img src="assets/usb2can连接.png" alt="usb2can连接" width="60%">
</div>

> 💡 使用时请确认硬件接入的通道编号与代码中的 `can_channel` 保持一致。

---

## 🛠️ 机械臂安装方式

机械臂通过两个 **G 型夹** 固定在桌面边缘，如下图所示：

<div align="center">
  <img src="assets/机械臂安装.jpg" alt="机械臂安装" width="55%">
</div>

机械臂坐标轴定义如下：

<div align="center">
  <img src="assets/坐标轴.png" alt="机械臂坐标轴" width="55%">
</div>

机械臂的 URDF 模型请参考：

- [gx7_description](https://github.com/Democratizing-Dexterous/gx7_description)

---

## 🚀 快速开始

### 基础初始化示例

```python
import numpy as np
from libgx7 import VCICAN, GX7

can = VCICAN()
can.init_can()

FREQ = 200  # Hz

# 如果使用 CAN1 通道连接机械臂，can_channel=0
# 如果使用 CAN2 通道连接机械臂，can_channel=1
robot = GX7(
    can,
    can_channel=1,
    freq=FREQ,
    control_mode="pvt",
    soft_limits=True,
    config="gx7.yaml",
)

robot.setup()
robot.run()  # Start the robot thread
```

---

## 🔀 单个 USB2CAN 连接两台机械臂

通过指定不同的 `can_channel`，可以使用一个 USB2CAN 同时连接两台机械臂：

```python
robot1 = GX7(can, can_channel=0, freq=FREQ, control_mode="pvt")
robot2 = GX7(can, can_channel=1, freq=FREQ, control_mode="pvt")
```

---

## 🤏 带夹爪版本初始化

如果使用的是**带夹爪版本**的 GX7，请使用对应配置文件初始化：

```python
robot = GX7(
    can,
    can_channel=1,
    freq=FREQ,
    control_mode="pvt",
    soft_limits=True,
    config="gx7-gripper.yaml",
)
```

> 📌 注意：夹爪本质上也是一个转动关节（`id=8`），因此兼容以下接口：
>
> - `getJP`
> - `setJPVT`
> - `setJPVTs`
> - 以及其他面向关节控制的方法

---

## 🧠 逆运动学示例

7 轴逆运动学示例代码位于：

```bash
examples/sew_ik.py
```

该示例演示了 **7 轴机械臂逆解的连续变化过程**：
在末端位姿保持不变的情况下，随着不同臂角参数 `psi` 的变化，得到的关节角解也会不同。

逆运动学解析解代码移植自：

- [sew_ik](https://github.com/rpiRobotics/stereo-sew)

下图展示了：**相同末端位姿下，不同 `psi` 对应的不同关节状态**。

<div align="center">
  <img src="assets/ik.png" alt="逆运动学" width="85%">
</div>

---

## 📚 文档

- [API 文档](docs/api.md)
- [动力学辨识说明](scripts/dynamics/readme.md)

---

## ⚠️ 注意事项

- 程序退出后，机械臂会**自动停止运动**。在重力作用下，机械臂可能会有一定阻力地下坠。
- 机械臂**断电后会直接下坠**，请务必做好防护，避免设备损坏或造成人员风险。

---

## ⚙️ 配置说明

默认机械臂配置文件为 [`gx7.yaml`](libgx7/hardware/configs/gx7.yaml)。该文件定义了：

- 电机类型参数
- 电机数量
- 每个关节的角度限制

例如：

- `dmj4310`
- `dmj4340`

并支持为每个关节单独设置位置上下限。

如果需要使用带夹爪版本，请切换到：

```yaml
gx7-gripper.yaml
```

> 💡 建议根据实际机械结构与安装方式，认真检查各关节限位配置，确保运动安全。

---

## ✅ 项目结构建议阅读顺序

如果你是第一次使用本项目，建议按以下顺序阅读：

1. 安装依赖
2. 配置 USB2CAN 权限
3. 检查 USB2CAN 通道连接
4. 使用 `gx7.yaml` 初始化机械臂
5. 运行基础控制示例
6. 阅读 API 文档
7. 尝试逆运动学与动力学辨识示例

---

## 🙌 备注

如果你在使用过程中遇到问题，建议优先检查：

- USB2CAN 权限是否配置正确
- `can_channel` 是否与实际硬件连接一致
- 配置文件是否选择正确（`gx7.yaml` / `gx7-gripper.yaml`）
- 关节限位是否合理
- 机械臂是否处于安全安装状态

---
