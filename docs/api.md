# Robot API 文档

本文档介绍 `robot/robot.py` 中 `Robot` 类的主要接口、控制模式、状态读取方法与使用说明。

---

## 📏 单位约定

- **角度**：`rad`
- **距离**：`m`
- **频率**：`Hz`
- **温度**：`°C`
- **力矩**：`Nm`

---

## 🧩 常量定义

```python
MODE_MIT = 1
MODE_PV = 2
MODE_PVT = 4
```

- `MODE_MIT = 1`：MIT 力位混合控制模式
- `MODE_PV = 2`：位置-速度模式
- `MODE_PVT = 4`：位置-速度-力矩限制模式（**推荐使用**）

---

## 🗂️ 数据类

### `ControlState`

用于记录当前控制模式与上一次控制模式。

```python
@dataclass
class ControlState:
    current_control_state: int
    prev_control_state: int
```

字段说明：

- `current_control_state`：当前控制模式
- `prev_control_state`：上一次控制模式

该结构主要用于在线程循环中检测是否发生了模式切换，并在必要时向电机下发新的控制模式。

---

# 🤖 类：`Robot`

## 1. 初始化

```python
Robot(
    can: VCICAN,
    can_channel=0,
    freq=100,
    control_mode="pvt",
    soft_limit=True,
    config="gx7.yaml"
)
```

### 参数说明

- `can`：`VCICAN` 实例，CAN 总线接口对象
- `can_channel`：CAN 通道编号
  - `0` 表示 CAN1
  - `1` 表示 CAN2
- `freq`：控制线程频率，单位 `Hz`
  - 推荐值：`100 ~ 200`
- `control_mode`：初始控制模式，可选：
  - `"pvt"`
  - `"pv"`
  - `"mit"`
- `soft_limit`：是否启用关节软限位检测
- `config`：机器人配置文件名，位于 `hardware/configs/` 目录中，例如：
  - `gx7.yaml`
  - `gx7-gripper.yaml`

### 配置文件说明

配置文件中定义了：

- 电机数量
- 电机型号
- 各关节位置限制
- 电机能力参数（如最大位置、最大速度、最大力矩）

例如 `gx7.yaml` 中会配置 7 个关节的活动范围，用于运行时软限位检查。

---

## 2. 推荐使用流程

典型使用流程如下：

```python
from libgx7 import VCICAN, GX7

can = VCICAN()
can.init_can()

robot = GX7(
    can,
    can_channel=1,
    freq=200,
    control_mode="pvt",
    soft_limit=True,
    config="gx7.yaml"
)

robot.setup()
robot.run()
```

推荐顺序：

1. 创建 `VCICAN` 对象并初始化 CAN
2. 创建 `Robot/GX7` 对象
3. 调用 `setup()` 完成初始化
4. 调用 `run()` 启动控制线程
5. 使用 `setJPVTs()`、`setJPVs()`、`setJPs()` 等接口发送控制目标
6. 程序结束前调用 `stop()`

---

## 3. 生命周期控制

### `setup()`

执行机器人初始化流程：

1. 失能所有电机
2. 清除错误
3. 等待约 1 秒
4. 重新使能所有电机

```python
robot.setup()
```

> ⚠️ 必须在 `run()` 前调用，否则控制线程会提示 `Please setup first!`

---

### `run()`

启动机器人后台控制线程。

```python
robot.run()
```

说明：

- 若线程已启动，则不会重复启动
- 内部创建守护线程并执行 `loop()`
- 启动后会打印 `Robot thread started.`

---

### `stop()`

停止机器人控制线程。

```python
robot.stop()
```

说明：

- 设置 `running = False`
- 等待线程退出
- 在线程退出时会自动尝试失能所有电机

---

## 4. 控制模式管理

### `get_mode()`

获取当前控制模式。

```python
mode = robot.get_mode()
```

返回值为：

- `1`：MIT
- `2`：PV
- `4`：PVT

---

### `switch_mit()`

切换到 MIT 模式。

```python
robot.switch_mit()
```

切换逻辑：

- 若当前已经是 MIT，则直接返回
- 否则先读取当前关节位置
- 使用当前位置作为 MIT 控制目标，避免切换瞬间跳变
- 更新控制状态，在线程中完成实际模式切换

---

### `switch_pvt()`

切换到 PVT 模式。

```python
robot.switch_pvt()
```

切换逻辑：

- 若当前已经是 PVT，则直接返回
- 否则先读取当前位置
- 设置目标位置为当前位置
- 速度设为 `0`
- 力矩限制设为 `0.6`
- 在线程中切换模式

---

### `switch_pv()`

切换到 PV 模式。

```python
robot.switch_pv()
```

切换逻辑：

- 若当前已经是 PV，则直接返回
- 否则先读取当前位置
- 设置目标位置为当前位置
- 速度限制设为 `0.5`
- 在线程中切换模式

---

## 5. 状态获取接口

### `get_status()`

获取完整电机状态对象。

```python
status = robot.get_status()
```

返回值类型：`MotorStatus`

通常包含：

- 电机 ID
- 状态码
- 关节位置
- 关节速度
- 关节力矩
- MOS 温度
- 转子温度
- 时间戳

---

### `get_delay()`

获取各电机反馈延迟，单位为毫秒。

```python
delays = robot.get_delay()
```

返回值示例：

```python
[1.2, 1.3, 1.1, 1.4, 1.0, 1.2, 1.3]
```

该值通过当前时间与反馈时间戳的差值计算得到，可用于粗略观察通信实时性。

---

### 单项状态读取

#### `getJP()`

获取关节位置数组。

```python
positions = robot.getJP()
```

返回：`np.ndarray`

---

#### `getJV()`

获取关节速度数组。

```python
velocities = robot.getJV()
```

返回：`np.ndarray`

---

#### `getJT()`

获取关节力矩数组。

```python
torques = robot.getJT()
```

返回：`np.ndarray`

---

#### `getJTimestamp()`

获取关节反馈时间戳数组。

```python
timestamps = robot.getJTimestamp()
```

返回：`np.ndarray`

---

#### `getRotorTemp()`

获取各电机转子温度。

```python
rotor_temps = robot.getRotorTemp()
```

返回：`np.ndarray`

---

#### `getMossTemp()`

获取各电机 MOS 温度。

```python
mos_temps = robot.getMossTemp()
```

返回：`np.ndarray`

---

### `get_status_summary()`

以可读字符串形式返回当前机器人状态摘要。

```python
print(robot.get_status_summary())
```

输出内容通常包括：

- 当前控制模式
- 关节位置
- 关节速度
- 关节力矩
- 转子温度
- MOS 温度
- 末端位姿（如果正运动学计算成功）

> 💡 该接口适合在交互式调试或命令行快速查看状态时使用。

---

## 6. 状态更新与安全检查

### `update_status(feedbacks_all)`

根据底层反馈结果更新全局状态。

```python
robot.update_status(feedbacks_all)
```

一般由内部控制线程自动调用，普通用户通常无需手动调用。

---

### `check_joint_limits()`

检查当前关节位置是否超出配置文件中的软限位。

```python
ok, info = robot.check_joint_limits()
```

返回：

- `ok`：`True / False`
- `info`：错误说明文本

若检测到超限，会返回类似信息：

```text
第3关节超出位置限制！请拖动到合理范围然后重新启动程序
```

> ⚠️ 当 `soft_limit=True` 时，该检查会在控制线程中持续执行。若超限，系统会自动失能并停止线程。

---

### `check_error()`

检查各电机状态码是否存在错误。

```python
ok, info = robot.check_error()
```

支持检测的错误包括：

- `0x8`：超压
- `0x9`：欠压
- `0xA`：过电流
- `0xB`：MOS 过温
- `0xC`：电机线圈过温
- `0xD`：通信丢失
- `0xE`：过载

如果存在错误，则返回：

- `False`
- 对应关节的报错信息

例如：

```text
第2关节过电流！
```

---

## 7. 运动学接口

### `fk(joint_positions)`

正运动学计算：由关节角计算末端位姿。

```python
ee_pose = robot.fk(joint_positions)
```

### 参数

- `joint_positions`：长度为 7 的关节角数组，单位 `rad`

### 返回

返回值由 `Kinematics.fk()` 决定，通常为末端位姿表示。

> 📌 输入关节顺序需与机械臂实际关节编号一致。

---

### `ik(position, orientation, psi=np.pi/2)`

逆运动学计算：由末端位姿求关节角。

```python
q = robot.ik(position, orientation, psi=np.pi / 2)
```

### 参数说明

- `position`：末端位置，形如 `[x, y, z]`
- `orientation`：末端旋转矩阵，形状为 `3×3`
- `psi`：7 轴机械臂的臂角参数，范围通常为 `[-π, π]`

默认姿态矩阵为：

```python
np.array([
    [0, 0, 1],
    [0, 1, 0],
    [-1, 0, 0]
])
```

### 返回

返回可行的逆解结果，由 `Kinematics.ik()` 实现决定。

> 💡 对于 7 轴机械臂，不同的 `psi` 对应不同的冗余解，但末端位姿可以保持不变。

---

## 8. MIT 模式接口

MIT 模式适用于力位混合控制。

### `setJP(id, position)`

设置单个关节的位置目标。

```python
robot.setJP(1, 0.5)
```

说明：

- `id` 从 `1` 开始
- 会自动设置：
  - 目标速度为 `0`
  - `kp = 10`
  - `kd = 2`

---

### `setJPs(positions)`

批量设置所有关节位置目标。

```python
robot.setJPs([0, 0.2, 0.1, 0, 0, 0, 0])
```

内部默认设置：

- 速度：全 `0`
- 力矩：全 `0`
- `kp`：全 `30`
- `kd`：全 `2`

---

### `setJTs(torques)`

批量设置所有关节力矩目标。

```python
robot.setJTs([0, 0, 1.0, 0, 0, 0, 0])
```

内部同时设置：

- 速度：全 `0`
- `kp`：全 `0`
- `kd`：全 `0`

即以力矩控制为主。

---

### `mit_cmd()`

发送 MIT 模式控制命令。

```python
feedbacks = robot.mit_cmd()
```

通常由内部线程自动调用，普通用户一般只需设置目标，不必手动调用。

---

## 9. PVT 模式接口

PVT 模式为推荐模式，适合大多数位置控制任务。

### `setJPVT(id, position, velocity, torque)`

设置单个关节的目标位置、最大速度和最大力矩比例。

```python
robot.setJPVT(1, position=0.5, velocity=1.0, torque=0.6)
```

参数说明：

- `id`：关节编号，从 `1` 开始
- `position`：目标位置，单位 `rad`
- `velocity`：运动过程中的最大速度限制
- `torque`：最大力矩限制比例
  - `1.0` 表示最大力矩
  - `0.6` 表示 60% 最大力矩

---

### `setJPVTs(positions, velocities, torques)`

批量设置所有关节的目标位置、速度限制和力矩限制。

```python
robot.setJPVTs(
    positions=[0, 0.2, 0.1, 0, 0, 0, 0],
    velocities=[1.0] * 7,
    torques=[0.6] * 7
)
```

---

### `pvt_cmd()`

发送 PVT 模式控制命令。

```python
feedbacks = robot.pvt_cmd()
```

通常由控制线程自动执行。

---

## 10. PV 模式接口

PV 模式适用于仅设置位置与速度限制的控制场景。

### `setJPV(id, position, velocity)`

设置单个关节的位置目标和速度限制。

```python
robot.setJPV(1, position=0.5, velocity=1.0)
```

---

### `setJPVs(positions, velocities)`

批量设置所有关节的位置目标和速度限制。

```python
robot.setJPVs(
    positions=[0, 0.2, 0.1, 0, 0, 0, 0],
    velocities=[1.0] * 7
)
```

---

### `pv_cmd()`

发送 PV 模式控制命令。

```python
feedbacks = robot.pv_cmd()
```

通常由内部线程自动执行。

---

## 11. 电机使能控制

### `enable()`

使能所有电机。

```python
robot.enable()
```

---

### `disable()`

失能所有电机。

```python
robot.disable()
```

> ⚠️ 失能后机械臂可能在重力作用下下坠，请提前做好支撑与防护。

---

## 12. 主控制循环

### `loop()`

后台控制线程的核心循环函数。

该函数通常由 `run()` 自动启动，不建议用户手动调用。

内部主要执行以下逻辑：

1. 检查是否已完成 `setup()`
2. 检查关节软限位
3. 检查电机错误状态
4. 检测是否需要切换控制模式
5. 根据当前模式发送控制指令
6. 更新反馈状态
7. 按设定频率精确睡眠

当出现异常或线程退出时：

- 自动尝试失能所有电机
- 打印退出信息
- 将 `running` 置为 `False`

---

## 13. 示例

### PVT 模式位置控制示例

```python
import time
from libgx7 import VCICAN, GX7

can = VCICAN()
can.init_can()

robot = GX7(
    can,
    can_channel=1,
    freq=200,
    control_mode="pvt",
    soft_limit=True,
    config="gx7.yaml"
)

robot.setup()
robot.run()

target = [0.0, 0.3, 0.0, 1.0, 0.0, 0.5, 0.0]
vel = [1.0] * 7
tau = [0.6] * 7

robot.setJPVTs(target, vel, tau)

time.sleep(3)
robot.stop()
```

---

### 模式切换示例

```python
robot.switch_mit()
robot.setJPs([0, 0, 0, 0, 0, 0, 0])

robot.switch_pv()
robot.setJPVs([0, 0.2, 0, 0, 0, 0, 0], [0.5] * 7)

robot.switch_pvt()
robot.setJPVTs([0, 0.2, 0, 0, 0, 0, 0], [1.0] * 7, [0.6] * 7)
```

---

## ⚠️ 注意事项

- `id` **从 1 开始编号**
- 标准 GX7 为 **7 个关节**
- 带夹爪版本中，**夹爪对应 `id=8`**
- 调用 `run()` 前请先执行 `setup()`
- 推荐优先使用 **PVT 模式**
- 若启用 `soft_limit=True`，关节超限后系统会自动停止并失能
- 程序退出或线程异常结束时，电机会自动失能
- 电机失能后机械臂可能因重力直接下坠，请确保环境安全

---

## 📌 补充说明

在交互式开发中，建议优先使用以下接口组合：

- 状态读取：
  - `getJP()`
  - `getJV()`
  - `getJT()`
  - `get_status_summary()`

- 位置控制：
  - `setJPVTs()`（推荐）
  - `setJPVs()`

- 模式切换：
  - `switch_pvt()`
  - `switch_pv()`
  - `switch_mit()`

如果你使用的是带夹爪版本，夹爪也可以像普通转动关节一样使用上述关节控制接口。