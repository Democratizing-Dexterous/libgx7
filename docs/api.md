# Robot API 文档

本文档介绍 `robot/robot.py` 中 `Robot` 类及相关方法的用途、参数与使用方法。

---

## 常量定义
- `MODE_MIT = 1`：MIT 力位混合控制模式
- `MODE_PV = 2`：位置限速模式
- `MODE_PVT = 4`：位置-速度-力混合模式 （推荐使用）

---

## 数据类
### `ControlState`
记录机器人当前与先前的控制模式。
- `current_control_state`：当前控制模式（整型）
- `prev_control_state`：上一个控制模式（整型）

---

## 类：`Robot`

### 初始化
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
**参数说明**：
- `can`：CAN 总线接口对象
- `can_channel`：CAN 通道编号（默认 0）
- `freq`：控制线程频率 Hz（最大推荐200Hz）
- `control_mode`：初始控制模式，可选 `"pvt"`，`"pv"`，`"mit"`
- `soft_limit`：是否启用关节软限位检测
- `config`：机器人电机配置文件，在`hardware/configs/`目录下

---

### 运行控制
- `run()`：启动机器人控制线程
- `stop()`：停止机器人控制线程
- `setup()`：执行初始化（失能→清错→使能）

---

### 模式控制
- `get_mode()`：获取当前模式
- `switch_mit()`：切换到 MIT 模式
- `switch_pvt()`：切换到 PVT 模式
- `switch_pv()`：切换到 PV 模式

---

### 状态获取
- `get_status()`：返回全局电机状态 `MotorStatus`
- `get_status_summary()`：返回关节角、速度、力矩、温度等的文字摘要
- `get_delay()`：获取各关节通信延迟（毫秒）

单项状态读取：
- `getJP()`：关节位置
- `getJV()`：关节速度
- `getJT()`：关节力矩
- `getRotorTemp()`：电机转子温度
- `getMossTemp()`：MOSFET 温度

---

### 状态更新
- `update_status(feedbacks_all)`：更新机器人状态
- `check_joint_limits()`：检测关节位置是否超限
- `check_error()`：检测电机工作状态错误码

---

### 运动学
- `fk(joint_positions)`：正运动学（关节位置 → 末端位姿）
- `ik(position, orientation, psi)`：逆运动学（末端位姿 → 关节位置）

---

### MIT 模式接口
- `setJP(id, position)`：设定单个关节位置
- `setJPs(positions)`：批量设定位置（带 PD 增益）
- `setJTs(torques)`：批量设定力矩
- `mit_cmd()`：发送 MIT 模式指令

---

### PVT 模式接口
- `setJPVT(id, position, velocity, torque)`：设定单关节位置、最大速度、最大力矩（百分比，1为最大力矩）
- `setJPVTs(positions, velocities, torques)`：批量设定
- `pvt_cmd()`：发送 PVT 模式指令

---

### PV 模式接口
- `setJPV(id, position, velocity)`：设定单关节位置、最大速度
- `setJPVs(positions, velocities)`：批量设定
- `pv_cmd()`：发送 PV 模式指令

---

### 使能控制
- `enable()`：使能所有电机
- `disable()`：失能所有电机

---

### 主循环
- `loop()`：控制线程循环函数（包含模式切换、限位检测、指令发送）

---

