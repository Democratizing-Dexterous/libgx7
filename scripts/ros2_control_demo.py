#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""极简 ROS2 控制 demo：订阅状态 -> 切PVT -> 发布目标并打印角度 -> 查询mode"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger


class Demo(Node):
    def __init__(self):
        super().__init__("gx7_ros2_control_demo")
        self.declare_parameter("topic_prefix", "/gx7")
        p = str(self.get_parameter("topic_prefix").value).strip() or "/gx7"
        self.p = ("/" + p.lstrip("/")).rstrip("/")

        self.state = None
        self.create_subscription(JointState, f"{self.p}/joints_published", self._on_state, 10)
        self.pub = self.create_publisher(JointState, f"{self.p}/joints_goal", 10)
        self.cli_pvt = self.create_client(SetBool, f"{self.p}/set_pvt")
        self.cli_mode = self.create_client(Trigger, f"{self.p}/mode")

    def _on_state(self, msg: JointState):
        self.state = msg

    def _call(self, cli, req, name, timeout=5.0):
        while rclpy.ok() and not cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().info(f"waiting {name} ...")
        fut = cli.call_async(req)
        end = time.time() + timeout
        while rclpy.ok() and not fut.done() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return fut.result() if fut.done() else None

    def run(self):
        self.get_logger().info(f"prefix={self.p}")

        # 等第一帧状态（最多3s）
        t0 = time.time()
        while rclpy.ok() and self.state is None and time.time() - t0 < 3.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        # 切PVT
        resp = self._call(self.cli_pvt, SetBool.Request(data=True), f"{self.p}/set_pvt")
        if not resp or not resp.success:
            self.get_logger().error("set_pvt failed")
            return
        self.get_logger().info(f"set_pvt: {resp.success}, {resp.message}")

        # 发布2秒目标 + 打印当前角度(5Hz)
        dof = 2
        msg = JointState(
            name=[f"joint_{i+1}" for i in range(dof)],
            position=[0.0] * dof,
            velocity=[0.2] * dof,
            effort=[0.6] * dof,
        )
        end_t, last_log = time.time() + 2.0, 0.0
        while rclpy.ok() and time.time() < end_t:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.01)
            now = time.time()
            if self.state and now - last_log >= 0.2:
                pos = ", ".join(f"{x:.4f}" for x in self.state.position)
                self.get_logger().info(f"q=[{pos}]")
                last_log = now
            time.sleep(0.02)  # 50Hz

        # 查mode
        mode = self._call(self.cli_mode, Trigger.Request(), f"{self.p}/mode")
        if mode:
            self.get_logger().info(f"mode: {mode.message}")


def main(args=None):
    rclpy.init(args=args)
    node = Demo()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
