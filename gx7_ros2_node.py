#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, SetBool

from libgx7 import VCICAN, GX7
from libgx7.robot.robot import MODE_MIT, MODE_PV, MODE_PVT

"""
# 查询模式
ros2 service call /gx7/mode std_srvs/srv/Trigger "{}"

# 切 PVT
ros2 service call /gx7/set_pvt std_srvs/srv/SetBool "{data: true}"

# 切 MIT
ros2 service call /gx7/set_mit std_srvs/srv/SetBool "{data: true}"

# 切 PV
ros2 service call /gx7/set_pv std_srvs/srv/SetBool "{data: true}"

# 查看发布
ros2 topic echo /gx7/joints_published
"""


class GX7Ros2Node(Node):
    def __init__(self):
        super().__init__("gx7_ros2_node")

        # ---------- Parameters ----------
        self.declare_parameter("can_channel", 1)
        self.declare_parameter("freq", 100)
        self.declare_parameter("control_mode", "pvt")
        self.declare_parameter("soft_limit", True)
        self.declare_parameter("config", "gx7.yaml")
        self.declare_parameter("publish_rate", 100.0)

        can_channel = int(self.get_parameter("can_channel").value)
        freq = int(self.get_parameter("freq").value)
        control_mode = str(self.get_parameter("control_mode").value)
        soft_limit = bool(self.get_parameter("soft_limit").value)
        config = str(self.get_parameter("config").value)
        publish_rate = float(self.get_parameter("publish_rate").value)

        # ---------- Init robot ----------
        self.can = VCICAN()
        self.can.init_can()

        self.robot = GX7(
            self.can,
            can_channel=can_channel,
            freq=freq,
            control_mode=control_mode,
            soft_limit=soft_limit,
            config=config,
        )
        self.robot.setup()
        self.robot.run()

        # ---------- Publisher ----------
        self.joint_pub = self.create_publisher(JointState, "/gx7/joints_published", 10)
        timer_period = 1.0 / publish_rate if publish_rate > 0 else 0.01
        self.timer = self.create_timer(timer_period, self.publish_joint_state)

        # ---------- Services ----------
        self.srv_mode = self.create_service(Trigger, "/gx7/mode", self.handle_mode)
        self.srv_set_pvt = self.create_service(SetBool, "/gx7/set_pvt", self.handle_set_pvt)
        self.srv_set_mit = self.create_service(SetBool, "/gx7/set_mit", self.handle_set_mit)
        self.srv_set_pv = self.create_service(SetBool, "/gx7/set_pv", self.handle_set_pv)

        self.get_logger().info("GX7 ROS2 node started.")

    def mode_to_string(self, mode: int) -> str:
        if mode == MODE_PVT:
            return "pvt"
        if mode == MODE_MIT:
            return "mit"
        if mode == MODE_PV:
            return "pv"
        return f"unknown({mode})"

    def publish_joint_state(self):
        try:
            positions = self.robot.getJP().tolist()
            velocities = self.robot.getJV().tolist()
            efforts = self.robot.getJT().tolist()

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            # 可按需要改成你的真实关节名
            msg.name = [f"joint_{i+1}" for i in range(len(positions))]
            msg.position = positions
            msg.velocity = velocities
            msg.effort = efforts

            self.joint_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"publish_joint_state error: {e}")

    # ---------- /gx7/mode ----------
    def handle_mode(self, request, response):
        mode = self.robot.get_mode()
        mode_str = self.mode_to_string(mode)
        response.success = True
        response.message = mode_str
        return response

    # ---------- /gx7/set_pvt ----------
    def handle_set_pvt(self, request, response):
        if request.data:
            try:
                self.robot.switch_pvt()
                response.success = True
                response.message = "Switched to PVT mode."
            except Exception as e:
                response.success = False
                response.message = f"Failed to switch PVT: {e}"
        else:
            response.success = False
            response.message = "Set request.data=true to switch mode."
        return response

    # ---------- /gx7/set_mit ----------
    def handle_set_mit(self, request, response):
        if request.data:
            try:
                self.robot.switch_mit()
                response.success = True
                response.message = "Switched to MIT mode."
            except Exception as e:
                response.success = False
                response.message = f"Failed to switch MIT: {e}"
        else:
            response.success = False
            response.message = "Set request.data=true to switch mode."
        return response

    # ---------- /gx7/set_pv ----------
    def handle_set_pv(self, request, response):
        if request.data:
            try:
                self.robot.switch_pv()
                response.success = True
                response.message = "Switched to PV mode."
            except Exception as e:
                response.success = False
                response.message = f"Failed to switch PV: {e}"
        else:
            response.success = False
            response.message = "Set request.data=true to switch mode."
        return response

    def destroy_node(self):
        try:
            self.robot.stop()
        except Exception as e:
            self.get_logger().warn(f"robot stop warning: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GX7Ros2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
