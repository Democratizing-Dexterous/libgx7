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
        self.declare_parameter("soft_limit", False)
        self.declare_parameter("config", "gx7.yaml")
        self.declare_parameter("publish_rate", 100.0)

        self.declare_parameter("topic_prefix", "/gx7")

        can_channel = int(self.get_parameter("can_channel").value)
        freq = int(self.get_parameter("freq").value)
        control_mode = str(self.get_parameter("control_mode").value)
        soft_limit = bool(self.get_parameter("soft_limit").value)
        config = str(self.get_parameter("config").value)
        publish_rate = float(self.get_parameter("publish_rate").value)

        topic_prefix = str(self.get_parameter("topic_prefix").value).strip()
        if not topic_prefix:
            topic_prefix = "/gx7"
        if not topic_prefix.startswith("/"):
            topic_prefix = "/" + topic_prefix
        self.topic_prefix = topic_prefix.rstrip("/")

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
        # 最多缓存最近 10 条消息；超过后会按 KeepLast 策略丢弃更旧的消息
        self.joint_pub = self.create_publisher(
            JointState, f"{self.topic_prefix}/joints_published", 10
        )
        timer_period = 1.0 / publish_rate if publish_rate > 0 else 0.01
        self.timer = self.create_timer(timer_period, self.publish_joint_state)

        # ---------- Services ----------
        self.srv_mode = self.create_service(
            Trigger, f"{self.topic_prefix}/mode", self.handle_mode
        )
        self.srv_set_pvt = self.create_service(
            SetBool, f"{self.topic_prefix}/set_pvt", self.handle_set_pvt
        )
        self.srv_set_mit = self.create_service(
            SetBool, f"{self.topic_prefix}/set_mit", self.handle_set_mit
        )
        self.srv_set_pv = self.create_service(
            SetBool, f"{self.topic_prefix}/set_pv", self.handle_set_pv
        )

        # unified goal subscriber
        self.sub_joints_goal = self.create_subscription(
            JointState,
            f"{self.topic_prefix}/joints_goal",
            self.handle_joints_goal,
            10,
        )

        self.get_logger().info(
            f"GX7 ROS2 node started. mode={control_mode}, freq={freq}, "
            f"publish_rate={publish_rate}, can_channel={can_channel}, soft_limit={soft_limit}, "
            f"topic_prefix={self.topic_prefix}"
        )

    def handle_joints_goal(self, msg: JointState):
        try:
            dof = self.robot.num_dof
            positions = list(msg.position)
            velocities = list(msg.velocity)
            efforts = list(msg.effort)
            current_mode = self.robot.get_mode()

            if len(positions) != dof:
                self.get_logger().warn(
                    f"{self.topic_prefix}/joints_goal position length={len(positions)} != num_dof={dof}, ignore."
                )
                return

            if current_mode == MODE_MIT:
                if len(efforts) != dof:
                    self.get_logger().warn(
                        f"MIT mode requires effort length={dof}, got {len(efforts)}, ignore."
                    )
                    return
                self.robot.setJTs(efforts)
                self.get_logger().debug(
                    f"Applied MIT torques from joints_goal, len={len(efforts)}"
                )
            elif current_mode == MODE_PVT:
                if len(velocities) != dof or len(efforts) != dof:
                    self.get_logger().warn(
                        f"PVT mode requires velocity/effort length={dof}, got "
                        f"v={len(velocities)}, e={len(efforts)}, ignore."
                    )
                    return
                self.robot.setJPVTs(positions, velocities, efforts)
                self.get_logger().debug(
                    f"Applied PVT command from joints_goal, len={len(positions)}"
                )
            elif current_mode == MODE_PV:
                if len(velocities) != dof:
                    self.get_logger().warn(
                        f"PV mode requires velocity length={dof}, got {len(velocities)}, ignore."
                    )
                    return
                self.robot.setJPVs(positions, velocities)
                self.get_logger().debug(
                    f"Applied PV command from joints_goal, len={len(positions)}"
                )
            else:
                self.get_logger().warn(
                    f"Unknown control mode={current_mode}, ignore {self.topic_prefix}/joints_goal"
                )
        except Exception as e:
            self.get_logger().error(f"handle_joints_goal error: {e}")

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
        self.get_logger().info(
            f"{self.topic_prefix}/mode queried, current mode={mode_str}"
        )
        return response

    # ---------- /gx7/set_pvt ----------
    def handle_set_pvt(self, request, response):
        if request.data:
            try:
                self.robot.switch_pvt()
                response.success = True
                response.message = "Switched to PVT mode."
                self.get_logger().info(
                    f"Mode switched to PVT via {self.topic_prefix}/set_pvt"
                )
            except Exception as e:
                response.success = False
                response.message = f"Failed to switch PVT: {e}"
                self.get_logger().error(response.message)
        else:
            response.success = False
            response.message = "Set request.data=true to switch mode."
            self.get_logger().warn(
                f"{self.topic_prefix}/set_pvt called with data=false, ignored"
            )
        return response

    # ---------- /gx7/set_mit ----------
    def handle_set_mit(self, request, response):
        if request.data:
            try:
                self.robot.switch_mit()
                response.success = True
                response.message = "Switched to MIT mode."
                self.get_logger().info(
                    f"Mode switched to MIT via {self.topic_prefix}/set_mit"
                )
            except Exception as e:
                response.success = False
                response.message = f"Failed to switch MIT: {e}"
                self.get_logger().error(response.message)
        else:
            response.success = False
            response.message = "Set request.data=true to switch mode."
            self.get_logger().warn(
                f"{self.topic_prefix}/set_mit called with data=false, ignored"
            )
        return response

    # ---------- /gx7/set_pv ----------
    def handle_set_pv(self, request, response):
        if request.data:
            try:
                self.robot.switch_pv()
                response.success = True
                response.message = "Switched to PV mode."
                self.get_logger().info(
                    f"Mode switched to PV via {self.topic_prefix}/set_pv"
                )
            except Exception as e:
                response.success = False
                response.message = f"Failed to switch PV: {e}"
                self.get_logger().error(response.message)
        else:
            response.success = False
            response.message = "Set request.data=true to switch mode."
            self.get_logger().warn(
                f"{self.topic_prefix}/set_pv called with data=false, ignored"
            )
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
