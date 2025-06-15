import os
import yaml

from .motor import CAN, extract_feedback_frame, make_mit_frame, make_pvt_frame, make_pv_frame

abs_path = os.path.abspath(__file__)


def load_config(config_file):
    with open(config_file, "r") as file:
        return yaml.safe_load(file)


gx7_configs = load_config(
    os.path.join(os.path.dirname(abs_path), "configs/gx7_test.yaml")
)

MOTOR_TYPES = gx7_configs["motor_types"]
NUM_MOTORS = gx7_configs["robot_config"]["num_motors"]
MOTOR_CONFIGS = gx7_configs["robot_config"]["motor_configs"]


class RobotMotors:  # https://gl1po2nscb.feishu.cn/wiki/VYrlwHI7liHzXIkx0s0cUOVdnzb
    def __init__(self, can):
        self.can = can
        self.can.initCan1()
        self.num_motors = NUM_MOTORS

    def set_zero(self, id):
        feedback_frame = self.can.send_frame(id, [0xFF] * 7 + [0xFE])  # zero frame
        return extract_feedback_frame(
            MOTOR_TYPES, MOTOR_CONFIGS[id - 1], feedback_frame
        )  # id start from 1

    def disable_motor(self, id):
        feedback_frame = self.can.send_frame(id, [0xFF] * 7 + [0xFD])  # disable frame
        return extract_feedback_frame(
            MOTOR_TYPES, MOTOR_CONFIGS[id - 1], feedback_frame
        )  # id start from 1

    def enable_motor(self, id):
        feedback_frame = self.can.send_frame(id, [0xFF] * 7 + [0xFC])  # enable frame
        return extract_feedback_frame(
            MOTOR_TYPES, MOTOR_CONFIGS[id - 1], feedback_frame
        )

    def enable_all(self):
        feedbacks_all = []
        for id in range(1, self.num_motors + 1):
            feedback_frame = self.enable_motor(id)
            feedbacks_all.append(feedback_frame)
        return feedbacks_all

    def disable_all(self):
        feedbacks_all = []
        for id in range(1, self.num_motors + 1):
            feedback_frame = self.disable_motor(id)
            feedbacks_all.append(feedback_frame)
        return feedbacks_all

    def set_motor_mit(self, id, pos, vel, kp, kd, torque):
        torq_control_frame = make_mit_frame(
            MOTOR_TYPES, MOTOR_CONFIGS[id - 1], pos, vel, kp, kd, torque
        )
        feedback_frame = self.can.send_frame(id, torq_control_frame)
        return extract_feedback_frame(
            MOTOR_TYPES, MOTOR_CONFIGS[id - 1], feedback_frame
        )

    def set_motor_mit_all(self, ids, poss, vels, kps, kds, torques):
        feedbacks_all = []
        for id, pos, vel, kp, kd, torque in zip(ids, poss, vels, kps, kds, torques):
            feedback_frame = self.set_motor_mit(id, pos, vel, kp, kd, torque)
            feedbacks_all.append(feedback_frame)
        return feedbacks_all

    def set_motor_pvt(self, id, pos, vel, torque):
        pvt_control_frame = make_pvt_frame(pos, vel, torque)
        feedback_frame = self.can.send_frame(id + 0x300, pvt_control_frame)
        return extract_feedback_frame(
            MOTOR_TYPES, MOTOR_CONFIGS[id - 1], feedback_frame
        )

    def set_motor_pvt_all(self, ids, poss, vels, torques):
        feedbacks_all = []
        for id, pos, vel, torque in zip(ids, poss, vels, torques):
            feedback_frame = self.set_motor_pvt(id, pos, vel, torque)
            feedbacks_all.append(feedback_frame)
        return feedbacks_all

    def set_motor_pv(self, id, pos, vel):
        pv_control_frame = make_pv_frame(pos, vel)
        feedback_frame = self.can.send_frame(id + 0x100, pv_control_frame)
        return extract_feedback_frame(
            MOTOR_TYPES, MOTOR_CONFIGS[id - 1], feedback_frame
        )

    def set_motor_pv_all(self, ids, poss, vels):
        feedbacks_all = []
        for id, pos, vel in zip(ids, poss, vels):
            feedback_frame = self.set_motor_pv(id, pos, vel)
            feedbacks_all.append(feedback_frame)
        return feedbacks_all

    def write_control_mode_all(self, mode):
        for i in range(self.num_motors):
            self.can.write_control_mode(i + 1, mode)
