import os
import yaml

from .can import VCICAN
from .dm_utils import *

abs_path = os.path.abspath(__file__)


def load_config(config_file):
    with open(config_file, "r") as file:
        return yaml.safe_load(file)


class RobotMotors:  # https://gl1po2nscb.feishu.cn/wiki/VYrlwHI7liHzXIkx0s0cUOVdnzb
    def __init__(self, can, can_channel, gx7_config_file="gx7.yaml"):

        gx7_config_file = os.path.join(
            os.path.dirname(abs_path), f"configs/{gx7_config_file}"
        )

        gx7_configs = load_config(gx7_config_file)

        MOTOR_TYPES = gx7_configs["motor_types"]
        NUM_MOTORS = gx7_configs["robot_config"]["num_motors"]
        MOTOR_CONFIGS = gx7_configs["robot_config"]["motor_configs"]
        MOTOR_NAMES = [c["type"] for c in MOTOR_CONFIGS]
        MOTOR_IDS = [c["id"] for c in MOTOR_CONFIGS]
        MOTOR_LIMITS = [
            [c["limits"]["position"]["lower"], c["limits"]["position"]["upper"]]
            for c in MOTOR_CONFIGS
        ]

        self.can = can
        self.can_channel = can_channel

        self.num_motors = NUM_MOTORS
        self.motor_limits = MOTOR_LIMITS
        self.motor_types = MOTOR_TYPES
        self.motor_names = MOTOR_NAMES
        self.motor_ids = MOTOR_IDS
        self.id_to_motor_name = {c["id"]: c["type"] for c in MOTOR_CONFIGS}

    def _get_motor_name_by_id(self, id):
        if id not in self.id_to_motor_name:
            raise ValueError(
                f"Motor id {id} not found in config. Available ids: {self.motor_ids}"
            )
        return self.id_to_motor_name[id]

    def set_zero(self, id):
        motor_name = self._get_motor_name_by_id(id)
        feedback_frame = self.can.send_frame(
            self.can_channel, id, [0xFF] * 7 + [0xFE]
        )  # zero frame
        return extract_feedback_frame(
            self.motor_types, motor_name, feedback_frame
        )

    def set_zero_all(self):
        for id in self.motor_ids:
            self.set_zero(id)

    def disable_motor(self, id):
        motor_name = self._get_motor_name_by_id(id)
        feedback_frame = self.can.send_frame(
            self.can_channel, id, [0xFF] * 7 + [0xFD]
        )  # disable frame
        return extract_feedback_frame(
            self.motor_types, motor_name, feedback_frame
        )

    def enable_motor(self, id):
        motor_name = self._get_motor_name_by_id(id)
        feedback_frame = self.can.send_frame(
            self.can_channel, id, [0xFF] * 7 + [0xFC]
        )  # enable frame
        return extract_feedback_frame(
            self.motor_types, motor_name, feedback_frame
        )

    def clear_error(self, id):
        self._get_motor_name_by_id(id)
        self.can.send_frame(self.can_channel, id, [0xFF] * 7 + [0xFB])

    def clear_error_all(self):
        for id in self.motor_ids:
            self.clear_error(id)

    def enable_all(self):
        feedbacks_all = []
        for id in self.motor_ids:
            feedback_frame = self.enable_motor(id)
            feedbacks_all.append(feedback_frame)
        return feedbacks_all

    def disable_all(self):
        feedbacks_all = []
        for id in self.motor_ids:
            feedback_frame = self.disable_motor(id)
            feedbacks_all.append(feedback_frame)
        return feedbacks_all

    def set_motor_mit(self, id, pos, vel, kp, kd, torque):
        motor_name = self._get_motor_name_by_id(id)
        torq_control_frame = make_mit_frame(
            self.motor_types, motor_name, pos, vel, kp, kd, torque
        )
        feedback_frame = self.can.send_frame(self.can_channel, id, torq_control_frame)
        return extract_feedback_frame(
            self.motor_types, motor_name, feedback_frame
        )

    def set_motor_mit_all(self, ids, poss, vels, kps, kds, torques):
        feedbacks_all = []
        for id, pos, vel, kp, kd, torque in zip(ids, poss, vels, kps, kds, torques):
            feedback_frame = self.set_motor_mit(id, pos, vel, kp, kd, torque)
            feedbacks_all.append(feedback_frame)
        return feedbacks_all

    def set_motor_pvt(self, id, pos, vel, torque):
        motor_name = self._get_motor_name_by_id(id)
        pvt_control_frame = make_pvt_frame(pos, vel, torque)
        feedback_frame = self.can.send_frame(
            self.can_channel, id + 0x300, pvt_control_frame
        )
        return extract_feedback_frame(
            self.motor_types, motor_name, feedback_frame
        )

    def set_motor_pvt_all(self, ids, poss, vels, torques):
        feedbacks_all = []
        for id, pos, vel, torque in zip(ids, poss, vels, torques):
            feedback_frame = self.set_motor_pvt(id, pos, vel, torque)
            feedbacks_all.append(feedback_frame)
        return feedbacks_all

    def set_motor_pv(self, id, pos, vel):
        motor_name = self._get_motor_name_by_id(id)
        pv_control_frame = make_pv_frame(pos, vel)
        feedback_frame = self.can.send_frame(
            self.can_channel, id + 0x100, pv_control_frame
        )
        return extract_feedback_frame(
            self.motor_types, motor_name, feedback_frame
        )

    def set_motor_pv_all(self, ids, poss, vels):
        feedbacks_all = []
        for id, pos, vel in zip(ids, poss, vels):
            feedback_frame = self.set_motor_pv(id, pos, vel)
            feedbacks_all.append(feedback_frame)
        return feedbacks_all

    def write_control_mode_all(self, mode):
        for id in self.motor_ids:
            self.can.write_control_mode(self.can_channel, id, mode)
            print(f"Joint(id={id}) writing mode done...")
