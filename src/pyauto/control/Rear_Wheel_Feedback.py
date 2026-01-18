"""
Rear_Wheel_Feedback
author: huiming zhou
"""

import math

from ..common.utils import process_wheel_angle
from ..config.config import Config
from .controller_base import ControlCommand, ControllerBase
from .path_structs import PATH, Node


class RearWheelFeedbackController(ControllerBase):
    def __init__(self, config: Config):
        super().__init__(config)
        self.config = config

    def ComputeControlCommand(self, node: Node, reference: PATH) -> ControlCommand:
        ref_path = reference

        target_ind = ref_path.calc_nearest_ind(node)
        ed, e_phi = ref_path.cal_ed_e_phi(node, target_ind)

        # 处理航向误差为零的情况，避免除零
        if abs(e_phi) < 1e-6:
            e_phi = 1e-6 if e_phi >= 0 else -1e-6

        k = ref_path.ccurv[target_ind]

        vr = node.v

        yaw_rate = (
            vr * k * math.cos(e_phi) / (1.0 - k * ed)
            - self.config.rear_wheel_feedback.K_theta * abs(vr) * e_phi
            - self.config.rear_wheel_feedback.K_e * vr * math.sin(e_phi) * ed / e_phi
        )

        delta = math.atan2(self.config.WB * yaw_rate, vr)
        # 后处理
        delta = process_wheel_angle(delta, -self.config.MAX_STEER, self.config.MAX_STEER)
        return ControlCommand(
            steer=delta,
            target_ind=target_ind,
            lat_error=ed,
            yaw_error=e_phi,
        )
