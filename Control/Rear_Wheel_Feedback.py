"""
Rear_Wheel_Feedback
author: huiming zhou
"""

import math

from config_control import Config
from controller_base import ControlCommand, ControllerBase
from path_structs import PATH, Node
from utils import pi_2_pi


class RearWheelFeedbackController(ControllerBase):
    def __init__(self, config: Config):
        super().__init__(config)
        self.config = config

    def ComputeControlCommand(self, node: Node, reference: PATH) -> ControlCommand:
        ref_path = reference

        target_ind = ref_path.calc_nearest_ind(node)
        ed, e_phi = ref_path.cal_ed_e_phi(node, target_ind)

        k = ref_path.ccurv[target_ind] * ref_path.cdirect[target_ind]

        vr = node.v

        yaw_rate = (
            vr * k * math.cos(e_phi) / (1.0 - k * ed)
            - self.config.rear_wheel_feedback.K_theta * (vr) * e_phi
            - self.config.rear_wheel_feedback.K_e
            * abs(vr)
            * math.sin(e_phi)
            * ed
            / e_phi
        )

        delta = math.atan2(self.config.WB * yaw_rate, vr)
        delta = pi_2_pi(delta)
        return ControlCommand(
            steer=delta,
            target_ind=target_ind,
            lat_error=ed,
            yaw_error=e_phi,
        )
