"""
Rear_Wheel_Feedback
author: huiming zhou
"""

import math

from ..config.config import Config
from .controller_base import ControlCommand, ControllerBase
from .path_structs import PATH, Node


class RearWheelFeedbackController(ControllerBase):
    def __init__(self, config: Config):
        super().__init__(config)
        self.config = config

    # --8<-- [start:compute_control_command]
    def ComputeControlCommand(self, node: Node, reference: PATH) -> ControlCommand:
        ref_path = reference

        # --8<-- [start:error_calculation]
        # 1. 找到参考路径上最近点
        target_ind = ref_path.calc_nearest_ind(node)
        # 2. 计算横向误差和航向误差
        ed, e_phi = ref_path.cal_ed_e_phi(node, target_ind)
        # --8<-- [end:error_calculation]

        # --8<-- [start:zero_protection]
        # 3. 处理航向误差为零的情况，避免除零
        if abs(e_phi) < 1e-6:
            e_phi = 1e-6 if e_phi >= 0 else -1e-6
        # --8<-- [end:zero_protection]

        # 4. 获取参考路径曲率
        k = ref_path.ccurv[target_ind]
        vr = node.v

        # --8<-- [start:control_law]
        # 5. 后轮反馈控制律
        yaw_rate = (
            vr * k * math.cos(e_phi) / (1.0 - k * ed)
            - self.config.rear_wheel_feedback.K_theta * abs(vr) * e_phi
            - self.config.rear_wheel_feedback.K_e * vr * math.sin(e_phi) * ed / e_phi
        )
        # 6. 计算前轮转向角
        delta = math.atan2(self.config.WB * yaw_rate, vr)
        # --8<-- [end:control_law]

        # 7. 限制转向角范围
        delta = self.ClampSteeringAngle(delta)
        return ControlCommand(
            steer=delta,
            target_ind=target_ind,
            lat_error=ed,
            yaw_error=e_phi,
        )

    # --8<-- [end:compute_control_command]
