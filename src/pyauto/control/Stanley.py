"""
Stanley
author: huiming zhou
"""

import math

import numpy as np

from ..config.config import Config
from .controller_base import ControlCommand, ControllerBase
from .path_structs import PATH, Node


class StanleyController(ControllerBase):
    def __init__(self, config: Config):
        super().__init__(config)
        self.config = config

    # --8<-- [start:compute_control_command]
    def ComputeControlCommand(self, node: Node, reference: PATH) -> ControlCommand:
        ref_path = reference
        # --8<-- [start:front_axle_center]
        # 1. 计算前轴中心位置
        fx = node.x + self.config.WB * math.cos(node.yaw)
        fy = node.y + self.config.WB * math.sin(node.yaw)
        node_front = Node(x=fx, y=fy, yaw=node.yaw, v=node.v, direct=node.direct, config=self.config)
        # --8<-- [end:front_axle_center]

        # 2. 找到参考路径上最近点
        target_ind = ref_path.calc_nearest_ind(node_front)

        # 3. 计算横向误差和航向误差
        ed, e_phi = ref_path.cal_ed_e_phi(node, target_ind)

        # --8<-- [start:zero_speed_protection]
        # 4. Stanley 控制律（含零速度保护）
        v_safe = max(abs(node.v), 1e-3)
        delta = -e_phi * np.sign(node.v) + math.atan2(-self.config.stanley.k * ed, v_safe)
        # --8<-- [end:zero_speed_protection]

        # 5. 限制转向角范围
        delta = self.ClampSteeringAngle(delta)

        return ControlCommand(
            steer=delta,
            target_ind=target_ind,
            lat_error=ed,
            yaw_error=e_phi,
        )

    # --8<-- [end:compute_control_command]
