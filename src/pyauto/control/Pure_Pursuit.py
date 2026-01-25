"""
Pure Pursuit
author: huiming zhou
"""

import math

from ..config.config import Config
from .controller_base import ControlCommand, ControllerBase
from .path_structs import PATH, Node


class PurePursuitController(ControllerBase):
    def __init__(self, config: Config):
        super().__init__(config)
        self.config = config

    # --8<-- [start:compute_control_command]
    def ComputeControlCommand(self, node: Node, reference: PATH) -> ControlCommand:
        ref_path = reference
        # --8<-- [start:look_ahead]
        # 1. 计算前视距离并找到前视点
        ind, Lf = ref_path.look_ahead_index(node)
        tx = ref_path.cx[ind]
        ty = ref_path.cy[ind]
        # --8<-- [end:look_ahead]

        # --8<-- [start:control_law]
        # 2. 计算前视点相对于车辆的方位角
        alpha = math.atan2(ty - node.y, tx - node.x) - node.yaw
        # 3. Pure Pursuit 控制律
        delta = math.atan2(2.0 * self.config.WB * math.sin(alpha), Lf)
        # --8<-- [end:control_law]

        # 4. 计算误差信息（用于监控）
        ed, e_phi = ref_path.cal_ed_e_phi(node, ind)

        # 5. 限制转向角范围
        delta = self.ClampSteeringAngle(delta)

        return ControlCommand(
            steer=delta,
            target_ind=ind,
            lat_error=ed,
            yaw_error=e_phi,
        )

    # --8<-- [end:compute_control_command]
