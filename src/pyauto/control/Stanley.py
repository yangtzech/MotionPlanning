"""
Stanley
author: huiming zhou
"""

import math

import numpy as np

from .config_control import Config
from .controller_base import ControlCommand, ControllerBase
from .path_structs import PATH, Node
from .utils import process_wheel_angle


class StanleyController(ControllerBase):
    def __init__(self, config: Config):
        super().__init__(config)
        self.config = config

    def ComputeControlCommand(self, node: Node, reference: PATH) -> ControlCommand:
        ref_path = reference
        fx = node.x + self.config.WB * math.cos(node.yaw)
        fy = node.y + self.config.WB * math.sin(node.yaw)
        node_front = Node(x=fx, y=fy, yaw=node.yaw, v=node.v, direct=node.direct, config=self.config)

        target_ind = ref_path.calc_nearest_ind(node_front)
        ed, e_phi = ref_path.cal_ed_e_phi(node, target_ind)

        delta = -e_phi * np.sign(node.v) + math.atan2(-self.config.stanley.k * ed, abs(node.v))

        # 后处理
        delta = process_wheel_angle(delta, -self.config.MAX_STEER, self.config.MAX_STEER)

        return ControlCommand(
            steer=delta,
            target_ind=target_ind,
            lat_error=ed,
            yaw_error=e_phi,
        )
