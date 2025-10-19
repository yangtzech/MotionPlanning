"""
Pure Pursuit
author: huiming zhou
"""

import math

from config_control import Config
from controller_base import ControlCommand, ControllerBase
from path_structs import PATH, Node


class PurePursuitController(ControllerBase):
    def __init__(self, config: Config):
        super().__init__(config)
        self.config = config

    def ComputeControlCommand(self, node: Node, reference: PATH) -> ControlCommand:
        ref_path = reference
        ind, Lf = ref_path.target_index(node)
        tx = ref_path.cx[ind]
        ty = ref_path.cy[ind]
        alpha = math.atan2(ty - node.y, tx - node.x) - node.yaw
        delta = math.atan2(2.0 * self.config.WB * math.sin(alpha), Lf)
        return ControlCommand(
            steer=delta,
        )
