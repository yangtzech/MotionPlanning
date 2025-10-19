import math
from typing import Any, Dict

from config_control import Config
from controller_base import ControlCommand, ControllerBase
from path_structs import PATH, Node


class PIDSpeedController(ControllerBase):
    def __init__(self, config: Config):
        super().__init__(config)
        self.config = config

    def ComputeControlCommand(self, node: Node, reference: PATH) -> ControlCommand:
        # 速度控制逻辑，可根据路径或目标点调整
        target_ind, _ = reference.target_index(node)
        target_v = reference.cv[target_ind]

        # 计算距离
        xt = node.x + self.config.dc * math.cos(node.yaw)
        yt = node.y + self.config.dc * math.sin(node.yaw)
        dist = math.hypot(xt - reference.cx[-1], yt - reference.cy[-1])

        a = self.config.Kp * (target_v - node.v * node.direct)

        if dist < self.config.dist_stop:
            if node.v > 3.0:
                a = -2.5
            elif node.v < -2.0:
                a = -1.0
        return ControlCommand(
            acceleration=a,
        )
