import math
from typing import Any, Dict

from config_control import Config
from controller_base import ControllerBase
from path_structs import Node

config = Config()


class PIDSpeedController(ControllerBase):
    def __init__(self, config: Dict[str, Any] = None):
        super().__init__(config)

    def ComputeControlCommand(self, node: Node, reference: Dict[str, Any]) -> float:
        """
        PID controller and design speed profile.
        :param target_v: target speed (forward and backward are different)
        :param v: current speed
        :param dist: distance from current position to end position
        :param direct: current direction
        :return: desired acceleration
        """
        # self.set_reference(reference["cx"], reference["cy"])
        # self.target_ind, _ = self.ref_path.target_index(node, config)
        # 控制参数
        target_speed = reference.get("target_speed", 30.0 / 3.6)

        # 计算距离
        xt = node.x + config.dc * math.cos(node.yaw)
        yt = node.y + config.dc * math.sin(node.yaw)
        dist = math.hypot(xt - reference["cx"][-1], yt - reference["cy"][-1])
        a = 0.3 * (target_speed - node.v * node.direct)
        if dist < 10.0:
            if node.v > 3.0:
                a = -2.5
            elif node.v < -2.0:
                a = -1.0
        return a
