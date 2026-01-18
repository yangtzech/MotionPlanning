from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, Dict, Optional


# 统一控制量数据结构
@dataclass
class ControlCommand:
    steer: Optional[float] = None
    acceleration: Optional[float] = None
    brake: Optional[float] = None
    throttle: Optional[float] = None
    target_ind: Optional[int] = None
    lat_error: Optional[float] = None
    yaw_error: Optional[float] = None
    extra: Optional[Dict[str, Any]] = None  # 便于扩展其它控制量


class ControllerBase(ABC):
    """
    控制器基类，所有控制算法需实现ComputeControlCommand方法。
    """

    def __init__(self, config: Dict[str, Any] = None):
        self.config = config or {}

    @abstractmethod
    def ComputeControlCommand(self, node, reference) -> ControlCommand:
        pass

    def ClampSteeringAngle(self, steer: float) -> float:
        """
        限制转向角在有效范围内

        将转向角限制在 [-MAX_STEER, MAX_STEER] 范围内。
        这是所有控制器的通用后处理操作。

        Args:
            steer: 计算得到的转向角

        Returns:
            float: 限制后的转向角
        """
        max_steer = getattr(self.config, "MAX_STEER", 0.698)  # 默认约40度

        # 限制转向角范围
        if steer > max_steer:
            steer = max_steer
        elif steer < -max_steer:
            steer = -max_steer

        return steer
