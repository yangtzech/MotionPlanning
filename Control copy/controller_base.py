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
