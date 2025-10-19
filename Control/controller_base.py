from abc import ABC, abstractmethod
from typing import Any, Dict


class ControllerBase(ABC):
    """
    控制器基类，所有控制算法需实现ComputeControlCommand方法。
    """

    def __init__(self, config: Dict[str, Any] = None):
        self.config = config or {}

    @abstractmethod
    def ComputeControlCommand(
        self, state: Dict[str, Any], reference: Dict[str, Any]
    ) -> Dict[str, Any]:
        """
        计算控制指令的统一接口。
        :param state: 当前车辆状态，字典格式
        :param reference: 目标轨迹或参考信息，字典格式
        :return: 控制输出，字典格式
        """
        pass
