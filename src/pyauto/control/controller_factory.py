"""控制器工厂模块 - 使用工厂模式创建控制器实例"""

from typing import Dict, Type

from pyauto.config.config import Config
from pyauto.control.controller_base import ControllerBase
from pyauto.control.LQR_Dynamic_Model import LQRDynamicController
from pyauto.control.LQR_Kinematic_Model import LQRKinematicController
from pyauto.control.MPC_Frenet_Frame import MPC_Frenet_FrameController
from pyauto.control.MPC_XY_Frame import MPC_XY_FrameController
from pyauto.control.Pure_Pursuit import PurePursuitController
from pyauto.control.Rear_Wheel_Feedback import RearWheelFeedbackController
from pyauto.control.Stanley import StanleyController


class ControllerFactory:
    """控制器工厂类 - 负责创建和管理所有控制器"""

    # 控制器注册表：映射控制器名称到类
    _registry: Dict[str, Type[ControllerBase]] = {
        "purepursuit": PurePursuitController,
        "stanley": StanleyController,
        "rearwheelfeedback": RearWheelFeedbackController,
        "lqr_kinematic": LQRKinematicController,
        "lqr_dynamic": LQRDynamicController,
        "mpc_frenet": MPC_Frenet_FrameController,
        "mpc_xy": MPC_XY_FrameController,
    }

    @classmethod
    def create(cls, name: str, config: Config) -> ControllerBase:
        """
        创建控制器实例

        Args:
            name: 控制器名称（不区分大小写）
            config: 配置对象

        Returns:
            控制器实例

        Raises:
            ValueError: 当控制器名称未注册时

        Examples:
            >>> factory = ControllerFactory()
            >>> controller = factory.create("stanley", config)
        """
        name_lower = name.lower()

        if name_lower not in cls._registry:
            available = ", ".join(cls._registry.keys())
            raise ValueError(f"Unknown controller: '{name}'. Available: {available}")

        controller_class = cls._registry[name_lower]
        return controller_class(config)

    @classmethod
    def register(cls, name: str, controller_class: Type[ControllerBase]) -> None:
        """
        注册新的控制器类型（支持运行时扩展）

        Args:
            name: 控制器名称
            controller_class: 控制器类

        Examples:
            >>> class MyController(ControllerBase):
            ...     pass
            >>> ControllerFactory.register("my_controller", MyController)
        """
        cls._registry[name.lower()] = controller_class

    @classmethod
    def list_available(cls) -> list[str]:
        """返回所有可用的控制器名称"""
        return list(cls._registry.keys())


# 便捷函数，兼容旧代码
def get_lat_controller(name: str, config: Config) -> ControllerBase:
    """创建横向控制器（向后兼容的包装函数）"""
    return ControllerFactory.create(name, config)


def get_controller(name: str, config: Config) -> ControllerBase:
    """创建任意控制器"""
    return ControllerFactory.create(name, config)
