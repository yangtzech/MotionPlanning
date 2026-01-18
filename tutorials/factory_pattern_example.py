"""工厂模式使用示例"""

from pyauto.config.config import Config
from pyauto.control.controller_factory import ControllerFactory

# ============ 示例1：基本使用 ============
config = Config()

# 旧方式（硬编码 if-elif）
# if name == "stanley":
#     controller = StanleyController(config)
# elif name == "purepursuit":
#     controller = PurePursuitController(config)
# ...

# 新方式（工厂模式）
controller = ControllerFactory.create("stanley", config)
print(f"创建控制器: {controller.__class__.__name__}")


# ============ 示例2：列出所有可用控制器 ============
available_controllers = ControllerFactory.list_available()
print(f"\n可用控制器: {', '.join(available_controllers)}")


# ============ 示例3：动态创建多个控制器 ============
controller_names = ["purepursuit", "stanley", "lqr_kinematic"]
controllers = [ControllerFactory.create(name, config) for name in controller_names]

for ctrl in controllers:
    print(f"- {ctrl.__class__.__name__}")


# ============ 示例4：运行时扩展（注册新控制器）============
from pyauto.control.controller_base import ControlCommand, ControllerBase  # noqa: E402
from pyauto.control.path_structs import PATH, Node  # noqa: E402


class MyCustomController(ControllerBase):
    """自定义控制器示例"""

    def ComputeControlCommand(self, node: Node, reference: PATH) -> ControlCommand:
        # 实现自定义逻辑
        return ControlCommand(steer=0.0)


# 注册自定义控制器
ControllerFactory.register("my_custom", MyCustomController)

# 现在可以像使用内置控制器一样使用它
custom_controller = ControllerFactory.create("my_custom", config)
print(f"\n自定义控制器: {custom_controller.__class__.__name__}")


# ============ 示例5：错误处理 ============
try:
    unknown_controller = ControllerFactory.create("nonexistent", config)
except ValueError as e:
    print(f"\n错误提示: {e}")
