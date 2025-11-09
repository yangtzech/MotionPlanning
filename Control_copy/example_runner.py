"""
示例：通过统一接口加载并调用控制器（单步演示）
"""

from .controller_base import VehicleState
from .controller_factory import get_controller, list_controllers


def demo_one_step(controller_name: str):
    print("Available controllers:", list_controllers())
    fn = get_controller(controller_name)

    # 构造一个简单的 straight-line reference path
    class SimpleRef:
        def __init__(self):
            self.cx = [0.0, 5.0, 10.0]
            self.cy = [0.0, 0.0, 0.0]
            self.cyaw = [0.0, 0.0, 0.0]
            self.ck = [0.0, 0.0, 0.0]

    ref = SimpleRef()

    state = VehicleState(x=0.0, y=-1.0, yaw=0.0, v=1.0)

    out = fn(state, ref)

    print(f"Controller={controller_name} -> output:", out)


if __name__ == "__main__":
    demo_one_step("pure_pursuit")
    demo_one_step("stanley")
    demo_one_step("rear_wheel")
