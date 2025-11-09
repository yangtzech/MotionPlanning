统一 Control 接口说明

新增文件：
- `controller_base.py`：定义 `VehicleState`, `RefPath` 协议与 `Controller` 抽象接口。
- `controller_factory.py`：控制器注册与获取（装饰器式注册）。
- `controllers_adapters.py`：把现有的 `Pure_Pursuit`, `Stanley`, `Rear_Wheel_Feedback` 适配为统一接口。
- `example_runner.py`：单步调用示例（smoke test）。

使用方法：
1. 在代码中导入并通过 `controller_factory.get_controller(name)` 获取适配函数。
2. 调用适配函数，传入 `controller_base.VehicleState` 和 简单的 `ref_path`（需包含 `cx`, `cy`, `cyaw`）。

示例：
```
from Control.controller_factory import get_controller
from Control.controller_base import VehicleState

fn = get_controller('pure_pursuit')
state = VehicleState(0, 0, 0, 0)
class RP: cx=[0,1]; cy=[0,0]; cyaw=[0,0]
out = fn(state, RP())
```

说明：当前适配器以最小侵入方式实现，尽量不修改原有控制器文件。后续可以为 LQR/MPC 实现更完整的适配器（包含 speed/accel 输出）。
