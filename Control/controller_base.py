"""
统一控制器接口与基本数据结构
"""

from typing import Protocol, Sequence


class VehicleState:
    def __init__(self, x: float, y: float, yaw: float, v: float, direct: float = 1.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.direct = direct


class RefPath(Protocol):
    cx: Sequence[float]
    cy: Sequence[float]
    cyaw: Sequence[float]
    # optional: curvature, etc.


class Controller(Protocol):
    """统一控制器接口

    compute(state, ref_path) -> (control, extra)
    - control: dict 含至少 keys: 'steer' (rad) 和 'accel' (m/s^2)
    - extra: 任意附加信息 (例如 target_index)
    """

    def compute(self, state: VehicleState, ref_path: RefPath):
        ...
        ...
        ...
