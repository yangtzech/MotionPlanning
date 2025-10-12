"""
适配器：将现有 Control 下的算法适配到统一接口
"""

from typing import Any, Dict

from .controller_base import RefPath, VehicleState
from .controller_factory import register_controller


@register_controller("pure_pursuit")
def pure_pursuit_adapter(state: VehicleState, ref_path: RefPath) -> Dict[str, Any]:
    from Control.Pure_Pursuit import PATH, Node, pure_pursuit

    node = Node(state.x, state.y, state.yaw, state.v, state.direct)
    path = PATH(ref_path.cx, ref_path.cy)
    steer, idx = pure_pursuit(node, path, getattr(path, "index_old", 0) or 0)

    return {"steer": steer, "accel": 0.0, "info": {"target_index": idx}}


@register_controller("stanley")
def stanley_adapter(state: VehicleState, ref_path: RefPath) -> Dict[str, Any]:
    from Control.Stanley import Node as SNode
    from Control.Stanley import Trajectory, front_wheel_feedback_control

    node = SNode(x=state.x, y=state.y, yaw=state.yaw, v=state.v)
    traj = Trajectory(ref_path.cx, ref_path.cy, ref_path.cyaw)
    steer, idx = front_wheel_feedback_control(node, traj)

    return {"steer": steer, "accel": 0.0, "info": {"target_index": idx}}


@register_controller("rear_wheel")
def rear_wheel_adapter(state: VehicleState, ref_path: RefPath) -> Dict[str, Any]:
    from Control.Rear_Wheel_Feedback import PATH
    from Control.Rear_Wheel_Feedback import Node as RNode
    from Control.Rear_Wheel_Feedback import rear_wheel_feedback_control

    node = RNode(x=state.x, y=state.y, yaw=state.yaw, v=state.v, direct=state.direct)
    traj = PATH(
        ref_path.cx,
        ref_path.cy,
        ref_path.cyaw,
        getattr(ref_path, "ck", [0] * len(ref_path.cx)),
    )
    steer, idx = rear_wheel_feedback_control(node, traj)

    return {"steer": steer, "accel": 0.0, "info": {"target_index": idx}}
    traj = PATH(
        ref_path.cx,
        ref_path.cy,
        ref_path.cyaw,
        getattr(ref_path, "ck", [0] * len(ref_path.cx)),
    )
    steer, idx = rear_wheel_feedback_control(node, traj)

    return {"steer": steer, "accel": 0.0, "info": {"target_index": idx}}
    steer, idx = rear_wheel_feedback_control(node, traj)

    return {"steer": steer, "accel": 0.0, "info": {"target_index": idx}}
