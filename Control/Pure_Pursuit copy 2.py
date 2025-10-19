"""
Pure Pursuit
author: huiming zhou
"""

import math
import os
import sys
from typing import Any, Dict

import matplotlib.pyplot as plt
from controller_base import ControllerBase

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../MotionPlanning/")
from config_control import Config
from path_structs import PATH, Node, Nodes
from path_tools import generate_path

import Control.draw as draw
import CurvesGenerator.reeds_shepp as rs

C = Config()


def pid_control(target_v, v, dist, direct):
    """
    PID controller and design speed profile.
    :param target_v: target speed (forward and backward are different)
    :param v: current speed
    :param dist: distance from current position to end position
    :param direct: current direction
    :return: desired acceleration
    """

    a = 0.3 * (target_v - direct * v)

    if dist < 10.0:
        if v > 3.0:
            a = -2.5
        elif v < -2.0:
            a = -1.0

    return a


# --- 统一接口的PurePursuit控制器类 ---
class PurePursuitController(ControllerBase):
    def __init__(self, config: Dict[str, Any] = None):
        super().__init__(config)
        self.target_ind = None
        self.ref_path = None

    def set_reference(self, cx, cy):
        self.ref_path = PATH(cx, cy)
        self.target_ind = None

    def ComputeControlCommand(
        self, node: Node, reference: Dict[str, Any]
    ) -> Dict[str, Any]:
        """
        pure pursuit controller
        :param node: current information
        :param reference: reference path: x, y, yaw, curvature
        :return: optimal ControlCommand {'acceleration':..., 'steer':..., 'target_ind':...}
        """

        self.set_reference(reference["cx"], reference["cy"])
        self.target_ind, _ = self.ref_path.target_index(node, C)
        # 控制参数
        target_speed = reference.get("target_speed", 30.0 / 3.6)
        C.Ld = reference.get("Ld", C.Ld)
        C.dist_stop = reference.get("dist_stop", C.dist_stop)
        C.dc = reference.get("dc", C.dc)

        # 计算距离
        xt = node.x + C.dc * math.cos(node.yaw)
        yt = node.y + C.dc * math.sin(node.yaw)
        dist = math.hypot(xt - reference["cx"][-1], yt - reference["cy"][-1])
        # 速度控制
        acceleration = pid_control(target_speed, node.v, dist, node.direct)
        # 纯跟踪控制
        ind, Lf = self.ref_path.target_index(
            node, C
        )  # target point and pursuit distance
        ind = max(ind, self.target_ind)

        tx = self.ref_path.cx[ind]
        ty = self.ref_path.cy[ind]

        alpha = math.atan2(ty - node.y, tx - node.x) - node.yaw
        delta = math.atan2(2.0 * C.WB * math.sin(alpha), Lf)

        self.target_ind = ind

        return {
            "acceleration": acceleration,
            "steer": delta,
            "target_ind": self.target_ind,
        }


def main():
    states = [
        (0, 0, 0),
        (20, 15, 0),
        (35, 20, 90),
        (40, 0, 180),
        (20, 0, 120),
        (5, -10, 180),
        (15, 5, 30),
    ]

    # states = [(-3, 3, 120), (10, -7, 30), (10, 13, 30), (20, 5, -25),
    #           (35, 10, 180), (30, -10, 160), (5, -12, 90)]

    x, y, yaw, direct, path_x, path_y = generate_path(states, C.MAX_STEER, C.WB)

    # simulation
    maxTime = 10.0
    yaw_old = 0.0
    x0, y0, yaw0, direct0 = x[0][0], y[0][0], yaw[0][0], direct[0][0]
    x_rec, y_rec = [], []
    controller = PurePursuitController()

    for cx, cy, cyaw, cdirect in zip(x, y, yaw, direct):
        t = 0.0
        node = Node(x=x0, y=y0, yaw=yaw0, v=0.0, direct=direct0)
        nodes = Nodes()
        nodes.add(t, node)
        ref_trajectory = PATH(cx, cy)
        target_ind, _ = ref_trajectory.target_index(node, C)
        last_ind = len(cx)

        while t <= maxTime and target_ind < last_ind:

            reference = {"cx": cx, "cy": cy}
            output = controller.ComputeControlCommand(node, reference)
            acceleration = output["acceleration"]
            delta = output["steer"]
            target_ind = output["target_ind"]

            t += C.dt

            node.update(acceleration, delta, cdirect[0], C)
            nodes.add(t, node)
            x_rec.append(node.x)
            y_rec.append(node.y)

            dy = (node.yaw - yaw_old) / (node.v * C.dt)
            steer = rs.pi_2_pi(-math.atan(C.WB * dy))

            yaw_old = node.yaw
            x0 = nodes.x[-1]
            y0 = nodes.y[-1]
            yaw0 = nodes.yaw[-1]
            direct0 = nodes.direct[-1]

            # animation
            plt.cla()
            plt.plot(path_x, path_y, color="gray", linewidth=2)
            plt.plot(x_rec, y_rec, color="darkviolet", linewidth=2)
            plt.plot(cx[target_ind], cy[target_ind], ".r")
            draw.draw_car(node.x, node.y, yaw_old, steer, C)

            plt.axis("equal")
            plt.title("PurePursuit: v=" + str(node.v * 3.6)[:4] + "km/h")
            plt.gcf().canvas.mpl_connect(
                "key_release_event",
                lambda event: [exit(0) if event.key == "escape" else None],
            )
            plt.pause(0.001)

    plt.show()


# 仅供测试
if __name__ == "__main__":
    main()
