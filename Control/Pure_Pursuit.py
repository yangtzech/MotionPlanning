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
from pid_speed_control import PIDSpeedController

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../MotionPlanning/")
from config_control import Config
from path_structs import PATH, Node, Nodes
from path_tools import generate_path

import Control.draw as draw
import CurvesGenerator.reeds_shepp as rs

# 实例化 config，只在主程序创建一次
config = Config()


# --- 统一接口的PurePursuit控制器类 ---
class PurePursuitController(ControllerBase):
    def __init__(self, config: Config):
        super().__init__(config)
        self.config = config

    def ComputeControlCommand(self, node: Node, reference: PATH) -> Dict[str, Any]:
        """
        pure pursuit controller
        :param node: current information
        :param reference: reference path: x, y, yaw, curvature
        :return: optimal ControlCommand {'steer':..., 'target_ind':...}
        """
        ref_path = reference
        ind, Lf = ref_path.target_index(node)
        tx = ref_path.cx[ind]
        ty = ref_path.cy[ind]

        alpha = math.atan2(ty - node.y, tx - node.x) - node.yaw
        delta = math.atan2(2.0 * self.config.WB * math.sin(alpha), Lf)

        return {
            "steer": delta,
            "target_ind": ind,
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

    x, y, yaw, direct, path_x, path_y = generate_path(
        states, config.MAX_STEER, config.WB
    )

    maxTime = 10.0
    yaw_old = 0.0
    x0, y0, yaw0, direct0 = x[0][0], y[0][0], yaw[0][0], direct[0][0]
    x_rec, y_rec = [], []
    controller = PurePursuitController(config)
    speed_controller = PIDSpeedController(config)

    for cx, cy, cyaw, cdirect in zip(x, y, yaw, direct):
        t = 0.0
        node = Node(x=x0, y=y0, yaw=yaw0, v=0.0, direct=direct0, config=config)
        nodes = Nodes()
        nodes.add(t, node)
        cv = [config.MAX_SPEED] * len(cx)
        ref_trajectory = PATH(cx, cy, config, cv)

        while t <= maxTime:
            target_ind, _ = ref_trajectory.target_index(node)
            acceleration = speed_controller.ComputeControlCommand(node, ref_trajectory)
            output = controller.ComputeControlCommand(node, ref_trajectory)
            delta = output["steer"]

            t += config.dt

            node.update(acceleration, delta, cdirect[0])
            nodes.add(t, node)
            x_rec.append(node.x)
            y_rec.append(node.y)

            v_safe = max(abs(node.v), 1e-3)
            dy = (node.yaw - yaw_old) / (v_safe * config.dt)
            steer = rs.pi_2_pi(-math.atan(config.WB * dy))

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
            draw.draw_car(node.x, node.y, yaw_old, steer, config)

            plt.axis("equal")
            plt.title("PurePursuit: v=" + str(node.v * 3.6)[:4] + "km/h")
            plt.gcf().canvas.mpl_connect(
                "key_release_event",
                lambda event: [exit(0) if event.key == "escape" else None],
            )
            plt.pause(0.001)

    plt.show()


if __name__ == "__main__":
    main()
