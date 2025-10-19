import math
import os
import sys

import matplotlib.pyplot as plt
from pid_speed_control import PIDSpeedController
from Pure_Pursuit import PurePursuitController

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../MotionPlanning/")

from config_control import Config
from path_structs import PATH, Node, Nodes
from path_tools import generate_path

import Control.draw as draw
import CurvesGenerator.reeds_shepp as rs


def run_simulation(config: Config, states):
    x, y, yaw, direct, path_x, path_y = generate_path(
        states, config.MAX_STEER, config.WB
    )

    lat_controller = PurePursuitController(config)
    lon_controller = PIDSpeedController(config)

    for cx, cy, cyaw, cdirect in zip(x, y, yaw, direct):
        t = 0.0
        maxTime = len(cx) * config.dt
        print(maxTime)
        yaw_old = cyaw[0]
        x0, y0, yaw0, direct0 = cx[0], cy[0], cyaw[0], cdirect[0]
        node = Node(x=x0, y=y0, yaw=yaw0, v=0.0, direct=direct0, config=config)
        nodes = Nodes()
        nodes.add(t, node)
        x_rec, y_rec = [], []

        cv = [config.MAX_SPEED] * len(cx)
        ref_trajectory = PATH(cx, cy, config, cv)

        while t <= maxTime:
            target_ind, _ = ref_trajectory.target_index(node)

            lat_output = lat_controller.ComputeControlCommand(node, ref_trajectory)
            delta = lat_output.steer

            lon_out = lon_controller.ComputeControlCommand(node, ref_trajectory)
            acceleration = lon_out.acceleration

            t += config.dt

            node.update(acceleration, delta, cdirect[0])
            nodes.add(t, node)
            x_rec.append(node.x)
            y_rec.append(node.y)

            v_safe = max(abs(node.v), 1e-3)
            dy = (node.yaw - yaw_old) / (v_safe * config.dt)
            steer = rs.pi_2_pi(-math.atan(config.WB * dy))

            yaw_old = node.yaw

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


def main():
    config = Config()
    states = [
        (0, 0, 0),
        (20, 15, 0),
        (35, 20, 90),
        (40, 0, 180),
        (20, 0, 120),
        (5, -10, 180),
        (15, 5, 30),
    ]
    run_simulation(config, states)


if __name__ == "__main__":
    main()
