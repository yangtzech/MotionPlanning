import math
import os
import sys

import matplotlib.pyplot as plt
from pid_speed_control import PIDSpeedController
from Pure_Pursuit import PurePursuitController
from utils import pi_2_pi

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../MotionPlanning/")

from config_control import Config
from path_structs import PATH, Node, Nodes
from path_tools import generate_path

import Control.draw as draw

is_paused = False


def on_key(event):
    global is_paused
    if event.key == " ":
        is_paused = not is_paused
    if event.key == "escape":
        exit(0)


def run_simulation(config: Config, states):
    x, y, yaw, direct, path_x, path_y = generate_path(
        states, config.MAX_STEER, config.WB
    )

    lat_controller = PurePursuitController(config)
    lon_controller = PIDSpeedController(config)

    # 合并所有段的分析数据
    all_time = []
    all_v_actual = []
    all_v_ref = []
    all_lat_error = []
    all_yaw_error = []
    seg_time_offset = 0.0

    for seg_id, (cx, cy, cyaw, cdirect) in enumerate(zip(x, y, yaw, direct)):
        t = 0.0
        maxTime = 10.0
        yaw_old = cyaw[0]
        x0, y0, yaw0, direct0 = cx[0], cy[0], cyaw[0], cdirect[0]
        node = Node(x=x0, y=y0, yaw=yaw0, v=0.0, direct=direct0, config=config)
        nodes = Nodes()
        nodes.add(t, node)
        x_rec, y_rec = [], []
        lastIndex = len(cx) - 1

        cv = [config.MAX_SPEED] * len(cx)

        ref_trajectory = PATH(cx, cy, config, cv)

        # 结果分析数据
        v_actual = []
        v_ref = []
        lat_error = []
        yaw_error = []
        time_list = []

        while t <= maxTime:
            target_ind, _ = ref_trajectory.target_index(node)
            print(lastIndex)
            print(
                f"Target Index: {target_ind}, Node Position: ({node.x:.2f}, {node.y:.2f})"
            )
            if target_ind >= lastIndex:
                break

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
            steer = pi_2_pi(-math.atan(config.WB * dy))

            yaw_old = node.yaw

            # 记录分析数据
            v_actual.append(node.v * node.direct)
            v_ref.append(ref_trajectory.cv[target_ind])
            # 横向误差
            dx = node.x - cx[target_ind]
            dy_lat = node.y - cy[target_ind]
            lat_error.append(math.hypot(dx, dy_lat))
            # 航向误差
            yaw_e = pi_2_pi(node.yaw - cyaw[target_ind])
            yaw_error.append(yaw_e)
            time_list.append(t)

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
            if is_paused:
                plt.waitforbuttonpress()

        # 合并分析数据
        all_time.append([t + seg_time_offset for t in time_list])
        all_v_actual.append([v * 3.6 for v in v_actual])
        all_v_ref.append([v * 3.6 for v in v_ref])
        all_lat_error.append(lat_error)
        all_yaw_error.append(yaw_error)
        if time_list:
            seg_time_offset += time_list[-1]

    # 所有段结束后统一画分析曲线
    fig, axs = plt.subplots(3, 1, figsize=(8, 10))
    colors = [
        "tab:blue",
        "tab:orange",
        "tab:green",
        "tab:red",
        "tab:purple",
        "tab:brown",
        "tab:pink",
        "tab:gray",
    ]
    for seg_id in range(len(all_time)):
        c = colors[seg_id % len(colors)]
        axs[0].plot(
            all_time[seg_id],
            all_v_actual[seg_id],
            label=f"Actual Speed Segment{seg_id+1}",
            color=c,
        )
        axs[0].plot(
            all_time[seg_id],
            all_v_ref[seg_id],
            label=f"Reference Speed Segment{seg_id+1}",
            color=c,
            linestyle="--",
        )
        axs[1].plot(
            all_time[seg_id],
            all_lat_error[seg_id],
            label=f"Latitude Error Segment{seg_id+1}",
            color=c,
        )
        axs[2].plot(
            all_time[seg_id],
            all_yaw_error[seg_id],
            label=f"Heading Error Segment{seg_id+1}",
            color=c,
        )

    axs[0].set_ylabel("Speed (km/h)")
    axs[0].legend()
    axs[0].grid()

    axs[1].set_ylabel("Latitude Error (m)")
    axs[1].legend()
    axs[1].grid()

    axs[2].set_ylabel("Heading Error (rad)")
    axs[2].set_xlabel("Time (s)")
    axs[2].legend()
    axs[2].grid()

    plt.suptitle("All Trajectory Segments Tracking Analysis")
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
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
