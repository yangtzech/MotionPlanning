import argparse
import math
import os
import sys

import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../MotionPlanning/")

from config_control import Config
from path_structs import PATH, Node, Nodes
from path_tools import generate_path
from pid_speed_control import PIDSpeedController
from Pure_Pursuit import PurePursuitController
from Stanley import StanleyController
from utils import pi_2_pi

import Control.draw as draw

is_paused = False


def on_key(event):
    global is_paused
    if event.key == " ":
        is_paused = not is_paused
    if event.key == "escape":
        exit(0)


def get_lat_controller(name, config):
    if name.lower() == "purepursuit":
        return PurePursuitController(config)
    elif name.lower() == "stanley":
        return StanleyController(config)
    else:
        raise ValueError(f"Unknown lateral controller: {name}")


def run_simulation_multi(config: Config, states, lat_controller_names):
    x, y, yaw, direct, path_x, path_y = generate_path(
        states, config.MAX_STEER, config.WB
    )

    results = []
    for lat_controller_name in lat_controller_names:
        lat_controller = get_lat_controller(lat_controller_name, config)
        lon_controller = PIDSpeedController(config)

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
            ref_trajectory = PATH(cx, cy, cyaw, cdirect, config, cv)
            v_actual = []
            v_ref = []
            lat_error = []
            yaw_error = []
            time_list = []
            while t <= maxTime:
                lat_output = lat_controller.ComputeControlCommand(node, ref_trajectory)
                delta = lat_output.steer
                target_ind = lat_output.target_ind
                if target_ind >= lastIndex:
                    break
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
                v_actual.append(node.v * node.direct)
                v_ref.append(ref_trajectory.cv[lon_out.target_ind])
                dx = node.x - cx[target_ind]
                dy_lat = node.y - cy[target_ind]
                lat_error.append(math.hypot(dx, dy_lat))
                yaw_e = pi_2_pi(node.yaw - cyaw[target_ind])
                yaw_error.append(yaw_e)
                time_list.append(t)
            all_time.append([t + seg_time_offset for t in time_list])
            all_v_actual.append([v * 3.6 for v in v_actual])
            all_v_ref.append([v * 3.6 for v in v_ref])
            all_lat_error.append(lat_error)
            all_yaw_error.append(yaw_error)
            if time_list:
                seg_time_offset += time_list[-1]
        results.append(
            {
                "controller": lat_controller_name,
                "all_time": all_time,
                "all_v_actual": all_v_actual,
                "all_v_ref": all_v_ref,
                "all_lat_error": all_lat_error,
                "all_yaw_error": all_yaw_error,
            }
        )
    # 绘图对比
    fig, axs = plt.subplots(3, 1, figsize=(8, 10))
    color_list = [
        "tab:blue",
        "tab:orange",
        "tab:green",
        "tab:red",
        "tab:purple",
        "tab:brown",
        "tab:pink",
        "tab:gray",
    ]
    for idx, result in enumerate(results):
        c = color_list[idx % len(color_list)]
        for seg_id in range(len(result["all_time"])):
            label_prefix = f"{result['controller']} S{seg_id+1}"
            axs[0].plot(
                result["all_time"][seg_id],
                result["all_v_actual"][seg_id],
                # label=f"{label_prefix} Actual Speed",
                color=c,
                linestyle="-",
            )
            axs[0].plot(
                result["all_time"][seg_id],
                result["all_v_ref"][seg_id],
                # label=f"{label_prefix} Ref Speed",
                color=c,
                linestyle=":",
            )
            axs[1].plot(
                result["all_time"][seg_id],
                result["all_lat_error"][seg_id],
                # label=f"{label_prefix} Lat Error",
                color=c,
            )
            axs[2].plot(
                result["all_time"][seg_id],
                result["all_yaw_error"][seg_id],
                # label=f"{label_prefix} Yaw Error",
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
    plt.suptitle("Controllers Tracking Analysis Comparison")
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Configurable Path Tracking Simulation"
    )
    parser.add_argument(
        "--controller",
        type=str,
        nargs="+",
        default=["PurePursuit"],
        choices=["PurePursuit", "Stanley"],
        help="Lateral controller type(s), e.g. --controller PurePursuit Stanley",
    )
    parser.add_argument(
        "--max_speed", type=float, default=None, help="Override max speed (m/s)"
    )
    parser.add_argument("--dt", type=float, default=None, help="Override time step (s)")
    args = parser.parse_args()
    config = Config()
    if args.max_speed is not None:
        config.MAX_SPEED = args.max_speed
    if args.dt is not None:
        config.dt = args.dt
    states = [
        (0, 0, 0),
        (20, 15, 0),
        (35, 20, 90),
        (40, 0, 180),
        (20, 0, 120),
        (5, -10, 180),
        (15, 5, 30),
    ]
    run_simulation_multi(config, states, args.controller)


if __name__ == "__main__":
    main()
