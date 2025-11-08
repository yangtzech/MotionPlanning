import math

import matplotlib.pyplot as plt
from path_structs import PATH, Node, Nodes
from path_tools import generate_path
from utils import pi_2_pi

import Control.draw as draw

is_paused = False


def on_key(event):
    global is_paused
    if event.key == " ":
        is_paused = not is_paused
    if event.key == "escape":
        exit(0)


def run_simulation(
    config,
    states,
    lat_controller,
    lon_controller,
    draw_car_func=draw.draw_car,
    show_animation=True,
):
    """
    Run path tracking simulation.
    Args:
        config: Config object
        states: list of (x, y, yaw) tuples
        lat_controller: lateral controller instance (必须有 ComputeControlCommand 方法)
        lon_controller: longitudinal controller instance (必须有 ComputeControlCommand 方法)
        draw_car_func: function for drawing car (optional)
        show_animation: whether to show animation
    Returns:
        dict with all_time, all_v_actual, all_v_ref, all_lat_error, all_yaw_error
    """
    x, y, yaw, cur, direct, path_x, path_y = generate_path(
        states, config.MAX_STEER, config.WB
    )
    # 合并所有段的分析数据
    all_time = []
    all_v_actual = []
    all_v_ref = []
    all_lat_error = []
    all_yaw_error = []
    seg_time_offset = 0.0

    event_bound = False
    for seg_id, (cx, cy, cyaw, ccurv, cdirect) in enumerate(
        zip(x, y, yaw, cur, direct)
    ):
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

        ref_trajectory = PATH(cx, cy, cyaw, ccurv, cdirect, config, cv)

        # 结果分析数据
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
            if node.v == 0:
                v_safe = 1e-3
            else:
                v_safe = max(abs(node.v), 1e-3) * math.copysign(1, node.v)
            dy = (node.yaw - yaw_old) / (v_safe * config.dt)
            steer = pi_2_pi(-math.atan(config.WB * dy))

            yaw_old = node.yaw

            # 记录分析数据
            v_actual.append(node.v * node.direct)
            v_ref.append(ref_trajectory.cv[lon_out.target_ind])
            # 横向误差
            lat_error.append(lat_output.lat_error)
            # 航向误差
            yaw_error.append(lat_output.yaw_error)
            time_list.append(t)

            # animation
            if show_animation:
                plt.cla()
                plt.plot(path_x, path_y, color="gray", linewidth=2)
                plt.plot(x_rec, y_rec, color="darkviolet", linewidth=2)
                plt.plot(cx[target_ind], cy[target_ind], ".r")
                draw_car_func(node.x, node.y, yaw_old, steer, config)

                plt.axis("equal")
                plt.title("v=" + str(node.v * 3.6)[:4] + "km/h")
                if not event_bound:
                    plt.gcf().canvas.mpl_connect("key_release_event", on_key)
                    event_bound = True
                plt.pause(0.001)
                if is_paused:
                    while is_paused:
                        plt.pause(0.1)

        # 合并分析数据
        all_time.append([t + seg_time_offset for t in time_list])
        all_v_actual.append([v * 3.6 for v in v_actual])
        all_v_ref.append([v * 3.6 for v in v_ref])
        all_lat_error.append(lat_error)
        all_yaw_error.append(yaw_error)
        if time_list:
            seg_time_offset += time_list[-1]

    # 返回仿真数据，绘图由外部处理
    return {
        "all_time": all_time,
        "all_v_actual": all_v_actual,
        "all_v_ref": all_v_ref,
        "all_lat_error": all_lat_error,
        "all_yaw_error": all_yaw_error,
    }
