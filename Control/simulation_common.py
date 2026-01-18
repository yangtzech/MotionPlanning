import math

import matplotlib.pyplot as plt
from path_structs import PATH, Node, Nodes
from path_tools import generate_path
from utils import pi_2_pi

import Control.draw as draw
from Control.animation_updater import get_animator
from Control.performance_monitor import get_performance_monitor

is_paused = False


def on_key(event):
    global is_paused
    if event.key == " ":
        is_paused = not is_paused
    if event.key == "escape":
        exit(0)


def _initialize_simulation_components(
    show_animation, path_x, path_y, config, show_realtime_performance, controller_name
):
    """初始化动画和性能监控组件"""
    animator = get_animator(path_x, path_y, config) if show_animation else None
    perf_monitor = get_performance_monitor(controller_name) if show_realtime_performance else None
    return animator, perf_monitor


def _initialize_segment_variables(cx, cy, cyaw, ccurv, cdirect, config):
    """初始化单个段的变量"""
    t = 0.0
    maxTime = config.time_max
    yaw_old = cyaw[0]
    x0, y0, yaw0, direct0 = cx[0], cy[0], cyaw[0], cdirect[0]
    node = Node(x=x0, y=y0, yaw=yaw0, v=0.1, direct=direct0, config=config)
    nodes = Nodes()
    nodes.add(t, node)
    x_rec, y_rec = [], []
    lastIndex = len(cx) - 1

    cv = [config.target_speed * d for d in cdirect]
    ref_trajectory = PATH(cx, cy, cyaw, ccurv, cdirect, config, cv)

    # 结果分析数据
    v_actual = []
    v_ref = []
    lat_error = []
    yaw_error = []
    time_list = []

    return (
        t,
        maxTime,
        yaw_old,
        node,
        nodes,
        x_rec,
        y_rec,
        lastIndex,
        ref_trajectory,
        v_actual,
        v_ref,
        lat_error,
        yaw_error,
        time_list,
    )


def _process_single_step(
    node, lat_controller, lon_controller, ref_trajectory, config, lat_lon_separate, cdirect, yaw_old
):
    """处理单步仿真"""
    lat_output = lat_controller.ComputeControlCommand(node, ref_trajectory)
    delta = lat_output.steer
    ed = lat_output.lat_error
    e_phi = lat_output.yaw_error
    target_ind = lat_output.target_ind

    if lat_lon_separate:
        lon_out = lon_controller.ComputeControlCommand(node, ref_trajectory)
        acceleration = lon_out.acceleration
    else:
        acceleration = lat_output.acceleration

    return delta, ed, e_phi, target_ind, acceleration


def _calculate_steering(node, yaw_old, config):
    """计算转向角"""
    v_safe = max(abs(node.v), 1e-3)
    if node.v == 0:
        v_safe = 1e-3
    else:
        v_safe = max(abs(node.v), 1e-3) * math.copysign(1, node.v)
    dy = (node.yaw - yaw_old) / (v_safe * config.dt)
    steer = pi_2_pi(-math.atan(config.WB * dy))

    return steer


def _handle_animation_and_pause(animator, node, cx, cy, target_ind, steer):
    """处理动画和暂停逻辑"""
    should_continue = True
    if animator:
        should_continue = animator.update_frame(node, cx, cy, target_ind, steer)
        if should_continue:
            plt.pause(0.001)
            if animator.is_paused:
                while animator.is_paused:
                    plt.pause(0.1)
            if animator.should_exit:
                print("Simulation stopped by user.")
                should_continue = False
    return should_continue


def _handle_performance_monitor(perf_monitor, t, seg_time_offset, v_actual_value, v_ref_value, ed, e_phi):
    """处理性能监控"""
    should_continue = True
    if perf_monitor:
        should_continue = perf_monitor.update_data(
            t + seg_time_offset,  # 使用段内时间
            v_actual_value * 3.6,  # 转换为 km/h
            v_ref_value * 3.6,  # 转换为 km/h
            ed,
            e_phi,
        )
    return should_continue


def _simulate_single_segment(
    seg_id,
    cx,
    cy,
    cyaw,
    ccurv,
    cdirect,
    config,
    lat_controller,
    lon_controller,
    animator,
    perf_monitor,
    show_animation,
    show_realtime_performance,
    seg_time_offset,
    lat_lon_separate,
):
    """模拟单个段"""
    (
        t,
        maxTime,
        yaw_old,
        node,
        nodes,
        x_rec,
        y_rec,
        lastIndex,
        ref_trajectory,
        v_actual,
        v_ref,
        lat_error,
        yaw_error,
        time_list,
    ) = _initialize_segment_variables(cx, cy, cyaw, ccurv, cdirect, config)

    while t <= maxTime:
        delta, ed, e_phi, target_ind, acceleration = _process_single_step(
            node, lat_controller, lon_controller, ref_trajectory, config, lat_lon_separate, cdirect, yaw_old
        )

        if target_ind >= lastIndex:
            break

        t += config.dt
        node.update(acceleration, delta, cdirect[0], ed, e_phi)
        nodes.add(t, node)
        x_rec.append(node.x)
        y_rec.append(node.y)

        steer = _calculate_steering(node, yaw_old, config)
        yaw_old = node.yaw

        # 记录分析数据
        v_actual_value = node.v * node.direct
        v_actual.append(v_actual_value)
        v_ref_value = ref_trajectory.cv[target_ind]
        v_ref.append(v_ref_value)
        lat_error.append(ed)
        yaw_error.append(e_phi)
        time_list.append(t)

        # 实时性能监控（支持多段轨迹显示）
        if show_realtime_performance:
            should_continue = _handle_performance_monitor(
                perf_monitor, t, seg_time_offset, v_actual_value, v_ref_value, ed, e_phi
            )
            if not should_continue:
                break  # 如果性能监控器指示停止，则退出循环

        # animation - 使用新的动画更新器
        if show_animation:
            should_continue = _handle_animation_and_pause(animator, node, cx, cy, target_ind, steer)
            if not should_continue:
                break  # 如果动画更新器或用户指示停止，则退出循环

    # 返回分析数据和最终时间偏移
    return time_list, v_actual, v_ref, lat_error, yaw_error, time_list[-1] if time_list else 0


def run_simulation(
    config,
    states,
    lat_controller,
    lon_controller,
    draw_car_func=draw.draw_car,
    show_animation=True,
    show_realtime_performance=False,
    controller_name="Controller",
    lat_lon_separate=True,
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
        show_realtime_performance: whether to show real-time performance graphs (支持多段轨迹显示)
        controller_name: name of the controller for performance monitor
        lat_lon_separate: whether to use separate lateral and longitudinal controllers
    Returns:
        dict with all_time, all_v_actual, all_v_ref, all_lat_error, all_yaw_error
    """
    x, y, yaw, cur, direct, path_x, path_y = generate_path(states, config.MAX_STEER, config.WB)
    # 合并所有段的分析数据
    all_time = []
    all_v_actual = []
    all_v_ref = []
    all_lat_error = []
    all_yaw_error = []
    seg_time_offset = 0.0

    # 初始化动画和性能监控
    animator, perf_monitor = _initialize_simulation_components(
        show_animation, path_x, path_y, config, show_realtime_performance, controller_name
    )

    for seg_id, (cx, cy, cyaw, ccurv, cdirect) in enumerate(zip(x, y, yaw, cur, direct)):
        # 处理单个段
        (time_list, v_actual, v_ref, lat_error, yaw_error, segment_duration) = _simulate_single_segment(
            seg_id,
            cx,
            cy,
            cyaw,
            ccurv,
            cdirect,
            config,
            lat_controller,
            lon_controller,
            animator,
            perf_monitor,
            show_animation,
            show_realtime_performance,
            seg_time_offset,
            lat_lon_separate,
        )

        # 合并分析数据
        all_time.append([t + seg_time_offset for t in time_list])
        all_v_actual.append([v * 3.6 for v in v_actual])  # 转换为 km/h
        all_v_ref.append([v * 3.6 for v in v_ref])  # 转换为 km/h
        all_lat_error.append(lat_error)
        all_yaw_error.append(yaw_error)
        seg_time_offset += segment_duration

        # 实时性能监控：完成当前段
        if show_realtime_performance and perf_monitor:
            perf_monitor.finalize_current_segment()

    # 如果动画器存在，关闭它
    if show_animation and animator is not None:
        animator.close()

    # 如果性能监控器存在，关闭它
    if show_realtime_performance and perf_monitor is not None:
        perf_monitor.close()

    # 返回仿真数据，绘图由外部处理
    return {
        "all_time": all_time,
        "all_v_actual": all_v_actual,
        "all_v_ref": all_v_ref,
        "all_lat_error": all_lat_error,
        "all_yaw_error": all_yaw_error,
    }
