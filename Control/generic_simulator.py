"""通用模拟运行器，用于运行各种控制器的仿真"""

import argparse

from config_control import Config
from logger import log_controller_info
from pid_speed_control import PIDSpeedController
from simulation_common import run_simulation
from simulation_runner_configurable import plot_tracking_results


def get_controller_class(controller_name):
    """根据控制器名称获取对应的控制器类"""
    # 导入所有控制器类
    from LQR_Dynamic_Model import LQRDynamicController
    from LQR_Kinematic_Model import LQRKinematicController
    from MPC_Frenet_Frame import MPC_Frenet_FrameController
    from MPC_XY_Frame import MPC_XY_FrameController
    from Pure_Pursuit import PurePursuitController
    from Rear_Wheel_Feedback import RearWheelFeedbackController
    from Stanley import StanleyController

    controllers = {
        "LQRDynamic": LQRDynamicController,
        "LQRKinematic": LQRKinematicController,
        "MPC_Frenet_Frame": MPC_Frenet_FrameController,
        "MPC_XY_Frame": MPC_XY_FrameController,
        "PurePursuit": PurePursuitController,
        "RearWheelFeedback": RearWheelFeedbackController,
        "Stanley": StanleyController,
    }

    if controller_name not in controllers:
        raise ValueError(
            f"Unknown controller: {controller_name}. Available: {list(controllers.keys())}"
        )

    return controllers[controller_name]


def is_mpc_controller(controller_name):
    """判断是否为MPC控制器（需要特殊的仿真参数）"""
    mpc_controllers = ["MPC_Frenet_Frame", "MPC_XY_Frame"]
    return controller_name in mpc_controllers


def run_single_simulation(
    controller_name, config_override=None, show_realtime_performance=False
):
    """运行单个控制器的仿真"""
    config = Config()

    # 应用配置覆盖
    if config_override:
        for attr, value in config_override.items():
            if hasattr(config, attr):
                setattr(config, attr, value)

    states = [
        (0, 0, 0),
        (20, 15, 0),
        (35, 20, 90),
        (40, 0, 180),
        (20, 0, 120),
        (5, -10, 180),
        (15, 5, 30),
    ]

    controller_class = get_controller_class(controller_name)
    controller = controller_class(config)

    # 根据控制器类型选择仿真参数
    if is_mpc_controller(controller_name):
        # MPC控制器使用独立控制模式
        result = run_simulation(
            config,
            states,
            controller,
            lon_controller=None,
            show_realtime_performance=show_realtime_performance,
            controller_name=controller_name,
            lat_lon_separate=False,
        )
    else:
        # 其他控制器使用PID速度控制器
        lon_controller = PIDSpeedController(config)
        result = run_simulation(
            config,
            states,
            controller,
            lon_controller,
            show_realtime_performance=show_realtime_performance,
            controller_name=controller_name,
        )

    result["controller"] = controller_name
    return result


def run_comparison_simulation(controller_names, config_override=None):
    """运行多个控制器的对比仿真"""
    results = []
    for controller_name in controller_names:
        log_controller_info(f"Running simulation for {controller_name}...")
        result = run_single_simulation(controller_name, config_override)
        results.append(result)
    return results


def main():
    parser = argparse.ArgumentParser(description="Generic Path Tracking Simulation")
    parser.add_argument(
        "--controller",
        type=str,
        nargs="+",
        required=True,
        help="Controller type(s), e.g. --controller PurePursuit LQRDynamic",
    )
    parser.add_argument(
        "--max_speed", type=float, default=None, help="Override max speed (m/s)"
    )
    parser.add_argument("--dt", type=float, default=None, help="Override time step (s)")
    parser.add_argument(
        "--max_time",
        type=float,
        default=10.0,
        help="Max simulation time per segment (s)",
    )

    args = parser.parse_args()

    # 准备配置覆盖
    config_override = {}
    if args.max_speed is not None:
        config_override["MAX_SPEED"] = args.max_speed
    if args.dt is not None:
        config_override["dt"] = args.dt
    if args.max_time != 10.0:
        config_override["time_max"] = args.max_time

    # 运行仿真
    results = run_comparison_simulation(args.controller, config_override)

    # 绘制结果
    plot_tracking_results(results)


if __name__ == "__main__":
    main()
