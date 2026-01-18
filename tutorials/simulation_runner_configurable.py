import argparse

import matplotlib.pyplot as plt

from pyauto.config.config import Config
from pyauto.control.pid_speed_control import PIDSpeedController
from pyauto.control.Pure_Pursuit import PurePursuitController
from pyauto.control.Rear_Wheel_Feedback import RearWheelFeedbackController
from pyauto.control.Stanley import StanleyController
from pyauto.simulation.simulation_common import run_simulation


def get_lat_controller(name, config):
    if name.lower() == "purepursuit":
        return PurePursuitController(config)
    elif name.lower() == "stanley":
        return StanleyController(config)
    elif name.lower() == "rearwheelfeedback":
        return RearWheelFeedbackController(config)
    else:
        raise ValueError(f"Unknown lateral controller: {name}")


def plot_tracking_results(results):
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
    for idx, result in enumerate(results):
        c = colors[idx % len(colors)]
        for seg_id in range(len(result["all_time"])):
            label_prefix = f"{result['controller']} S{seg_id + 1}"
            axs[0].plot(
                result["all_time"][seg_id],
                result["all_v_actual"][seg_id],
                label=f"{label_prefix} Actual Speed",
                color=c,
                linestyle="-",
            )
            axs[0].plot(
                result["all_time"][seg_id],
                result["all_v_ref"][seg_id],
                label=f"{label_prefix} Ref Speed",
                color=c,
                linestyle="--",
            )
            axs[1].plot(
                result["all_time"][seg_id],
                result["all_lat_error"][seg_id],
                label=f"{label_prefix} Lat Error",
                color=c,
            )
            axs[2].plot(
                result["all_time"][seg_id],
                result["all_yaw_error"][seg_id],
                label=f"{label_prefix} Yaw Error",
                color=c,
            )
    axs[0].set_ylabel("Speed (km/h)")
    axs[0].legend(loc="best", fontsize=8)
    axs[0].grid(True, linestyle="--", alpha=0.5)
    axs[1].set_ylabel("Latitude Error (m)")
    axs[1].legend(loc="best", fontsize=8)
    axs[1].grid(True, linestyle="--", alpha=0.5)
    axs[2].set_ylabel("Heading Error (rad)")
    axs[2].set_xlabel("Time (s)")
    axs[2].legend(loc="best", fontsize=8)
    axs[2].grid(True, linestyle="--", alpha=0.5)
    plt.suptitle("Controllers Tracking Analysis Comparison", fontsize=14)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Configurable Path Tracking Simulation")
    parser.add_argument(
        "--controller",
        type=str,
        nargs="+",
        default=["PurePursuit"],
        choices=["PurePursuit", "Stanley", "RearWheelFeedback"],
        help="Lateral controller type(s), e.g. --controller PurePursuit Stanley",
    )
    parser.add_argument("--max_speed", type=float, default=None, help="Override max speed (m/s)")
    parser.add_argument("--dt", type=float, default=None, help="Override time step (s)")
    parser.add_argument(
        "--max_time",
        type=float,
        default=10.0,
        help="Max simulation time per segment (s)",
    )
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
    results = []
    for ctrl_name in args.controller:
        lat_controller = get_lat_controller(ctrl_name, config)
        lon_controller = PIDSpeedController(config)
        sim_result = run_simulation(config, states, lat_controller, lon_controller, show_animation=False)
        sim_result["controller"] = ctrl_name
        results.append(sim_result)
    plot_tracking_results(results)


if __name__ == "__main__":
    main()
