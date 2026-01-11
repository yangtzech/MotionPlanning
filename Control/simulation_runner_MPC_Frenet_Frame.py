from config_control import Config
from MPC_Frenet_Frame import MPC_Frenet_FrameController
from simulation_common import run_simulation
from simulation_runner_configurable import plot_tracking_results


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
    result = run_simulation(
        config,
        states,
        MPC_Frenet_FrameController(config),
        lon_controller=None,
        lat_lon_separate=False,
    )
    result["controller"] = "MPC_Frenet_Frame"
    plot_tracking_results([result])


if __name__ == "__main__":
    main()
