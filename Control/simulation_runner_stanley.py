from config_control import Config
from pid_speed_control import PIDSpeedController
from simulation_common import run_simulation
from simulation_runner_configurable import plot_tracking_results
from Stanley import StanleyController


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
        config, states, StanleyController(config), PIDSpeedController(config)
    )
    result["controller"] = "Stanley"
    plot_tracking_results([result])


if __name__ == "__main__":
    main()
