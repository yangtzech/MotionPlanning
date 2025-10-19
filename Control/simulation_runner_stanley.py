from config_control import Config
from simulation_common import run_simulation


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
    run_simulation(config, states, "Stanley", "PID")


if __name__ == "__main__":
    main()
