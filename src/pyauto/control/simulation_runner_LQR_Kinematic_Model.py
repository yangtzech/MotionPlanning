from .generic_simulator import plot_tracking_results, run_single_simulation


def main():
    result = run_single_simulation("LQRKinematic")
    plot_tracking_results([result])


if __name__ == "__main__":
    main()
