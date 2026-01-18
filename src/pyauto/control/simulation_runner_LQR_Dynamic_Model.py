from .generic_simulator import plot_tracking_results, run_single_simulation


def main():
    # 只使用实时性能监控（支持多段轨迹显示）
    result = run_single_simulation(
        "LQRDynamic",
        show_realtime_performance=True,  # 使用实时性能监控
    )
    plot_tracking_results([result])


if __name__ == "__main__":
    main()
