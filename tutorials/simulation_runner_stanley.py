from generic_simulator import plot_tracking_results, run_single_simulation


def main():
    # 默认生成GIF，不显示实时动画以避免阻塞
    result = run_single_simulation("Stanley", save_gif=True)
    plot_tracking_results([result])


if __name__ == "__main__":
    main()
