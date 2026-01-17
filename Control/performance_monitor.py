"""
实时性能监控器，用于在仿真过程中实时更新性能指标图表
"""

import matplotlib.pyplot as plt
import numpy as np


class RealTimePerformanceMonitor:
    """
    实时性能监控器，用于在仿真过程中实时更新性能指标图表
    包括速度、横向误差和航向误差的实时显示
    支持多段轨迹的区分显示
    """

    def __init__(self, controller_name="Controller", max_display_points=1000):
        """
        初始化实时性能监控器

        Args:
            controller_name: 控制器名称
            max_display_points: 最大显示点数，超过此数值将滚动显示
        """
        self.controller_name = controller_name
        self.max_display_points = max_display_points

        # 存储所有段的数据
        self.all_segments_data = []  # 存储每段的完整数据
        self.current_segment_data = {
            "times": [],
            "actual_speeds": [],
            "ref_speeds": [],
            "lat_errors": [],
            "yaw_errors": [],
            "segment_id": 0,  # 当前段ID
        }

        # 创建图形
        self.fig, self.axs = plt.subplots(3, 1, figsize=(10, 10))

        # 存储所有线条对象
        self.segment_lines = []  # 存储每段的线条

        # 设置图形属性
        self.axs[0].set_ylabel("Speed (km/h)")
        self.axs[0].grid(True, linestyle="--", alpha=0.5)

        self.axs[1].set_ylabel("Latitude Error (m)")
        self.axs[1].grid(True, linestyle="--", alpha=0.5)

        self.axs[2].set_ylabel("Heading Error (rad)")
        self.axs[2].set_xlabel("Time (s)")
        self.axs[2].grid(True, linestyle="--", alpha=0.5)

        self.fig.suptitle(
            f"Real-time Performance Monitor - {controller_name}", fontsize=14
        )
        plt.tight_layout()

        # 暂停控制
        self.is_paused = False
        self.should_exit = False
        self.event_bound = False

        # 绑定键盘事件
        if not self.event_bound:
            self.fig.canvas.mpl_connect("key_release_event", self.on_key)
            self.event_bound = True

    def on_key(self, event):
        """键盘事件处理"""
        if event.key == " ":
            self.is_paused = not self.is_paused
        elif event.key == "escape":
            self.should_exit = True
            plt.close(self.fig)  # 确保窗口关闭

    def start_new_segment(self):
        """开始新段，保存当前段数据并准备新段"""
        if self.current_segment_data["times"]:
            # 保存当前段的数据
            self.all_segments_data.append(self.current_segment_data.copy())

            # 为新段创建新的数据字典
            self.current_segment_data = {
                "times": [],
                "actual_speeds": [],
                "ref_speeds": [],
                "lat_errors": [],
                "yaw_errors": [],
                "segment_id": len(self.all_segments_data),  # 新段ID
            }

    def update_data(self, time, actual_speed, ref_speed, lat_error, yaw_error):
        """
        更新数据并刷新图表

        Args:
            time: 当前时间
            actual_speed: 实际速度
            ref_speed: 参考速度
            lat_error: 横向误差
            yaw_error: 航向误差
        """
        if self.should_exit:
            return False  # 返回False表示应停止更新

        if self.is_paused:
            return True  # 返回True表示应继续更新

        # 添加数据到当前段
        self.current_segment_data["times"].append(time)
        self.current_segment_data["actual_speeds"].append(actual_speed)
        self.current_segment_data["ref_speeds"].append(ref_speed)
        self.current_segment_data["lat_errors"].append(lat_error)
        self.current_segment_data["yaw_errors"].append(yaw_error)

        # 更新整个图表显示所有段的数据
        self._refresh_plot()

        return True  # 表示应继续更新

    def _refresh_plot(self):
        """刷新整个图表，显示所有段的数据"""
        # 清除当前所有线条
        for lines in self.segment_lines:
            for line in lines.values():
                if line:
                    line.remove()
        self.segment_lines = []

        # 绘制所有已完成的段
        colors = plt.cm.tab10(np.linspace(0, 1, 10))  # 使用预定义颜色
        for seg_idx, seg_data in enumerate(self.all_segments_data):
            if seg_data["times"]:  # 确保段有数据
                color = colors[seg_idx % len(colors)]

                # 绘制该段的线条
                if seg_data["times"]:  # 确保有数据才绘制
                    speed_line_actual = self.axs[0].plot(
                        seg_data["times"],
                        seg_data["actual_speeds"],
                        "-",
                        label=f"Seg {seg_idx + 1} Actual",
                        color=color,
                        linewidth=1.0,
                    )[0]
                    speed_line_ref = self.axs[0].plot(
                        seg_data["times"],
                        seg_data["ref_speeds"],
                        "--",
                        label=f"Seg {seg_idx + 1} Ref",
                        color=color,
                        linewidth=1.0,
                    )[0]

                    lat_error_line = self.axs[1].plot(
                        seg_data["times"],
                        seg_data["lat_errors"],
                        "-",
                        label=f"Seg {seg_idx + 1} Lat",
                        color=color,
                        linewidth=1.0,
                    )[0]

                    yaw_error_line = self.axs[2].plot(
                        seg_data["times"],
                        seg_data["yaw_errors"],
                        "-",
                        label=f"Seg {seg_idx + 1} Yaw",
                        color=color,
                        linewidth=1.0,
                    )[0]

                    self.segment_lines.append(
                        {
                            "speed_actual": speed_line_actual,
                            "speed_ref": speed_line_ref,
                            "lat_error": lat_error_line,
                            "yaw_error": yaw_error_line,
                        }
                    )

        # 绘制当前段（如果有的话）
        if self.current_segment_data["times"]:
            seg_idx = len(self.all_segments_data)
            color = colors[seg_idx % len(colors)]

            # 绘制当前段的线条
            speed_line_actual = self.axs[0].plot(
                self.current_segment_data["times"],
                self.current_segment_data["actual_speeds"],
                "-",
                label=f"Seg {seg_idx + 1} Actual (curr)",
                color=color,
                linewidth=1.5,
            )[0]
            speed_line_ref = self.axs[0].plot(
                self.current_segment_data["times"],
                self.current_segment_data["ref_speeds"],
                "--",
                label=f"Seg {seg_idx + 1} Ref (curr)",
                color=color,
                linewidth=1.5,
            )[0]

            lat_error_line = self.axs[1].plot(
                self.current_segment_data["times"],
                self.current_segment_data["lat_errors"],
                "-",
                label=f"Seg {seg_idx + 1} Lat (curr)",
                color=color,
                linewidth=1.5,
            )[0]

            yaw_error_line = self.axs[2].plot(
                self.current_segment_data["times"],
                self.current_segment_data["yaw_errors"],
                "-",
                label=f"Seg {seg_idx + 1} Yaw (curr)",
                color=color,
                linewidth=1.5,
            )[0]

            self.segment_lines.append(
                {
                    "speed_actual": speed_line_actual,
                    "speed_ref": speed_line_ref,
                    "lat_error": lat_error_line,
                    "yaw_error": yaw_error_line,
                }
            )

        # 更新坐标轴范围
        self.axs[0].relim()
        self.axs[0].autoscale_view(scalex=True, scaley=True)

        self.axs[1].relim()
        self.axs[1].autoscale_view(scalex=True, scaley=True)

        self.axs[2].relim()
        self.axs[2].autoscale_view(scalex=True, scaley=True)

        # 重新设置图例
        for ax in self.axs:
            handles, labels = ax.get_legend_handles_labels()
            ax.legend(handles, labels, loc="upper right", fontsize=8)

        # 刷新图形
        self.fig.canvas.draw_idle()

    def finalize_current_segment(self):
        """完成当前段，将数据移到已完成段列表"""
        if self.current_segment_data["times"]:
            self.all_segments_data.append(self.current_segment_data.copy())
            self.current_segment_data = {
                "times": [],
                "actual_speeds": [],
                "ref_speeds": [],
                "lat_errors": [],
                "yaw_errors": [],
                "segment_id": len(self.all_segments_data),  # 新段ID
            }
            # 刷新图表以显示新的段
            self._refresh_plot()

    def pause(self):
        """暂停更新"""
        self.is_paused = True

    def resume(self):
        """恢复更新"""
        self.is_paused = False

    def close(self):
        """关闭监控器窗口"""
        plt.close(self.fig)


def get_performance_monitor(controller_name="Controller", max_display_points=1000):
    """
    获取性能监控器实例

    Args:
        controller_name: 控制器名称
        max_display_points: 最大显示点数

    Returns:
        RealTimePerformanceMonitor: 性能监控器实例（支持多段轨迹）
    """
    return RealTimePerformanceMonitor(controller_name, max_display_points)
