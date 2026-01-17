"""
车辆路径跟踪动画更新器，实现在一张图上不断更新车辆位置和轨迹
"""

import matplotlib.pyplot as plt
import numpy as np

import Control.draw as draw


class AdvancedAnimationUpdater:
    """
    高级动画更新器，使用draw.draw_car函数进行车辆绘制
    实现：保留轨迹曲线，只更新当前车辆位置，移除之前车辆位置
    """

    def __init__(self, path_x, path_y, config, draw_car_func=None):
        """
        初始化高级动画更新器

        Args:
            path_x: 参考路径x坐标
            path_y: 参考路径y坐标
            config: 配置对象
            draw_car_func: 绘制车辆的函数，默认为draw.draw_car
        """
        self.path_x = path_x
        self.path_y = path_y
        self.config = config
        self.draw_car_func = draw_car_func or draw.draw_car
        self.x_rec = []  # 车辆历史轨迹x坐标
        self.y_rec = []  # 车辆历史轨迹y坐标
        self.fig, self.ax = plt.subplots(figsize=(10, 8))

        # 绘制静态元素（参考路径）
        self.ax.plot(
            self.path_x, self.path_y, color="gray", linewidth=2, label="Reference Path"
        )

        # 创建可更新的线条对象
        (self.traj_line,) = self.ax.plot(
            [], [], color="darkviolet", linewidth=2, label="Vehicle Trajectory"
        )
        (self.target_point,) = self.ax.plot(
            [], [], ".r", markersize=8, label="Target Point"
        )

        # 车辆图形对象列表（用于删除之前的车辆）
        self.vehicle_artists = []

        # 设置图形属性
        self.ax.set_title("Vehicle Path Tracking")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.grid(True, linestyle="--", alpha=0.5)
        self.ax.axis("equal")
        self.ax.legend()

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

    def _clear_previous_vehicle(self):
        """清除之前的车辆图形"""
        for artist in self.vehicle_artists:
            try:
                artist.remove()
            except ValueError:
                # 如果图形已经被移除，则忽略
                pass
        self.vehicle_artists.clear()

    def update_frame(self, node, cx, cy, target_ind, steer):
        """
        更新动画帧

        Args:
            node: 当前车辆状态
            cx: 当前段参考路径x坐标
            cy: 当前段参考路径y坐标
            target_ind: 目标索引
            steer: 转向角度
        """
        if self.should_exit:
            return False  # 返回False表示应停止更新

        if self.is_paused:
            return True  # 返回True表示应继续更新

        # 添加当前位置到轨迹
        self.x_rec.append(node.x)
        self.y_rec.append(node.y)

        # 高效更新轨迹线（保留所有历史轨迹）
        self.traj_line.set_data(self.x_rec, self.y_rec)

        # 更新目标点
        if target_ind < len(cx) and target_ind < len(cy):
            self.target_point.set_data([cx[target_ind]], [cy[target_ind]])

        # 清除之前的车辆图形
        self._clear_previous_vehicle()

        # 保存当前轴
        original_ax = plt.gca()

        # 设置当前轴为动画轴
        if original_ax != self.ax:
            plt.sca(self.ax)

        # 为了精确控制车辆绘制，我们手动实现车辆绘制而不是依赖draw_car
        # 这样可以更好地控制只显示当前车辆而不保留之前的
        self._draw_current_vehicle(node.x, node.y, node.yaw, steer)

        # 恢复原来的轴（如果有）
        if original_ax != self.ax:
            plt.sca(original_ax)

        # 更新标题
        speed_kmh = node.v * 3.6
        self.ax.set_title(f"Vehicle Path Tracking - v={speed_kmh:.2f} km/h")

        # 保持相等的轴比例
        self.ax.relim()
        self.ax.autoscale_view()

        # 强制刷新
        self.fig.canvas.draw_idle()

        return True  # 表示应继续更新

    def _draw_current_vehicle(self, x, y, yaw, steer):
        """
        手动绘制当前车辆，确保只显示当前车辆而不保留之前的位置
        """
        C = self.config

        # 车身
        car = np.array(
            [
                [-C.RB, -C.RB, C.RF, C.RF, -C.RB],
                [C.W / 2, -C.W / 2, -C.W / 2, C.W / 2, C.W / 2],
            ]
        )

        # 车轮
        wheel = np.array(
            [
                [-C.TR, -C.TR, C.TR, C.TR, -C.TR],
                [C.TW / 4, -C.TW / 4, -C.TW / 4, C.TW / 4, C.TW / 4],
            ]
        )

        rlWheel = wheel.copy()
        rrWheel = wheel.copy()
        frWheel = wheel.copy()
        flWheel = wheel.copy()

        Rot1 = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])

        Rot2 = np.array(
            [[np.cos(steer), np.sin(steer)], [-np.sin(steer), np.cos(steer)]]
        )

        frWheel = np.dot(Rot2, frWheel)
        flWheel = np.dot(Rot2, flWheel)

        frWheel += np.array([[C.WB], [-C.WD / 2]])
        flWheel += np.array([[C.WB], [C.WD / 2]])
        rrWheel[1, :] -= C.WD / 2
        rlWheel[1, :] += C.WD / 2

        frWheel = np.dot(Rot1, frWheel)
        flWheel = np.dot(Rot1, flWheel)

        rrWheel = np.dot(Rot1, rrWheel)
        rlWheel = np.dot(Rot1, rlWheel)
        car = np.dot(Rot1, car)

        frWheel += np.array([[x], [y]])
        flWheel += np.array([[x], [y]])
        rrWheel += np.array([[x], [y]])
        rlWheel += np.array([[x], [y]])
        car += np.array([[x], [y]])

        # 绘制车辆各部分并保存引用以便稍后清除
        car_line = self.ax.plot(car[0, :], car[1, :], "red", linewidth=2)[0]
        fr_line = self.ax.plot(frWheel[0, :], frWheel[1, :], "red")[0]
        fl_line = self.ax.plot(flWheel[0, :], flWheel[1, :], "red")[0]
        rr_line = self.ax.plot(rrWheel[0, :], rrWheel[1, :], "red")[0]
        rl_line = self.ax.plot(rlWheel[0, :], rlWheel[1, :], "red")[0]

        # 添加方向箭头
        from matplotlib.patches import FancyArrowPatch

        arrow_length = C.WB * 0.6
        dx = arrow_length * np.cos(yaw)
        dy = arrow_length * np.sin(yaw)
        arrow = FancyArrowPatch(
            (x, y),
            (x + dx, y + dy),
            arrowstyle="->",
            mutation_scale=20,
            color="blue",
            linewidth=2,
        )
        self.ax.add_patch(arrow)

        # 将新绘制的车辆部分添加到列表中，以便下次清除
        self.vehicle_artists.extend(
            [car_line, fr_line, fl_line, rr_line, rl_line, arrow]
        )

    def clear_trajectory(self):
        """清空轨迹记录"""
        self.x_rec.clear()
        self.y_rec.clear()
        self.traj_line.set_data([], [])

    def pause(self):
        """暂停动画"""
        self.is_paused = True

    def resume(self):
        """恢复动画"""
        self.is_paused = False

    def close(self):
        """关闭动画窗口"""
        plt.close(self.fig)


def get_animator(path_x, path_y, config):
    """
    获取动画更新器实例

    Args:
        path_x: 参考路径x坐标
        path_y: 参考路径y坐标
        config: 配置对象

    Returns:
        AdvancedAnimationUpdater: 动画更新器实例
    """
    return AdvancedAnimationUpdater(path_x, path_y, config)
