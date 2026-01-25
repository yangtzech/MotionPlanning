"""
Stanley控制器几何原理图绘制
展示航向误差和横向误差的定义
"""

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Arc

# 设置中文字体
plt.rcParams["font.sans-serif"] = ["SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False


def draw_vehicle(ax, x, y, yaw, wheelbase=2.5, width=1.8):
    """绘制车辆"""

    # 计算车辆四个角点（相对于后轴中心，后轴在原点）
    # 车辆后部留一点空间
    rear_overhang = 0.3
    corners = np.array(
        [
            [-rear_overhang, -width / 2],
            [wheelbase + 0.5, -width / 2],
            [wheelbase + 0.5, width / 2],
            [-rear_overhang, width / 2],
            [-rear_overhang, -width / 2],
        ]
    )

    # 旋转和平移
    rotation_matrix = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    rotated_corners = corners @ rotation_matrix.T
    translated_corners = rotated_corners + np.array([x, y])

    # 绘制车辆轮廓
    vehicle_patch = patches.Polygon(
        translated_corners, closed=True, edgecolor="black", facecolor="lightblue", linewidth=2, alpha=0.7
    )
    ax.add_patch(vehicle_patch)

    # 计算后轴中心
    rear_x = x
    rear_y = y

    # 计算前轴中心
    front_x = x + wheelbase * np.cos(yaw)
    front_y = y + wheelbase * np.sin(yaw)

    # 绘制后轴轮子（位于后轴位置，即车辆坐标系原点）
    wheel_length = 0.5
    wheel_width = 0.2

    # 后轮在车辆坐标系中的位置（y方向）
    for side in [-1, 1]:  # 左右两侧
        wheel_local = np.array(
            [
                [-wheel_length / 2, side * (width / 2 + wheel_width / 2)],
                [wheel_length / 2, side * (width / 2 + wheel_width / 2)],
                [wheel_length / 2, side * (width / 2 - wheel_width / 2)],
                [-wheel_length / 2, side * (width / 2 - wheel_width / 2)],
            ]
        )
        rotated_wheel = wheel_local @ rotation_matrix.T
        translated_wheel = rotated_wheel + np.array([rear_x, rear_y])
        wheel_patch = patches.Polygon(translated_wheel, closed=True, edgecolor="black", facecolor="gray", linewidth=1.5)
        ax.add_patch(wheel_patch)

    # 绘制前轮（带转向角）
    steer_angle = -0.6  # 向右转更大角度（负角度）

    for side in [-1, 1]:  # 左右两侧
        # 前轮在车辆坐标系中的位置
        wheel_local = np.array(
            [
                [wheelbase - wheel_length / 2, side * (width / 2 + wheel_width / 2)],
                [wheelbase + wheel_length / 2, side * (width / 2 + wheel_width / 2)],
                [wheelbase + wheel_length / 2, side * (width / 2 - wheel_width / 2)],
                [wheelbase - wheel_length / 2, side * (width / 2 - wheel_width / 2)],
            ]
        )
        # 先应用转向角，再应用车辆航向
        steer_rotation = np.array(
            [[np.cos(steer_angle), -np.sin(steer_angle)], [np.sin(steer_angle), np.cos(steer_angle)]]
        )
        # 转向旋转中心在前轴位置
        wheel_center = np.array([wheelbase, side * width / 2])
        wheel_relative = wheel_local - wheel_center
        wheel_steered = wheel_relative @ steer_rotation.T + wheel_center

        # 应用车辆整体旋转和平移
        rotated_wheel = wheel_steered @ rotation_matrix.T
        translated_wheel = rotated_wheel + np.array([rear_x, rear_y])
        wheel_patch = patches.Polygon(translated_wheel, closed=True, edgecolor="black", facecolor="gray", linewidth=1.5)
        ax.add_patch(wheel_patch)

    # 标记后轴中心
    ax.plot(rear_x, rear_y, "ko", markersize=8, zorder=10)
    ax.text(
        rear_x - 0.8,
        rear_y - 0.5,
        "后轴中心\n(x, y)",
        fontsize=11,
        ha="center",
        bbox={"boxstyle": "round,pad=0.3", "facecolor": "white", "edgecolor": "black"},
    )

    # 标记前轴中心
    ax.plot(front_x, front_y, "ro", markersize=8, zorder=10)
    ax.text(
        front_x + 0.8,
        front_y + 0.5,
        "前轴中心 F\n(fx, fy)",
        fontsize=11,
        ha="center",
        bbox={"boxstyle": "round,pad=0.3", "facecolor": "white", "edgecolor": "red"},
    )

    return rear_x, rear_y, front_x, front_y


def draw_reference_path(ax):
    """绘制参考路径（曲线）"""
    # 参考路径（往下移动1m）
    t = np.linspace(0, 8, 100)
    ref_x = t
    ref_y = 4.5 + 0.5 * np.sin(0.5 * t)  # 从5.5改为4.5

    ax.plot(ref_x, ref_y, "g--", linewidth=2.5, label="参考路径", zorder=5)

    return ref_x, ref_y


def draw_errors(ax, front_x, front_y, rear_x, rear_y, vehicle_yaw, ref_x, ref_y, steer_angle=-0.6):
    """绘制误差"""
    # 找到参考路径上最近的点
    distances = np.sqrt((ref_x - front_x) ** 2 + (ref_y - front_y) ** 2)
    nearest_idx = np.argmin(distances)
    nearest_x = ref_x[nearest_idx]
    nearest_y = ref_y[nearest_idx]

    # 计算参考路径在该点的切线方向
    if nearest_idx < len(ref_x) - 1:
        dx = ref_x[nearest_idx + 1] - ref_x[nearest_idx]
        dy = ref_y[nearest_idx + 1] - ref_y[nearest_idx]
    else:
        dx = ref_x[nearest_idx] - ref_x[nearest_idx - 1]
        dy = ref_y[nearest_idx] - ref_y[nearest_idx - 1]

    path_yaw = np.arctan2(dy, dx)

    # 绘制横向误差 (ed)
    ax.plot([front_x, nearest_x], [front_y, nearest_y], "r-", linewidth=2.5, label="横向误差 $e_d$")
    ax.plot(nearest_x, nearest_y, "go", markersize=10, zorder=10)
    # 标注最近点H
    ax.text(
        nearest_x - 0.8,
        nearest_y - 0.3,
        "H",
        fontsize=13,
        color="green",
        fontweight="bold",
        bbox={"boxstyle": "round,pad=0.3", "facecolor": "white", "edgecolor": "green", "linewidth": 2},
    )

    # 横向误差标注
    mid_x = (front_x + nearest_x) / 2
    mid_y = (front_y + nearest_y) / 2
    ax.text(
        mid_x - 0.5,
        mid_y,
        "$e_d$",
        fontsize=16,
        color="red",
        fontweight="bold",
        bbox={"boxstyle": "round,pad=0.4", "facecolor": "yellow", "edgecolor": "red", "linewidth": 2, "alpha": 0.8},
    )

    # 计算前轮速度方向（车辆航向+转向角）
    front_wheel_direction = vehicle_yaw + steer_angle

    # 计算两条线的交点
    # 前轮速度方向线: (front_x, front_y) + t1 * (cos(front_wheel_direction), sin(front_wheel_direction))
    # 路径切线: (nearest_x, nearest_y) + t2 * (cos(path_yaw), sin(path_yaw))
    # 求解线性方程组
    A = np.array(
        [[np.cos(front_wheel_direction), -np.cos(path_yaw)], [np.sin(front_wheel_direction), -np.sin(path_yaw)]]
    )
    b = np.array([nearest_x - front_x, nearest_y - front_y])

    try:
        params = np.linalg.solve(A, b)
        t1 = params[0]
        intersection_x = front_x + t1 * np.cos(front_wheel_direction)
        intersection_y = front_y + t1 * np.sin(front_wheel_direction)

        # 重新绘制两条延长线，使其绘制到交点
        # 前轮速度方向延长线 - 绘制到交点，带箭头
        ax.plot(
            [front_x, intersection_x],
            [front_y, intersection_y],
            "orange",
            linewidth=2.5,
            linestyle="--",
            label="前轮速度方向",
            zorder=7,
        )
        # 在末端添加箭头
        arrow_start_ratio = 0.85  # 箭头从85%位置开始
        arrow_start_x = front_x + arrow_start_ratio * (intersection_x - front_x)
        arrow_start_y = front_y + arrow_start_ratio * (intersection_y - front_y)
        ax.arrow(
            arrow_start_x,
            arrow_start_y,
            (intersection_x - arrow_start_x) * 0.95,
            (intersection_y - arrow_start_y) * 0.95,
            head_width=0.25,
            head_length=0.2,
            fc="orange",
            ec="orange",
            linewidth=2,
            zorder=7,
        )

        # 标注速度v在箭头旁边
        mid_point_x = (front_x + intersection_x) * 0.6
        mid_point_y = (front_y + intersection_y) * 0.6
        ax.text(
            mid_point_x + 0.3,
            mid_point_y - 1.8,
            "$v$",
            fontsize=14,
            color="orange",
            fontweight="bold",
            bbox={"boxstyle": "round,pad=0.4", "facecolor": "white", "edgecolor": "orange", "linewidth": 2},
        )

        # 路径切线延长线 - 绘制到交点
        # 计算从最近点到交点的方向和距离
        path_line_start_x = nearest_x - 1.0 * np.cos(path_yaw)  # 向后延伸一点
        path_line_start_y = nearest_y - 1.0 * np.sin(path_yaw)
        ax.plot(
            [path_line_start_x, intersection_x],
            [path_line_start_y, intersection_y],
            "cyan",
            linewidth=2.5,
            linestyle="--",
            label="路径切线延长",
            zorder=7,
        )

        # 绘制交点
        ax.plot(
            intersection_x, intersection_y, "ro", markersize=10, zorder=12, markeredgecolor="darkred", markeredgewidth=2
        )
        # 标注交点C
        ax.text(
            intersection_x + 0.4,
            intersection_y + 0.4,
            "C",
            fontsize=13,
            color="darkred",
            fontweight="bold",
            bbox={"boxstyle": "round,pad=0.3", "facecolor": "white", "edgecolor": "darkred", "linewidth": 2},
        )
    except np.linalg.LinAlgError:
        # 如果线平行，不绘制交点
        # 绘制固定长度的延长线
        front_line_length = 5.0
        front_line_end_x = front_x + front_line_length * np.cos(front_wheel_direction)
        front_line_end_y = front_y + front_line_length * np.sin(front_wheel_direction)
        ax.plot(
            [front_x, front_line_end_x],
            [front_y, front_line_end_y],
            "orange",
            linewidth=2.5,
            linestyle="--",
            label="前轮速度方向",
            zorder=7,
        )

        path_line_length = 5.0
        path_line_start_x = nearest_x - path_line_length * 0.3 * np.cos(path_yaw)
        path_line_start_y = nearest_y - path_line_length * 0.3 * np.sin(path_yaw)
        path_line_end_x = nearest_x + path_line_length * 0.7 * np.cos(path_yaw)
        path_line_end_y = nearest_y + path_line_length * 0.7 * np.sin(path_yaw)
        ax.plot(
            [path_line_start_x, path_line_end_x],
            [path_line_start_y, path_line_end_y],
            "cyan",
            linewidth=2.5,
            linestyle="--",
            label="路径切线延长",
            zorder=7,
        )

    # 航向误差可视化 - 使用两条直线表示
    # 航向误差
    e_phi = vehicle_yaw - path_yaw

    # 归一化角度到 [-pi, pi]
    while e_phi > np.pi:
        e_phi -= 2 * np.pi
    while e_phi < -np.pi:
        e_phi += 2 * np.pi

    # 从前轴中心向外延伸的两条方向线
    line_length = 2.8

    # 绘制车辆航向线（蓝色）
    vehicle_end_x = front_x + line_length * np.cos(vehicle_yaw)
    vehicle_end_y = front_y + line_length * np.sin(vehicle_yaw)
    ax.plot([front_x, vehicle_end_x], [front_y, vehicle_end_y], "b-", linewidth=3, label="车辆航向 $\\theta$", zorder=8)
    ax.arrow(
        vehicle_end_x - 0.3 * np.cos(vehicle_yaw),
        vehicle_end_y - 0.3 * np.sin(vehicle_yaw),
        0.3 * np.cos(vehicle_yaw),
        0.3 * np.sin(vehicle_yaw),
        head_width=0.25,
        head_length=0.15,
        fc="blue",
        ec="blue",
        linewidth=2,
        zorder=8,
    )

    # 绘制路径切线方向（绿色）
    path_end_x = front_x + line_length * np.cos(path_yaw)
    path_end_y = front_y + line_length * np.sin(path_yaw)
    ax.plot([front_x, path_end_x], [front_y, path_end_y], "g-", linewidth=3, label="路径切线 $\\theta_{ref}$", zorder=8)
    ax.arrow(
        path_end_x - 0.3 * np.cos(path_yaw),
        path_end_y - 0.3 * np.sin(path_yaw),
        0.3 * np.cos(path_yaw),
        0.3 * np.sin(path_yaw),
        head_width=0.25,
        head_length=0.15,
        fc="green",
        ec="green",
        linewidth=2,
        zorder=8,
    )

    # 绘制航向误差角度弧（使用较小的弧线）
    arc_radius = 1.0
    angle1_deg = np.degrees(path_yaw)
    angle2_deg = np.degrees(vehicle_yaw)

    arc = Arc(
        (front_x, front_y),
        2 * arc_radius,
        2 * arc_radius,
        angle=0,
        theta1=min(angle1_deg, angle2_deg),
        theta2=max(angle1_deg, angle2_deg),
        color="purple",
        linewidth=3,
        linestyle="-",
        zorder=9,
    )
    ax.add_patch(arc)

    # 航向误差标注（放在航向线的右边）
    # 计算一个位置，在车辆航向方向的右侧
    ax.text(
        front_x + 1 * np.cos(vehicle_yaw) + 0.5,
        front_y + 1 * np.sin(vehicle_yaw) + 0.3,
        "$e_\\phi$",
        fontsize=16,
        color="purple",
        fontweight="bold",
        bbox={"boxstyle": "round,pad=0.4", "facecolor": "yellow", "edgecolor": "purple", "linewidth": 2, "alpha": 0.8},
    )

    return e_phi, distances[nearest_idx]


def main():
    """主函数"""
    fig, ax = plt.subplots(figsize=(14, 8))

    # 车辆位置和姿态（让横向误差和航向误差都偏向右侧）
    vehicle_x = 3.5
    vehicle_y = 7.0  # 向上移动，在路径上方
    vehicle_yaw = np.deg2rad(0)  # 车身航向沿x轴方向
    wheelbase = 2.5

    # 绘制参考路径
    ref_x, ref_y = draw_reference_path(ax)

    # 绘制车辆
    rear_x, rear_y, front_x, front_y = draw_vehicle(ax, vehicle_x, vehicle_y, vehicle_yaw, wheelbase=wheelbase)

    # 绘制误差（传递转向角）
    steer_angle = -0.6  # 与绘制车辆时使用的相同转向角
    e_phi, e_d = draw_errors(ax, front_x, front_y, rear_x, rear_y, vehicle_yaw, ref_x, ref_y, steer_angle)

    # 添加控制律公式
    formula_text = (
        "Stanley 控制律:\n"
        r"$\delta(t) = -e_\phi \cdot \mathrm{sign}(v) + \arctan\left(\frac{-k \cdot e_d}{|v|}\right)$"
    )
    ax.text(
        0.5,
        9.5,
        formula_text,
        fontsize=13,
        bbox={"boxstyle": "round,pad=0.5", "facecolor": "lightcyan", "edgecolor": "black", "linewidth": 2},
        verticalalignment="top",
    )

    # 添加误差说明
    error_text = (
        "$e_\\phi$: 航向误差 (车辆航向与路径切线的夹角)\n"
        "$e_d$: 横向误差 (前轴中心到参考路径的垂直距离)\n"
        "$v$: 车辆速度\n"
        "$k$: 横向误差增益系数"
    )
    ax.text(
        0.5,
        4.0,
        error_text,
        fontsize=11,
        bbox={"boxstyle": "round,pad=0.5", "facecolor": "lightyellow", "edgecolor": "black", "linewidth": 1.5},
        verticalalignment="top",
    )

    # 设置坐标轴（向上移动整体布局，扩大横轴）
    ax.set_xlim(-1, 14)  # 扩大横轴范围
    ax.set_ylim(1.5, 10.5)  # 调整y轴范围，向上移动
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("X (m)", fontsize=12)
    ax.set_ylabel("Y (m)", fontsize=12)
    ax.set_title("Stanley 控制器几何原理图", fontsize=16, fontweight="bold", pad=20)

    # 添加图例
    ax.legend(loc="upper right", fontsize=11, framealpha=0.9)

    plt.tight_layout()

    # 保存图片
    output_path = "/home/yzc/projects/MotionPlanning/docs/control/image/Stanley/stanley_geometry_diagram.png"
    plt.savefig(output_path, dpi=300, bbox_inches="tight")
    print(f"图片已保存到: {output_path}")

    plt.show()


if __name__ == "__main__":
    main()
