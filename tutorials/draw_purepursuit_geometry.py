"""
Pure Pursuit控制器几何原理图绘制
展示前视距离、方位角和圆弧轨迹的定义
"""

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Arc

# 设置中文字体
plt.rcParams["font.sans-serif"] = ["SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False


def draw_vehicle(ax, x, y, yaw, wheelbase=2.5, width=1.8):
    """绘制车辆（以后轴中心为参考点）"""

    # 计算车辆四个角点（相对于后轴中心）
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

    # 后轴中心
    rear_x = x
    rear_y = y

    # 前轴中心
    front_x = x + wheelbase * np.cos(yaw)
    front_y = y + wheelbase * np.sin(yaw)

    # 绘制后轮
    wheel_length = 0.5
    wheel_width = 0.2

    for side in [-1, 1]:
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
    steer_angle = 0.3  # 向左转

    for side in [-1, 1]:
        wheel_local = np.array(
            [
                [wheelbase - wheel_length / 2, side * (width / 2 + wheel_width / 2)],
                [wheelbase + wheel_length / 2, side * (width / 2 + wheel_width / 2)],
                [wheelbase + wheel_length / 2, side * (width / 2 - wheel_width / 2)],
                [wheelbase - wheel_length / 2, side * (width / 2 - wheel_width / 2)],
            ]
        )
        steer_rotation = np.array(
            [[np.cos(steer_angle), -np.sin(steer_angle)], [np.sin(steer_angle), np.cos(steer_angle)]]
        )
        wheel_center = np.array([wheelbase, side * width / 2])
        wheel_relative = wheel_local - wheel_center
        wheel_steered = wheel_relative @ steer_rotation.T + wheel_center

        rotated_wheel = wheel_steered @ rotation_matrix.T
        translated_wheel = rotated_wheel + np.array([rear_x, rear_y])
        wheel_patch = patches.Polygon(translated_wheel, closed=True, edgecolor="black", facecolor="gray", linewidth=1.5)
        ax.add_patch(wheel_patch)

    # 标记后轴中心（Pure Pursuit 的参考点，标记为 OR）
    ax.plot(rear_x, rear_y, "ko", markersize=10, zorder=10)
    ax.text(
        rear_x - 1,
        rear_y + 0,
        "后轴中心 $O_R$\n(x, y)",
        fontsize=11,
        ha="center",
        bbox={"boxstyle": "round,pad=0.3", "facecolor": "white", "edgecolor": "black"},
    )

    # 标记前轴中心
    ax.plot(front_x, front_y, "bo", markersize=6, zorder=10)

    return rear_x, rear_y, front_x, front_y, wheelbase


def draw_reference_path(ax):
    """绘制参考路径"""
    t = np.linspace(-1, 12, 150)
    ref_x = t
    ref_y = 3.0 + 0.8 * np.sin(0.3 * t)  # 平缓的正弦曲线

    ax.plot(ref_x, ref_y, "g--", linewidth=2.5, label="参考路径", zorder=5)

    return ref_x, ref_y


def draw_pure_pursuit_geometry(ax, rear_x, rear_y, vehicle_yaw, wheelbase, ref_x, ref_y):
    """绘制 Pure Pursuit 几何关系"""

    # 计算前视距离（假设速度相关）
    Lf = 4.5  # 前视距离

    # 在参考路径上找到前视点
    # 从后轴中心出发，找到距离为 Lf 的点
    distances = np.sqrt((ref_x - rear_x) ** 2 + (ref_y - rear_y) ** 2)

    # 找到距离最接近 Lf 且在车辆前方的点
    # 先找到最近点索引
    nearest_idx = np.argmin(distances)

    # 从最近点开始向前搜索
    goal_idx = nearest_idx
    for i in range(nearest_idx, len(ref_x)):
        if distances[i] >= Lf:
            goal_idx = i
            break
    else:
        goal_idx = len(ref_x) - 1

    goal_x = ref_x[goal_idx]
    goal_y = ref_y[goal_idx]

    # 绘制前视点
    ax.plot(goal_x, goal_y, "ro", markersize=12, zorder=15, markeredgecolor="darkred", markeredgewidth=2)
    ax.text(
        goal_x + 0.4,
        goal_y + 0.6,
        "前视点 G\n$(g_x, g_y)$",
        fontsize=11,
        ha="left",
        bbox={"boxstyle": "round,pad=0.3", "facecolor": "white", "edgecolor": "red", "linewidth": 2},
    )

    # 绘制前视距离 Lf（从后轴中心到前视点）
    ax.plot([rear_x, goal_x], [rear_y, goal_y], "r-", linewidth=3, label="前视距离 $L_f$", zorder=8)

    # 前视距离标注（放在连线下方）
    mid_x = (rear_x + goal_x) / 2
    mid_y = (rear_y + goal_y) / 2
    # 计算垂直于 Lf 线的方向（向下偏移）
    angle_to_goal = np.arctan2(goal_y - rear_y, goal_x - rear_x)
    perpendicular_angle = angle_to_goal - np.pi / 2  # 垂直向下
    offset_dist = 0.6
    label_x = mid_x + offset_dist * np.cos(perpendicular_angle)
    label_y = mid_y + offset_dist * np.sin(perpendicular_angle)
    ax.text(
        label_x,
        label_y,
        "$L_f$",
        fontsize=16,
        color="red",
        fontweight="bold",
        bbox={"boxstyle": "round,pad=0.4", "facecolor": "yellow", "edgecolor": "red", "linewidth": 2, "alpha": 0.8},
    )

    # 计算方位角 alpha
    alpha = np.arctan2(goal_y - rear_y, goal_x - rear_x) - vehicle_yaw

    # 绘制车辆航向线
    heading_length = 5.0
    heading_end_x = rear_x + heading_length * np.cos(vehicle_yaw)
    heading_end_y = rear_y + heading_length * np.sin(vehicle_yaw)
    ax.plot([rear_x, heading_end_x], [rear_y, heading_end_y], "b-", linewidth=2.5, label="车辆航向", zorder=7)
    ax.arrow(
        heading_end_x - 0.4 * np.cos(vehicle_yaw),
        heading_end_y - 0.4 * np.sin(vehicle_yaw),
        0.4 * np.cos(vehicle_yaw),
        0.4 * np.sin(vehicle_yaw),
        head_width=0.25,
        head_length=0.15,
        fc="blue",
        ec="blue",
        linewidth=2,
        zorder=7,
    )

    # 绘制方位角 alpha 的弧线
    arc_radius = 1.8
    angle_to_goal = np.arctan2(goal_y - rear_y, goal_x - rear_x)
    angle1_deg = np.degrees(vehicle_yaw)
    angle2_deg = np.degrees(angle_to_goal)

    arc = Arc(
        (rear_x, rear_y),
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

    # 方位角标注
    arc_mid_angle = (vehicle_yaw + angle_to_goal) / 2
    label_x = rear_x + (arc_radius + 0.5) * np.cos(arc_mid_angle) + 0.2
    label_y = rear_y + (arc_radius + 0.5) * np.sin(arc_mid_angle) + 0.3
    ax.text(
        label_x,
        label_y,
        r"$\alpha$",
        fontsize=18,
        color="purple",
        fontweight="bold",
        ha="center",
        va="center",
        bbox={"boxstyle": "round,pad=0.3", "facecolor": "yellow", "edgecolor": "purple", "linewidth": 2, "alpha": 0.8},
    )

    # 计算转向圆弧
    # 圆弧半径 R = Lf / (2 * sin(alpha))
    if abs(np.sin(alpha)) > 0.01:
        R = Lf / (2 * np.sin(alpha))

        # 圆心在后轴中心的左侧（alpha > 0 时）或右侧（alpha < 0 时）
        # 圆心方向垂直于车辆航向
        if alpha > 0:
            center_angle = vehicle_yaw + np.pi / 2
        else:
            center_angle = vehicle_yaw - np.pi / 2
            R = -R

        center_x = rear_x + R * np.cos(center_angle)
        center_y = rear_y + R * np.sin(center_angle)

        # 绘制转向圆弧（从后轴中心到前视点）
        # 计算圆弧的起止角度
        start_angle = np.arctan2(rear_y - center_y, rear_x - center_x)
        end_angle = np.arctan2(goal_y - center_y, goal_x - center_x)

        # 生成圆弧点
        if alpha > 0:
            angles = np.linspace(start_angle, end_angle, 50)
        else:
            angles = np.linspace(start_angle, end_angle, 50)

        arc_x = center_x + abs(R) * np.cos(angles)
        arc_y = center_y + abs(R) * np.sin(angles)

        ax.plot(arc_x, arc_y, "orange", linewidth=3, linestyle="--", label="圆弧轨迹", zorder=6)

        # 绘制圆心
        ax.plot(center_x, center_y, "m+", markersize=15, markeredgewidth=3, zorder=10)
        ax.text(
            center_x + 0.5,
            center_y + 0.4,
            "圆心 O",
            fontsize=10,
            ha="center",
            color="magenta",
            bbox={"boxstyle": "round,pad=0.2", "facecolor": "white", "edgecolor": "magenta"},
        )

        # 绘制半径 R（从圆心到后轴中心）- 作为 2α 的一条边
        ax.plot([center_x, rear_x], [center_y, rear_y], "m-", linewidth=2.5, zorder=5)

        # 绘制从圆心到前视点的半径 - 作为 2α 的另一条边
        ax.plot([center_x, goal_x], [center_y, goal_y], "m-", linewidth=2.5, zorder=5)

        # 半径 R 标注（放在圆心到后轴中心连线的中点偏左）
        r_mid_x = (center_x + rear_x) / 2
        r_mid_y = (center_y + rear_y) / 2
        ax.text(
            r_mid_x - 0.6,
            r_mid_y + 0.3,
            "$R$",
            fontsize=14,
            color="magenta",
            fontweight="bold",
            bbox={"boxstyle": "round,pad=0.3", "facecolor": "white", "edgecolor": "magenta", "alpha": 0.8},
        )

        # 绘制圆心角 2α 的弧线
        arc_2alpha_radius = 1.0
        arc_2alpha = Arc(
            (center_x, center_y),
            2 * arc_2alpha_radius,
            2 * arc_2alpha_radius,
            angle=0,
            theta1=np.degrees(min(start_angle, end_angle)),
            theta2=np.degrees(max(start_angle, end_angle)),
            color="cyan",
            linewidth=3,
            linestyle="-",
            zorder=9,
        )
        ax.add_patch(arc_2alpha)

        # 2α 标注（放在弧线中点外侧，向下偏移避免与曲线重叠）
        mid_arc_angle = (start_angle + end_angle) / 2
        label_2a_x = center_x + (arc_2alpha_radius + 0.8) * np.cos(mid_arc_angle)
        label_2a_y = center_y + (arc_2alpha_radius + 0.8) * np.sin(mid_arc_angle) - 0.5
        ax.text(
            label_2a_x,
            label_2a_y,
            r"$2\alpha$",
            fontsize=14,
            color="cyan",
            fontweight="bold",
            ha="center",
            va="center",
            bbox={"boxstyle": "round,pad=0.2", "facecolor": "white", "edgecolor": "cyan", "linewidth": 2},
        )

    return alpha, Lf


def main():
    """主函数"""
    fig, ax = plt.subplots(figsize=(16, 10))

    # 车辆位置和姿态（向右移动以避免与左下角文字重叠）
    vehicle_x = 6.0
    vehicle_y = 6.5
    vehicle_yaw = np.deg2rad(-10)  # 略微向下的航向
    wheelbase = 2.5

    # 绘制参考路径
    ref_x, ref_y = draw_reference_path(ax)

    # 绘制车辆
    rear_x, rear_y, front_x, front_y, wb = draw_vehicle(ax, vehicle_x, vehicle_y, vehicle_yaw, wheelbase=wheelbase)

    # 绘制 Pure Pursuit 几何关系
    alpha, Lf = draw_pure_pursuit_geometry(ax, rear_x, rear_y, vehicle_yaw, wheelbase, ref_x, ref_y)

    # 添加控制律公式（移到左上角）
    formula_text = (
        "Pure Pursuit 控制律:\n"
        r"$\delta = \arctan\left(\frac{2 \cdot L \cdot \sin(\alpha)}{L_f}\right)$"
        "\n\n"
        r"前视距离: $L_f = k_f \cdot v + L_d$"
        "\n"
        r"转向半径: $R = \frac{L_f}{2\sin(\alpha)}$"
    )
    ax.text(
        0.5,
        11.5,
        formula_text,
        fontsize=12,
        bbox={"boxstyle": "round,pad=0.5", "facecolor": "lightcyan", "edgecolor": "black", "linewidth": 2},
        verticalalignment="top",
    )

    # 添加参数说明（移到左下角）
    param_text = (
        r"$\alpha$: 方位角 (前视点相对于车辆航向的角度)" + "\n"
        r"$L_f$: 前视距离 (后轴中心到前视点)" + "\n"
        r"$L$: 车辆轴距 (WB)" + "\n"
        r"$R$: 转向圆弧半径" + "\n"
        r"$\delta$: 前轮转向角"
    )
    ax.text(
        0.5,
        2.2,
        param_text,
        fontsize=11,
        bbox={"boxstyle": "round,pad=0.5", "facecolor": "lightyellow", "edgecolor": "black", "linewidth": 1.5},
        verticalalignment="top",
    )

    # 设置坐标轴
    ax.set_xlim(-1, 16)
    ax.set_ylim(-0.5, 12)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("X (m)", fontsize=12)
    ax.set_ylabel("Y (m)", fontsize=12)
    ax.set_title("Pure Pursuit 控制器几何原理图", fontsize=16, fontweight="bold", pad=20)

    # 添加图例
    ax.legend(loc="upper right", fontsize=11, framealpha=0.9)

    plt.tight_layout()

    # 保存图片
    output_path = "/home/yzc/projects/MotionPlanning/docs/control/image/PurePursuit/purepursuit_geometry_diagram.png"
    plt.savefig(output_path, dpi=300, bbox_inches="tight")
    print(f"图片已保存到: {output_path}")

    plt.show()


if __name__ == "__main__":
    main()
