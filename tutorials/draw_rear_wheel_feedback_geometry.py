"""
Rear Wheel Feedback 控制器几何原理图绘制
展示基于后轴中心的横向误差、航向误差和曲率前馈
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

    # 后轴中心（Rear Wheel Feedback 的参考点）
    rear_x = x
    rear_y = y

    # 前轴中心
    front_x = x + wheelbase * np.cos(yaw)
    front_y = y + wheelbase * np.sin(yaw)

    # 绘制后轮
    wheel_length = 0.5
    wheel_width = 0.2

    for side in [-1, 1]:
        wheel_y_offset = side * width / 2
        wheel_center_x = rear_x + wheel_y_offset * np.sin(yaw)
        wheel_center_y = rear_y - wheel_y_offset * np.cos(yaw)

        wheel_corners = np.array(
            [
                [-wheel_length / 2, -wheel_width / 2],
                [wheel_length / 2, -wheel_width / 2],
                [wheel_length / 2, wheel_width / 2],
                [-wheel_length / 2, wheel_width / 2],
            ]
        )
        rotated_wheel = wheel_corners @ rotation_matrix.T
        translated_wheel = rotated_wheel + np.array([wheel_center_x, wheel_center_y])

        wheel_patch = patches.Polygon(
            translated_wheel, closed=True, edgecolor="black", facecolor="darkgray", linewidth=1.5
        )
        ax.add_patch(wheel_patch)

    # 绘制前轮（带转向角）
    steer_angle = -0.4  # 向右转

    for side in [-1, 1]:
        wheel_y_offset = side * width / 2
        wheel_center_x = front_x + wheel_y_offset * np.sin(yaw)
        wheel_center_y = front_y - wheel_y_offset * np.cos(yaw)

        wheel_corners = np.array(
            [
                [-wheel_length / 2, -wheel_width / 2],
                [wheel_length / 2, -wheel_width / 2],
                [wheel_length / 2, wheel_width / 2],
                [-wheel_length / 2, wheel_width / 2],
            ]
        )

        wheel_yaw = yaw + steer_angle
        wheel_rotation = np.array([[np.cos(wheel_yaw), -np.sin(wheel_yaw)], [np.sin(wheel_yaw), np.cos(wheel_yaw)]])
        rotated_wheel = wheel_corners @ wheel_rotation.T
        translated_wheel = rotated_wheel + np.array([wheel_center_x, wheel_center_y])

        wheel_patch = patches.Polygon(
            translated_wheel, closed=True, edgecolor="black", facecolor="darkgray", linewidth=1.5
        )
        ax.add_patch(wheel_patch)

    # 标记后轴中心（Rear Wheel Feedback 的控制参考点）
    ax.plot(rear_x, rear_y, "ko", markersize=12, zorder=10, markeredgecolor="black", markeredgewidth=2)
    ax.text(
        rear_x - 1.0,
        rear_y + 0.5,
        "后轴中心 R\n(x, y)",
        fontsize=11,
        ha="center",
        bbox={"boxstyle": "round,pad=0.3", "facecolor": "white", "edgecolor": "black", "linewidth": 2},
    )

    # 标记前轴中心
    ax.plot(front_x, front_y, "bo", markersize=6, zorder=10)

    return rear_x, rear_y, front_x, front_y


def draw_reference_path(ax):
    """绘制参考路径"""
    t = np.linspace(0, 10, 150)
    ref_x = t
    ref_y = 4.5 + 0.6 * np.sin(0.4 * t)  # 平缓的正弦曲线

    ax.plot(ref_x, ref_y, "g--", linewidth=2.5, label="参考路径", zorder=5)

    return ref_x, ref_y


def draw_rear_wheel_feedback_geometry(ax, rear_x, rear_y, vehicle_yaw, ref_x, ref_y):
    """绘制 Rear Wheel Feedback 几何关系"""

    # 找到后轴中心到参考路径的最近点
    distances = np.sqrt((ref_x - rear_x) ** 2 + (ref_y - rear_y) ** 2)
    nearest_idx = np.argmin(distances)
    nearest_x = ref_x[nearest_idx]
    nearest_y = ref_y[nearest_idx]

    # 计算参考路径在该点的切线方向（航向）
    if nearest_idx < len(ref_x) - 1:
        dx = ref_x[nearest_idx + 1] - ref_x[nearest_idx]
        dy = ref_y[nearest_idx + 1] - ref_y[nearest_idx]
    else:
        dx = ref_x[nearest_idx] - ref_x[nearest_idx - 1]
        dy = ref_y[nearest_idx] - ref_y[nearest_idx - 1]

    path_yaw = np.arctan2(dy, dx)

    # 计算曲率（使用数值方法）
    if 1 < nearest_idx < len(ref_x) - 2:
        # 使用三点计算曲率
        p1 = np.array([ref_x[nearest_idx - 1], ref_y[nearest_idx - 1]])
        p2 = np.array([ref_x[nearest_idx], ref_y[nearest_idx]])
        p3 = np.array([ref_x[nearest_idx + 1], ref_y[nearest_idx + 1]])

        # 计算曲率 κ = |det(v1, v2)| / |v1|^3
        v1 = p2 - p1
        v2 = p3 - p2
        curvature = abs(np.cross(v1, v2)) / (np.linalg.norm(v1) ** 3 + 1e-6)
    else:
        curvature = 0.0

    # 绘制最近点
    ax.plot(nearest_x, nearest_y, "go", markersize=10, zorder=10, markeredgecolor="darkgreen", markeredgewidth=2)
    ax.text(
        nearest_x - 0.3,
        nearest_y - 0.7,
        "最近点 P",
        fontsize=11,
        ha="center",
        bbox={"boxstyle": "round,pad=0.3", "facecolor": "white", "edgecolor": "green", "linewidth": 2},
    )

    # 绘制横向误差 e_d（从后轴中心到最近点）
    ax.plot([rear_x, nearest_x], [rear_y, nearest_y], "r-", linewidth=3, label="横向误差 $e_d$", zorder=8)

    # 横向误差标注（调整到线段上方）
    mid_x = (rear_x + nearest_x) / 2
    mid_y = (rear_y + nearest_y) / 2
    # 计算垂直于横向误差线的方向
    error_angle = np.arctan2(nearest_y - rear_y, nearest_x - rear_x)
    perpendicular = error_angle + np.pi / 2
    offset = 0.4
    label_x = mid_x + offset * np.cos(perpendicular)
    label_y = mid_y + offset * np.sin(perpendicular)
    ax.text(
        label_x,
        label_y,
        "$e_d$",
        fontsize=16,
        color="red",
        fontweight="bold",
        bbox={"boxstyle": "round,pad=0.4", "facecolor": "yellow", "edgecolor": "red", "linewidth": 2, "alpha": 0.8},
    )

    # 绘制车辆航向线（从后轴中心）
    heading_length = 3.5
    heading_end_x = rear_x + heading_length * np.cos(vehicle_yaw)
    heading_end_y = rear_y + heading_length * np.sin(vehicle_yaw)
    ax.plot([rear_x, heading_end_x], [rear_y, heading_end_y], "b-", linewidth=2.5, label="车辆航向 $\\psi$", zorder=8)
    ax.arrow(
        heading_end_x - 0.3 * np.cos(vehicle_yaw),
        heading_end_y - 0.3 * np.sin(vehicle_yaw),
        0.3 * np.cos(vehicle_yaw),
        0.3 * np.sin(vehicle_yaw),
        head_width=0.25,
        head_length=0.15,
        fc="blue",
        ec="blue",
        linewidth=2,
        zorder=8,
    )

    # 绘制路径切线方向（从后轴中心，与车辆航向形成对比）
    path_length = 3.5
    path_end_x = rear_x + path_length * np.cos(path_yaw)
    path_end_y = rear_y + path_length * np.sin(path_yaw)
    ax.plot([rear_x, path_end_x], [rear_y, path_end_y], "g-", linewidth=2.5, label="路径切线 $\\psi_{ref}$", zorder=8)
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

    # 计算航向误差 e_φ
    e_phi = vehicle_yaw - path_yaw
    while e_phi > np.pi:
        e_phi -= 2 * np.pi
    while e_phi < -np.pi:
        e_phi += 2 * np.pi

    # 绘制航向误差角度弧（从后轴中心）
    arc_radius = 1.8
    angle1_deg = np.degrees(path_yaw)
    angle2_deg = np.degrees(vehicle_yaw)

    # 在后轴中心绘制航向误差弧线
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

    # 航向误差标注（优化位置）
    arc_mid_angle = (vehicle_yaw + path_yaw) / 2
    label_x = rear_x + (arc_radius + 0.7) * np.cos(arc_mid_angle)
    label_y = rear_y + (arc_radius + 0.7) * np.sin(arc_mid_angle)
    ax.text(
        label_x,
        label_y,
        "$e_\\phi$",
        fontsize=16,
        color="purple",
        fontweight="bold",
        bbox={"boxstyle": "round,pad=0.4", "facecolor": "yellow", "edgecolor": "purple", "linewidth": 2, "alpha": 0.8},
    )

    # 绘制曲率指示（在最近点处绘制曲率圆）
    if abs(curvature) > 0.01:
        radius = 1.0 / curvature  # 曲率半径

        # 限制显示的曲率半径，避免过大
        if abs(radius) < 20:
            # 曲率中心在路径切线的法向方向
            normal_angle = path_yaw + np.pi / 2
            center_x = nearest_x + radius * np.cos(normal_angle)
            center_y = nearest_y + radius * np.sin(normal_angle)

            # 绘制小段曲率圆弧（更长的弧段）
            arc_span = 50  # 度
            start_angle_deg = np.degrees(np.arctan2(nearest_y - center_y, nearest_x - center_x))
            curvature_arc = Arc(
                (center_x, center_y),
                2 * abs(radius),
                2 * abs(radius),
                angle=0,
                theta1=start_angle_deg - arc_span / 2,
                theta2=start_angle_deg + arc_span / 2,
                color="orange",
                linewidth=2.5,
                linestyle="--",
                zorder=6,
            )
            ax.add_patch(curvature_arc)

            # 绘制曲率半径线（从曲率中心到最近点）
            ax.plot(
                [center_x, nearest_x],
                [center_y, nearest_y],
                "orange",
                linewidth=1.5,
                linestyle=":",
                alpha=0.6,
                zorder=6,
            )

            # 曲率标注（调整位置）
            ax.text(
                nearest_x + 0.5,
                nearest_y + 1.0,
                f"$\\kappa$ = {curvature:.3f}",
                fontsize=11,
                color="orange",
                fontweight="bold",
                bbox={"boxstyle": "round,pad=0.3", "facecolor": "white", "edgecolor": "orange", "linewidth": 1.5},
            )

    return e_phi, distances[nearest_idx], curvature


def main():
    """主函数"""
    fig, ax = plt.subplots(figsize=(14, 8))

    # 车辆位置和姿态（优化位置使得几何关系更清晰）
    vehicle_x = 5.5
    vehicle_y = 5.5
    vehicle_yaw = np.deg2rad(10)  # 略微偏向右上，产生更明显的航向误差
    wheelbase = 2.5

    # 绘制参考路径
    ref_x, ref_y = draw_reference_path(ax)

    # 绘制车辆
    rear_x, rear_y, front_x, front_y = draw_vehicle(ax, vehicle_x, vehicle_y, vehicle_yaw, wheelbase=wheelbase)

    # 绘制 Rear Wheel Feedback 几何关系
    e_phi, e_d, kappa = draw_rear_wheel_feedback_geometry(ax, rear_x, rear_y, vehicle_yaw, ref_x, ref_y)

    # 添加控制律公式（调整位置和格式）
    formula_text = (
        "Rear Wheel Feedback 控制律:\n"
        r"$\dot{\psi} = \frac{v_r \cdot \kappa \cdot \cos(e_\phi)}{1 - \kappa \cdot e_d} - "
        r"K_\theta \cdot |v_r| \cdot e_\phi - K_e \cdot v_r \cdot "
        r"\frac{\sin(e_\phi)}{e_\phi} \cdot e_d$"
        "\n\n"
        r"$\delta = \arctan\left(\frac{L \cdot \dot{\psi}}{v_r}\right)$"
    )
    ax.text(
        0.3,
        7.3,
        formula_text,
        fontsize=10.5,
        bbox={"boxstyle": "round,pad=0.5", "facecolor": "lightcyan", "edgecolor": "black", "linewidth": 2},
        verticalalignment="top",
    )

    # 添加参数说明（优化格式）
    param_text = (
        "$e_\\phi$: 航向误差 (车辆航向与路径切线的夹角)\n"
        "$e_d$: 横向误差 (后轴中心到参考路径的距离)\n"
        "$\\kappa$: 路径曲率 (在最近点处)\n"
        "$v_r$: 车辆速度\n"
        "$K_\\theta$: 航向误差增益系数\n"
        "$K_e$: 横向误差增益系数\n"
        "$L$: 车辆轴距 (WB)\n"
        "$\\dot{\\psi}$: 横摆角速度"
    )
    ax.text(
        0.3,
        3.5,
        param_text,
        fontsize=9.5,
        bbox={"boxstyle": "round,pad=0.5", "facecolor": "lightyellow", "edgecolor": "black", "linewidth": 1.5},
        verticalalignment="top",
    )

    # 设置坐标轴
    ax.set_xlim(-1, 11)
    ax.set_ylim(0, 8)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("X (m)", fontsize=12)
    ax.set_ylabel("Y (m)", fontsize=12)
    ax.set_title("Rear Wheel Feedback 控制器几何原理图", fontsize=16, fontweight="bold", pad=20)

    # 添加图例
    ax.legend(loc="upper right", fontsize=11, framealpha=0.9)

    plt.tight_layout()

    # 保存图片
    output_path = (
        "/home/yzc/projects/MotionPlanning/docs/control/image/"
        "RearWheelFeedback/rear_wheel_feedback_geometry_diagram.png"
    )
    plt.savefig(output_path, dpi=300, bbox_inches="tight")
    print(f"图片已保存到: {output_path}")

    plt.show()


if __name__ == "__main__":
    main()
