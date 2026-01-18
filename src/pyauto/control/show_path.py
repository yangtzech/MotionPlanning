import math

import matplotlib.pyplot as plt

from .config_control import Config
from .path_tools import generate_path

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

x, y, yaw, cur, direct, path_x, path_y = generate_path(states, config.MAX_STEER, config.WB)

# 打印信息（保留原有代码）
for seg_id, (cx, cy, cyaw, ccurv, cdirect) in enumerate(zip(x, y, yaw, cur, direct)):
    for i in range(len(cx)):
        print(
            f"Segment {seg_id}, Point {i}: x={cx[i]:.2f}, y={cy[i]:.2f}, "
            f"yaw={cyaw[i]:.2f}, curvature={ccurv[i]:.4f}, direction={cdirect[i]}"
        )

# 可视化
plt.figure(figsize=(10, 8))

# 绘制路径
for seg_id, (cx, cy, cyaw, ccurv, cdirect) in enumerate(zip(x, y, yaw, cur, direct)):
    color = "blue" if cdirect[0] == 1 else "red"  # 前进蓝色，后退红色（假设 direction 为 1 或 -1）
    plt.plot(cx, cy, color=color, linewidth=2, label=f"Segment {seg_id} (dir={cdirect[0]})")

# 显示 yaw：在路径点上画箭头表示航向
for seg_id, (cx, cy, cyaw, ccurv, cdirect) in enumerate(zip(x, y, yaw, cur, direct)):
    for i in range(0, len(cx), max(1, len(cx) // 5)):  # 每段取少量点画箭头，避免拥挤
        arrow_len = 0.5 * config.WB * 0.1  # 箭头长度
        plt.arrow(
            cx[i],
            cy[i],
            arrow_len * math.cos(cyaw[i]),
            arrow_len * math.sin(cyaw[i]),
            head_width=0.5,
            head_length=0.5,
            fc="green",
            ec="green",
        )

# 显示 curv：在路径点上显示曲率值
for seg_id, (cx, cy, cyaw, ccurv, cdirect) in enumerate(zip(x, y, yaw, cur, direct)):
    for i in range(0, len(cx), max(1, len(cx) // 5)):  # 每段取少量点显示曲率
        plt.text(cx[i] + 0.5, cy[i] + 0.5, f"{ccurv[i]:.4f}", fontsize=10, color="purple")

plt.xlabel("X")
plt.ylabel("Y")
plt.title("Path Visualization: X, Y, Yaw, Direction, Curvature")
plt.legend()
plt.axis("equal")
plt.grid(True)
plt.show()
