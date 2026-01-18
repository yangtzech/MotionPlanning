import matplotlib.pyplot as plt
import numpy as np

# ...existing code...
# angles in radians and degrees
angles = np.linspace(0.0, np.deg2rad(50), 500)  # 0 ~ 50°
angles_deg = np.degrees(angles)

plt.rcParams["font.family"] = "JetBrains Mono + Microsoft YaHei"

# compute values
tan_vals = np.tan(angles)
err_abs = np.abs(tan_vals - angles)
max_err = np.max(err_abs)
max_err_angle = angles_deg[np.argmax(err_abs)]

# create subplots: top shows x and tan(x), bottom shows error
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 7), sharex=True)

# top: x and tan(x)
ax1.plot(angles_deg, angles, label="x (rad)", color="C0")
ax1.plot(angles_deg, tan_vals, label="tan(x)", color="C1")
ax1.set_ylabel("Value (rad)")
ax1.set_title("x 与 tan(x) 曲线（0 ~ π/4）")
ax1.grid(True)
ax1.legend()

# annotate sample points (0°, 15°, 30°, 45°)
sample_degs = [0.0, 15.0, 30.0, 45.0]
for sd in sample_degs:
    idx = np.argmin(np.abs(angles_deg - sd))
    ax1.scatter(angles_deg[idx], angles[idx], color="C0")
    ax1.scatter(angles_deg[idx], tan_vals[idx], color="C1")
    ax1.text(
        angles_deg[idx],
        tan_vals[idx],
        f"tan={tan_vals[idx]:.4f}\n x={angles[idx]:.4f}",
        fontsize=9,
        va="bottom",
        ha="left",
    )

# bottom: absolute error
ax2.plot(angles_deg, err_abs, label="|tan(x)-x|", color="C2")
ax2.scatter([max_err_angle], [max_err], color="red", zorder=3)
ax2.text(
    max_err_angle,
    max_err,
    f"  max={max_err:.3e} rad\n  at {max_err_angle:.1f}°",
    va="bottom",
)
ax2.set_xlabel("Angle (deg)")
ax2.set_ylabel("Absolute error (rad)")
ax2.set_title("绝对误差")
ax2.grid(True)
ax2.legend()

plt.tight_layout()
plt.show()
# ...existing code...
