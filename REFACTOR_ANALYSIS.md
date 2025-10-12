# PyPnC 重构分析报告

日期：2025-10-12

本文档为重构初步分析，包含文件映射、demo 脚本列表、共享工具候选、优先迁移建议。

## 仓库文件概览（按目录）

- Control/
  - config_control.py
  - draw_lqr.py
  - draw.py
  - LQR_Dynamics_Model.py
  - LQR_Kinematic_Model.py
  - MPC_Frenet_Frame.py
  - MPC_XY_Frame.py
  - Pure_Pursuit.py
  - Rear_Wheel_Feedback.py
  - Stanley.py
  - utils.py

- CurvesGenerator/
  - cubic_spline.py
  - draw.py
  - dubins_path.py
  - quartic_polynomial.py
  - quintic_polynomial.py
  - reeds_shepp.py

- HybridAstarPlanner/
  - astar.py
  - draw.py
  - hybrid_astar.py
  - hybrid_astar_with_trailer.py

- LatticePlanner/
  - draw.py
  - env.py
  - lattice_planner.py

其他：README.md, fig/, gif/

## 包/模块候选分组（建议）
- pypnc.models — 状态/控制/轨迹 dataclasses
- pypnc.utils — 数值/坐标/辅助函数
- pypnc.planners — Hybrid A*, Lattice, Curves (dubins/reeds)
- pypnc.controllers — Pure Pursuit, Stanley, LQR, MPC
- pypnc.visualization — draw/plot/animate

## 含演示入口（需要迁移到 demo/ 或移除的脚本）
- HybridAstarPlanner/*: astar.py, hybrid_astar.py, hybrid_astar_with_trailer.py, draw.py
- LatticePlanner/lattice_planner.py
- Control/*: LQR_*.py, Stanley.py, Rear_Wheel_Feedback.py, MPC_*.py, Pure_Pursuit.py, draw_lqr.py, utils.py（含 main）
- CurvesGenerator/*: draw.py, dubins_path.py, reeds_shepp.py, quintic_polynomial.py, cubic_spline.py

这些脚本通常包含 `if __name__ == '__main__'`，应迁移为 `examples/` 或 `pypnc.demo`。

## 共享工具候选
- `Control/utils.py` — 存在多种基础函数，应抽象并放入 `pypnc.utils`。
- CurvesGenerator 中对曲线计算/路径点生成的公共函数。

## 循环导入/耦合高风险点（初步）
- draw.py 文件通常同时依赖算法实现与 utils，可能引入循环导入。
- 控制模块直接引用绘图或读取全局配置（`config_control.py`）。建议：算法模块不依赖 matplotlib 或全局路径，仅返回数据结构。

## 优先迁移候选（按低阻力优先）
1. CurvesGenerator（独立、纯计算） — 适合先迁移到 `pypnc.planners.curves`。
2. Pure_Pursuit / Stanley（轻量、依赖少） — 迁移到 `pypnc.controllers`。
3. HybridAstar（较大、需分层） — 迁移后接入测试。

## 下一步建议（我将执行）
1. 创建包骨架 `pypnc/`，实现 `models.py`（dataclasses）和 `__init__.py`。  
2. 把 `CurvesGenerator/quintic_polynomial.py` 的核心函数封装为可导入的函数并写单元测试（下一步）。

---
报告生成自仓库文件扫描（2025-10-12）。
