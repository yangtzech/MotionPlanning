# Stanley 控制器

## 算法简介

Stanley 控制器是斯坦福大学为其无人车 Stanley 开发的路径跟踪算法，该算法在 2005 年 DARPA 挑战赛中帮助 Stanley 赢得冠军。该算法结合了横向误差和航向误差，能够有效地进行路径跟踪控制[@Thrun2006Stanley;@CodingHaJiMiZiDongJiaShiJiYuJiHeMoXingDeGenZongFangFastanleySuanFa]。

## 控制原理

### 核心思想

Stanley 控制器通过两个误差项来计算前轮转向角：

1. **航向误差校正项**：使车辆航向与参考轨迹切线方向对齐
   假设无横向误差，仅通过转动前轮，使得前路与轨迹切线一致即可跟上，因此可令 $|\delta_{\phi}(t)| = e_{\phi} $
2. **横向误差校正项**：消除车辆前轴中心到参考轨迹的横向距离

### 几何原理

下图展示了 Stanley 控制器的几何原理，其中 F 为车辆前轴中心，H 为参考轨迹上的最近点，C 为前轮方向与最近点切线方向的交点：

![Stanley 几何原理图](image/Stanley/stanley_geometry_diagram.png)


### 控制律

Stanley 控制器的转向角计算公式为：

$$
\delta(t) = -e_{\phi} \cdot \text{sign}(v) + \arctan\left(\frac{-k \cdot e_d}{|v|}\right)
$$

其中：

- $\delta(t)$ : 前轮转向角 (左正右负)
- $e_{\phi}$ : 航向误差 (车辆航向与参考轨迹切线方向的夹角，左正右负)
- $e_d$ : 横向误差 (车辆到参考轨迹的垂直距离，左正右负)
- $v$ : 车辆速度 (前进为正，倒车为负)
- $k$ : 横向误差增益系数
- $\text{sign}(v)$ : 速度的符号函数，用于处理倒车情况

## 代码实现

### 核心代码

完整的 `ComputeControlCommand` 方法实现：

[:material-file-code: 查看源代码](https://github.com/yangtzech/MotionPlanning/blob/master/src/pyauto/control/Stanley.py#L21-L41){ .md-button }

```python title="src/pyauto/control/Stanley.py"
--8<-- "src/pyauto/control/Stanley.py:compute_control_command"
```

### 实现细节

#### 1. 前轴中心计算

Stanley 控制器基于**前轴中心**而非车辆质心进行控制：

```python title="前轴中心计算"
--8<-- "src/pyauto/control/Stanley.py:front_axle_center"
```

其中 `WB` 是车辆轴距 (wheelbase)，从质心到前轴的距离。

#### 2. 双向行驶支持

通过 `np.sign(node.v)` 处理前进和倒车两种情况：

| 行驶方向 | 速度符号  | 航向误差项    | 效果                                |
| -------- | --------- | ------------- | ----------------------------------- |
| 前进     | $v > 0$ | $-e_{\phi}$ | 航向偏左时右打方向盘                |
| 倒车     | $v < 0$ | $+e_{\phi}$ | 航向偏左时左打方向盘 (符合倒车特性) |

#### 3. 横向误差校正

横向误差项 `arctan(-k * ed / |v|)` 的特点：

- **速度归一化**：除以速度使得横向误差校正与速度成反比，高速时减小校正幅度，提高稳定性
- **增益系数 k**：控制横向误差的校正强度
  - $k$ 过大：响应快但可能震荡
  - $k$ 过小：响应慢但平滑
- **反正切函数**：提供非线性校正，避免大误差时转角饱和

#### 4. 零速度保护

在实际应用中，应对零速度或极低速度情况进行保护：

```python title="零速度保护"
--8<-- "src/pyauto/control/Stanley.py:zero_speed_protection"
```

## 参数配置

### 配置类定义

```python title="src/pyauto/config/config.py"
--8<-- "src/pyauto/config/config.py:stanley_config"
```

### 参数调优指南

### 优点

1. **简单高效**：控制律简单，计算量小，易于实时实现
2. **实践验证**：在 DARPA 挑战赛中得到实际验证
3. **非线性特性**：反正切函数提供非线性响应，避免饱和

### 缺点

1. **低速问题**：极低速度时横向误差项分母接近零，需要特殊处理
2. **参数固定**：增益系数通常固定，无法自适应不同速度和曲率
3. **无前馈**：缺少对轨迹曲率的前馈补偿，可能存在稳态误差
4. **局部控制**：只考虑当前误差，不具备预测能力

## 使用示例

### 基本用法

```python
from pyauto.config.config import Config
from pyauto.control.Stanley import StanleyController
from pyauto.control.pid_speed_control import PIDSpeedController

# 1. 创建配置
config = Config()
config.stanley.k = 0.5  # 设置增益系数

# 2. 创建控制器
lat_controller = StanleyController(config)
lon_controller = PIDSpeedController(config)

# 3. 在控制循环中使用
for step in range(max_steps):
    # 计算控制命令
    cmd = lat_controller.ComputeControlCommand(current_node, reference_path)
  
    # 获取转向角和误差信息
    steering = cmd.steer
    lat_error = cmd.lat_error
    yaw_error = cmd.yaw_error
  
    # 应用控制命令
    vehicle.update(acceleration, steering)
```

### 使用工厂模式创建

```python
from pyauto.control.controller_factory import ControllerFactory
from pyauto.config.config import Config

config = Config()
config.stanley.k = 0.6

# 通过工厂创建控制器
controller = ControllerFactory.create("stanley", config)
```

## 常见问题

### Q1: 为什么要使用前轴中心而不是车辆质心？

**答**：Stanley 控制器的核心思想是让前轮指向目标路径。使用前轴中心可以更直接地控制前轮朝向，减少控制延迟。如果使用质心，需要额外考虑车辆的旋转特性。

### Q2: 如何处理极低速或静止情况？

**答**：在速度接近零时，横向误差项的分母会导致数值不稳定。建议:

```python
v_safe = max(abs(node.v), 0.001)  # 设置最小速度阈值
delta = -e_phi * np.sign(node.v) + math.atan2(-k * ed, v_safe)
```

### Q3: 为什么倒车时航向误差不取相反数？

**答**：代码中已通过 `np.sign(node.v)` 自动处理了倒车情况。倒车时 `v < 0`，`sign(v) = -1`，使得航向误差项的符号自动翻转，符合倒车时的转向逻辑。

### Q4: 增益系数 k 如何选择？

**答**：

- 初始值：从 k = 0.5 开始
- 震荡时：减小 k (0.3 ~ 0.4)
- 响应慢时：增大 k (0.6 ~ 0.8)
- 速度相关调节：高速时使用较小的 k

### Q5: Stanley 控制器能否处理大曲率路径？

**答**：Stanley 控制器能够跟踪大曲率路径，但由于缺少曲率前馈，在急转弯时可能会有较大的横向误差。如需更好的性能，可考虑：

- 降低车速
- 增大增益系数 k
- 引入前馈
## 仿真演示

下图展示了 Stanley 控制器的实际运行效果：

![Stanley 控制器仿真动画](image/Stanley/stanley.gif)