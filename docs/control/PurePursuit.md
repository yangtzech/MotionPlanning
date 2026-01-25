# Pure Pursuit 控制器

## 算法简介

Pure Pursuit（纯追踪）是一种经典的几何路径跟踪算法，最早由 Craig Coulter 于 1992 年在 CMU 提出。该算法通过在参考路径上选取一个前视点（look-ahead point），计算出使车辆能够到达该点的圆弧轨迹，从而确定转向角[@Coulter1992Implementation]。Pure Pursuit 算法简单直观，计算效率高，至今仍被广泛应用于自动驾驶和移动机器人领域。

## 控制原理

### 核心思想

Pure Pursuit 控制器的核心思想是：

1. **前视点选取**：在参考路径上选取一个距离车辆后轴中心为前视距离 $L_f$ 的目标点
2. **圆弧拟合**：假设车辆从当前位置沿圆弧轨迹行驶到前视点
3. **转向角计算**：根据几何关系计算出所需的前轮转向角

### 几何推导

设车辆后轴中心为原点，航向为 x 轴正方向，前视点相对于车辆的方位角为 $\alpha$，前视距离为 $L_f$，车辆轴距为 $L$（即 WB）。

根据圆弧几何关系，可以推导出曲率 $\kappa$：

$$
\kappa = \frac{2 \sin(\alpha)}{L_f}
$$

由自行车模型的运动学关系 $\kappa = \frac{\tan(\delta)}{L}$，可得前轮转向角：

$$
\delta = \arctan\left(\frac{2 L \sin(\alpha)}{L_f}\right)
$$

### 控制律

Pure Pursuit 控制器的转向角计算公式为：

$$
\delta = \arctan\left(\frac{2 \cdot WB \cdot \sin(\alpha)}{L_f}\right)
$$

其中：

- $\delta$ : 前轮转向角
- $WB$ : 车辆轴距 (wheelbase)
- $\alpha$ : 前视点相对于车辆的方位角，$\alpha = \arctan\left(\frac{y_t - y}{x_t - x}\right) - \psi$
- $L_f$ : 前视距离 (look-ahead distance)
- $(x, y, \psi)$ : 车辆当前位置和航向
- $(x_t, y_t)$ : 前视点坐标

### 前视距离

前视距离通常与车速相关，采用线性模型：

$$
L_f = k_f \cdot v + L_d
$$

其中：

- $k_f$ : 前视增益系数
- $v$ : 车辆当前速度
- $L_d$ : 基础前视距离

## 代码实现

### 核心代码

```python
class PurePursuitController(ControllerBase):
    def __init__(self, config: Config):
        super().__init__(config)
        self.config = config

    def ComputeControlCommand(self, node: Node, reference: PATH) -> ControlCommand:
        ref_path = reference
  
        # 1. 计算前视距离并找到前视点
        ind, Lf = ref_path.look_ahead_index(node)
        tx = ref_path.cx[ind]
        ty = ref_path.cy[ind]
  
        # 2. 计算前视点相对于车辆的方位角
        alpha = math.atan2(ty - node.y, tx - node.x) - node.yaw
  
        # 3. Pure Pursuit 控制律
        delta = math.atan2(2.0 * self.config.WB * math.sin(alpha), Lf)
  
        # 4. 计算误差信息（用于监控）
        ed, e_phi = ref_path.cal_ed_e_phi(node, ind)

        # 5. 限制转向角范围
        delta = self.ClampSteeringAngle(delta)

        return ControlCommand(
            steer=delta,
            target_ind=ind,
            lat_error=ed,
            yaw_error=e_phi,
        )
```

### 实现细节

#### 1. 前视点查找

前视点在参考路径上按照前视距离进行搜索：

```python
def look_ahead_index(self, node):
    if self.index_old is None:
        self.index_old = self.calc_nearest_ind(node)
  
    # 计算前视距离（与速度相关）
    Lf = config.kf * node.v + config.Ld
  
    # 沿路径搜索满足前视距离的点
    for ind in range(self.index_old, self.ind_end + 1):
        if self.calc_distance(node, ind) > Lf:
            self.index_old = ind
            return ind, Lf
  
    return self.ind_end, Lf
```

#### 2. 方位角计算

方位角 $\alpha$ 是前视点相对于车辆航向的角度差：

```python
alpha = math.atan2(ty - node.y, tx - node.x) - node.yaw
```

| 前视点位置 | $\alpha$ 符号 | 转向方向 |
| ---------- | --------------- | -------- |
| 车辆左侧   | $\alpha > 0$  | 左转     |
| 车辆右侧   | $\alpha < 0$  | 右转     |
| 正前方     | $\alpha = 0$  | 直行     |

#### 3. 速度自适应前视距离

前视距离与速度成线性关系，这使得：

- **高速时**：前视距离增大，轨迹更平滑，避免过度响应
- **低速时**：前视距离减小，跟踪更精确

```python
Lf = config.kf * node.v + config.Ld
```

## 参数配置

### 配置类定义

```python
class PurePursuitConfig:
    Ld = 2.6  # 基础前视距离 [m]
    kf = 0.1  # 前视增益系数 [s]
```

## 优缺点分析

### 优点

1. **简单直观**：基于几何关系，物理意义明确，易于理解和实现
2. **计算高效**：仅需简单的三角函数运算，适合实时控制
3. **稳定可靠**：在合适的参数下，控制稳定，不易震荡
4. **速度自适应**：前视距离随速度变化，自然适应不同速度

### 缺点

1. **参数敏感**：前视距离的选择对性能影响较大

   - 前视距离过小：跟踪精确但可能震荡
   - 前视距离过大：平滑但会「切弯」
2. **曲率限制**：在急转弯时可能无法跟踪

## 使用示例

### 基本用法

```python
from pyauto.config.config import Config
from pyauto.control.Pure_Pursuit import PurePursuitController
from pyauto.control.pid_speed_control import PIDSpeedController

# 1. 创建配置
config = Config()
config.Ld = 3.0  # 设置基础前视距离
config.kf = 0.1  # 设置前视增益

# 2. 创建控制器
lat_controller = PurePursuitController(config)
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
config.Ld = 3.0
config.kf = 0.1

# 通过工厂创建控制器
controller = ControllerFactory.create("pure_pursuit", config)
```

## 常见问题

### Q1: 前视距离应该如何选择？

**答**：前视距离的选择需要权衡跟踪精度和平滑性：

- **经验法则**：$L_f \approx 1 \sim 2$ 倍车长
- **低速场景**（如泊车）：$L_d = 1.5 \sim 2.5$ m
- **高速场景**（如高速公路）：$L_d = 3.0 \sim 5.0$ m
- **调试建议**：从较大值开始，逐步减小直到出现震荡，然后适当增大

### Q2: 为什么高速时车辆会「切弯」？

**答**：这是 Pure Pursuit 的固有特性。前视距离越大，算法会更早地开始转向，导致在弯道内侧「切弯」。解决方法：

- 减小前视距离（但可能影响稳定性）
- 降低 `kf` 系数
- 考虑使用 Stanley 或其他算法

### Q3: Pure Pursuit 能否支持倒车？

**答**：直接支持倒车。

### Q4: 为什么使用后轴中心而不是前轴中心？

**答**：Pure Pursuit 基于自行车模型推导，后轴中心是车辆的转向中心（瞬心在后轴延长线上）。使用后轴中心可以简化几何推导，且后轴轨迹更稳定，便于路径规划。

### Q5: 如何处理找不到前视点的情况？

**答**：当车辆接近路径终点时，可能找不到满足前视距离的点。常见处理方式：

```python
# 返回路径终点作为前视点
if ind >= self.ind_end:
    return self.ind_end, Lf
```

### Q6: Pure Pursuit 和 Stanley 有什么区别？

**答**：

| 特性     | Pure Pursuit | Stanley             |
| -------- | ------------ | ------------------- |
| 参考点   | 后轴中心     | 前轴中心            |
| 核心思想 | 追踪前视点   | 消除航向和横向误差  |
| 误差项   | 无显式误差项 | 航向误差 + 横向误差 |
| 参数     | 前视距离     | 增益系数 k          |
