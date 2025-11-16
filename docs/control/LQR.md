## LQR

### 车辆运动学模型

$$
\dot{e}_d = v \sin e_{phi}
$$

当 $e_{phi}$ 较小时，$\sin e_{phi} \approx e_{phi}$，因此有：

$$
\dot{e}_d = v  e_{phi}
$$

同时在运动学模型中，有：

$$
\dot{e}_{phi} = \frac{v}{L} \tan \delta_f
$$

首先定义状态变量：

$$
\mathbf{x}_{t} = 
\begin{bmatrix}
e_d \\
\dot{e}_d \\
e_{phi} \\
\dot{e}_{phi}
\end{bmatrix}
$$

并定义控制输入为：

$$
\mathbf{u}_{t} = \tan \delta_f
$$

则误差状态方程可写为：

$$
\dot{\mathbf{x}_{t + 1}} = \mathbf{A} \mathbf{x}_{t} + \mathbf{B} \mathbf{u}_{t}
$$

其中：

$$
\mathbf{A} = 
\begin{bmatrix}
1 & dT & 0 & 0 \\
0 & 0 & v & 0 \\
0 & 0 & 1 & dT \\
0 & 0 & 0 & 0
\end{bmatrix},
\quad
\mathbf{B} = 
\begin{bmatrix}
0\\
0 \\
0  \\
\frac{v}{L}
\end{bmatrix}
$$

### LQR问题求解

迭代求解线性二次型调节器（LQR）问题，目标是找到一个最优控制策略 $\mathbf{u}_{t}$，使得以下性能指标最小化：

$$
J = \sum_{t=0}^{\infty} (\mathbf{x}_{t}^T \mathbf{Q} \mathbf{x}_{t} + \mathbf{u}_{t}^T \mathbf{R} \mathbf{u}_{t})
$$

其中，$\mathbf{Q}$ 和 $\mathbf{R}$ 分别是状态和控制输入的权重矩阵。
通过求解离散时间代数Riccati方程，可以得到最优反馈增益矩阵 $\mathbf{K}$，从而确定最优控制策略：

$$
\mathbf{u}_{t} = - \mathbf{K} \mathbf{x}_{t}
$$      

其中，反馈增益矩阵 $\mathbf{K}$ 由以下方程计算得到：

$$
\mathbf{K} = (\mathbf{R} + \mathbf{B}^T \mathbf{P} \mathbf{B})^{-1} \mathbf{B}^T \mathbf{P} \mathbf{A}
$$

这里，$\mathbf{P}$ 是通过迭代Riccati方程得到的正定矩阵。
通过这种方式，LQR控制器能够有效地调节车辆的横向误差和航向误差，实现平稳的路径跟踪。

### 控制指令

最终的转向控制指令 $\delta_f$ 可通过以下关系计算得到：

$$
\delta_f = \delta_{feedforward} + \delta_{feedback}
$$

其中：

$$
\delta_{feedforward} = \arctan \frac{L}{R}
$$

$$
\delta_{feedback} = \arctan \mathbf{u}_{t}
$$

