# LQR

## 基于车辆运动学模型误差状态方程

### 车辆运动学模型

车辆行驶时横向和航向误差有如下关系：

$$
\dot{e}_d = v \sin e_{\psi}
$$

当 $e_{phi}$ 较小时，$\sin e_{\psi} \approx e_{\psi}$，因此有：

$$
\dot{e}_d = v  e_{\psi}
$$

同时在运动学模型中，有：

$$
\tan \delta_f = \frac{L}{R}
$$

又：

$$
v = R \omega
$$

所以

$$
\dot{e}_{\psi} = \omega  - \omega_{ref}  =  \frac{v}{L} \tan \delta_f - \kappa_d v
$$

### 误差状态方程

按照不同的控制目标，可以分别定义不同的状态变量

#### 误差

定义状态变量：

$$
\mathbf{x}_{t} = 
\begin{bmatrix}
e_d \\
e_{\psi} \\
\end{bmatrix}
$$

控制输入为：

$$
\mathbf{u}_{t} = \tan \delta_f
$$

扰动项为参考曲率：

$$
w_t = \kappa_d
$$

则可得到误差状态方程为：

$$
\mathbf{x}_{t + 1} = \mathbf{A} \mathbf{x}_{t} + \mathbf{B} \mathbf{u}_{t} + \mathbf{D} w_t
$$

其中：

$$
\mathbf{A} = 
\begin{bmatrix}
0 & v \\
0 & 0
\end{bmatrix}
$$

$$
\mathbf{B} = 
\begin{bmatrix}
0  \\
\frac{v}{L}
\end{bmatrix}
$$

$$
\mathbf{D} =  
\begin{bmatrix}
0 \\
-v
\end{bmatrix}
$$

#### 误差和误差的变化率

首先定义状态变量：

$$
\mathbf{x}_{t} = 
\begin{bmatrix}
e_d \\
\dot{e}_d \\
e_{\psi} \\
\dot{e}_{\psi}
\end{bmatrix}
$$

并定义控制输入为：

$$
\mathbf{u}_{t} = \tan \delta_f
$$

##### 方式1：

将上式简单变化可得到误差状态方程为：

$$
\mathbf{x}_{t + 1} = \mathbf{A} \mathbf{x}_{t} + \mathbf{B} \mathbf{u}_{t} + \mathbf{D} w_t
$$

其中：

$$
\mathbf{A} = 
\begin{bmatrix}
1 & dT & 0 & 0 \\
0 & 0 & v & 0 \\
0 & 0 & 1 & dT \\
0 & 0 & 0 & 0
\end{bmatrix}
$$

$$
\mathbf{B} = 
\begin{bmatrix}
0\\
0 \\
0  \\
\frac{v}{L}
\end{bmatrix}
$$

$$
\mathbf{D} =  
\begin{bmatrix}
0 \\
-v
\end{bmatrix}
$$

##### 方式2：

$$
\ddot{e}_d = v  \dot{e}_{\psi}
$$

$$
\ddot{e}_{\psi} = 0
$$

因此误差状态方程为：

$$
\begin{bmatrix}
\dot{e}_d\\
\ddot{e}_d \\
\dot{e}_{\psi} \\
\ddot{e}_{\psi}
\end{bmatrix}
 = 
 \begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & 0 & 0 & v \\
0 & 0 & 0 & 1 \\
0 & 0 & 0 & 0
\end{bmatrix} 
\begin{bmatrix}
e_d \\
\dot{e}_d \\
e_{\psi} \\
\dot{e}_{\psi}
\end{bmatrix}
$$

离散化处理后可得到：

$$
\begin{bmatrix}
{e}_d\\
\dot{e}_d \\
{e}_{\psi} \\
\dot{e}_{\psi}
\end{bmatrix}
 = 
 \begin{bmatrix}
1 & dT & 0 & 0 \\
0 & 1 & 0 & v dT \\
0 & 0 & 1 & dT \\
0 & 0 & 0 & 1
\end{bmatrix} 
\begin{bmatrix}
e_d \\
\dot{e}_d \\
e_{\psi} \\
\dot{e}_{\psi}
\end{bmatrix}
$$

其中第二行

$$
\dot{e}_d = \dot{e}_d + v dT \dot{e}_{\psi}
$$

可以变形为

$$
\dot{e}_d = v e_{\psi}
$$

其中第四行

$$
\dot{e}_{\psi} = \dot{e}_{\psi}
$$

可以代入

$$
\dot{e}_{\psi} = \frac{v}{L} \tan \delta_f - \kappa_d v
$$

整理后结果同方式1。

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

## 基于车辆侧向动力学模型误差状态方程

### 车辆侧向动力学模型

误差状态方程如下：

$$
\frac{d}{dt}
\begin{bmatrix}
e_1 \\
\dot{e}_1 \\
e_2 \\
\dot{e}_2
\end{bmatrix}
=
\begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & -\frac{2C_{\alpha f} + 2C_{\alpha r}}{mv_x} & \frac{2C_{\alpha f} + 2C_{\alpha r}}{m} & -\frac{2C_{\alpha f} l_f - 2C_{\alpha r} l_r}{mv_x} \\
0 & 0 & 0 & 1 \\
0 & -\frac{2C_{\alpha f} l_f - 2C_{\alpha r} l_r}{I_z v_x} & \frac{2C_{\alpha f} l_f - 2C_{\alpha r} l_r}{I_z} & -\frac{2C_{\alpha f} l_f^2 + 2C_{\alpha r} l_r^2}{I_z v_x}
\end{bmatrix}
\begin{bmatrix}
e_1 \\
\dot{e}_1 \\
e_2 \\
\dot{e}_2
\end{bmatrix}
+
\begin{bmatrix}
0 \\
\frac{2C_{\alpha f}}{m} \\
0 \\
\frac{2C_{\alpha f} l_f}{I_z}
\end{bmatrix}
\delta +
\begin{bmatrix}
0 \\
\frac{2C_{\alpha f} l_f - 2C_{\alpha r} l_r}{m v_x} \\
0 \\
\frac{2C_{\alpha f} l_f^2 + 2C_{\alpha r} l_r^2}{I_z v_x}
\end{bmatrix} \omega_{ref}
$$

反馈项：

$$
\delta_{\mathrm{fb}} = -Kx
$$

前馈项：

$$
\delta_{\mathrm{ff}}=\frac{L}{R}+K_{V}a_{y}-k_{3}\left[\frac{l_{r}}{R}-\frac{l_{f}}{2C_{\alpha r}}\times\frac{mv_{x}^{2}}{Rl}\right]
$$



[1] 王国业, 江发潮. 车辆动力学及控制 第2版[M]. （美）拉杰什·拉贾马尼（Rajesh Rajamani）, ed.. 北京：机械工业出版社, 2018[2021-05-20]. http://book.ucdrs.superlib.net/views/specific/2929/bookDetail.jsp?dxNumber=000017310311&d=563C7A67DB80CB14DAC6F0EF53F0BA86&fenlei=1904050101.